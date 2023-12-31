#!/usr/bin/python3

import sys
import numpy as np
import threading
import websocket
import json

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget
from PyQt5 import uic
from PyQt5.QtCore import QTimer, pyqtSignal, pyqtSlot, QThreadPool, QRunnable

# generate this file using the command 'pyrcc5 -o resources.py resources.qrc' in your terminal
import resources

import rospy
from std_msgs.msg import Float64
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode, SetModeRequest

from mplwidget import *
from mplradar import *

from xilinx import *
from DF_Antenna_Data import *
from PTZ_Controller import *

FPGA_PORT = '/dev/ttyUSB1'
FPGA_BAUD = 115200

# PTZ_PORT = '/dev/ttyCH341USB0'
PTZ_PORT = '/dev/ttyUSB0'
PTZ_BAUD = 9600

# websocket_url = "ws://localhost:8000/"
websocket_url = "ws://192.168.0.116:8000/"

class Worker(QRunnable):
    
	def __init__(self, function, *args, **kwargs):
		super(Worker, self).__init__()
		self.function = function
		self.args = args
		self.kwargs = kwargs

	@pyqtSlot()
	def run(self):

		self.function(*self.args, **self.kwargs)


class MainWindow(QMainWindow):

    initialization_complete = pyqtSignal()

    def __init__(self, ws_url="ws://localhost:8000/"):
        super(MainWindow, self).__init__()

        uic.loadUi('gui.ui', self)  # Load the UI file
        self.setWindowTitle("DF Tank - Graphical Interface")

        self.df_data = DF_Data()

        self.ws_url = ws_url

        self.plot_matrix = [self.plot_11, self.plot_12, self.plot_13, self.plot_21, self.plot_22, self.plot_23, self.plot_31, self.plot_32, self.plot_33]

        self.show_page('splash_screen')
        self.timer = QTimer()
        self.progressValue = 0
        self.timer.timeout.connect(self.splash_screen_timer)
        self.timer.start(100)

        self.initialization_complete.connect(self.goto_visualization)

        self.init = False
        self.initialize()

        self.threadpool = QThreadPool()
        self.run_threads = True

        self._throttle_channel = 1
        self._steering_channel = 0
        self._forward = False
        self._backward = False
        self._right = False
        self._left = False
        self._stop = False

        # self.forward_button.pressed.connect(self.move_forward)
        # self.forward_button.released.connect(self.stop)
        # self.backward_button.pressed.connect(self.move_backward)
        # self.backward_button.released.connect(self.stop)
        # self.left_button.pressed.connect(self.turn_left)
        # self.left_button.released.connect(self.stop)
        # self.right_button.pressed.connect(self.turn_right)
        # self.right_button.released.connect(self.stop)
        # self.stop_button.clicked.connect(self.stop)

        # self.arm_button.setChecked(False)
        # self.arm_button.clicked.connect(self.arm_disarm_ugv)
        # self.manual_button.clicked.connect(self.ugv_mode)

        self.initialize()

    def closeEvent(self, event):
        self.run_threads = False
    
    def show_page(self, page_name):
        target_page=self.stackedWidget.findChild(QWidget, page_name)
        self.stackedWidget.setCurrentWidget(target_page)

    def splash_screen_timer(self):
        self.progressValue += 1  # Adjust increment value as needed
        self.progressBar.setValue(self.progressValue)
        if self.progressValue >= 100:
            self.timer.stop()

    def initialize(self):
        self.ws_thread = threading.Thread(target=self.initialize_ws_client, daemon=True)
        self.ws_thread.start()

    def initialize_ws_client(self):
        self.ws = websocket.WebSocketApp(self.ws_url,
                                on_message=self.on_message,
                                on_error=self.on_error,
                                on_close=self.on_close)
        
        # Set the on_open callback to handle the opening of the connection
        self.ws.on_open = self.on_open

        # Start the WebSocket connection (this will run the on_open callback)
        self.ws.run_forever()

    def on_message(self, ws, message):
        # print(f"Received message: {message}")
        data = json.loads(message)

        if self.init:
            self.extract_data(data)
            print("Done saving Antenna data")
        else:
            self.init = True
            self.extract_static_data(data)
            print("System Initialization complete!")
            self.initialization_complete.emit()

    def on_error(self, ws, error):
        print(f"Error: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        print(f"Closed with status code {close_status_code}: {close_msg}")

    def on_open(self, ws):
        print("WebSocket connection opened")

    def extract_static_data(self, data):
        self.df_data.f1 = data['f1']
        self.df_data.f2 = data['f2']
        self.df_data.n_samples = data['n_samples']
        self.df_data.beam_width = data['beam_width']
        self.df_data.amplitudes = data['amplitudes']
        self.df_data.angle_pt = data['angle_pt']
        self.df_data.heading = data['heading']

        self.frequencies_range_hz = np.arange(start=self.df_data.f1, stop=self.df_data.f2, step=((self.df_data.f2-self.df_data.f1)/self.df_data.n_samples))
        self.frequencies_range_Mhz = self.frequencies_range_hz / 1000000

        self.frequencies = list(self.frequencies_range_Mhz)

        self.avg_freqs = self.df_data.averaging(array=self.frequencies, N=10)

        self.df_data.initialize_matrix()
        self.df_data.matrix[:, self.df_data.current_sector] = self.df_data.amplitudes
        
        self.radar_plot.set_colorbar(self.frequencies)
    
    def extract_data(self, data):
        self.df_data.amplitudes = data['amplitudes']
        self.df_data.angle_pt = data['angle_pt']
        self.df_data.heading = data['heading']
        self.df_data.matrix[:, self.df_data.current_sector] = self.df_data.amplitudes

    def goto_visualization(self):

        # show the visualization page
        self.show_page('visualization_page')
        # show the live spectrum by default
        self.TabWidget.setCurrentIndex(0)

        self.RadarPlotThread = Worker(self.update_radar_plot)
        self.SpectrumPlotThread = Worker(self.redraw_spectrum)
        self.ScanHistoryThread = Worker(self.update_scan_history)

        self.initialize_plots()
        
        self.threadpool.start(self.RadarPlotThread)
        self.threadpool.start(self.SpectrumPlotThread)
        self.threadpool.start(self.ScanHistoryThread)

    def update_radar_plot(self):

        while self.run_threads:
            self.tank_heading.setText(str(self.get_tank_heading())+" °N")
            self.antenna_heading.setText(str(self.get_antenna_heading())+" °N")
                
            # get updated data
            self.df_data.normalize_matrix_byCol()
            updated_data = self.df_data.radar_plot_data()
            freqs = updated_data[0]
            angles = updated_data[1]
            amps = updated_data[2]

            for i in range(len(angles)):
                if angles[i] < 0:
                    angles[i] = 360 + angles[i]

            self.radar_plot.canvas.ax.cla()
            self.radar_plot.plotScatterPoints(angles, amps, size=100, color=freqs, marker='o', label='Scatter Points', edgecolors='white')
            self.radar_plot.canvas.draw()

            self.plot_0_degrees.canvas.ax.cla()
            # self.plot_0_degrees.canvas.ax.plot(self.frequencies, self.df_data.matrix[:, 4], 'y')
            self.plot_0_degrees.canvas.ax.plot(self.avg_freqs, self.df_data.averaging(self.df_data.matrix[:, 4]), 'y')
            self.plot_0_degrees.setLabels('Frequency (MHz)', 'Amplitude', fontsize=10)
            self.plot_0_degrees.canvas.draw()
                
            time.sleep(0.5)
    
    def redraw_spectrum(self):
        
        while self.run_threads:
            avg_amplitudes = self.df_data.averaging(array=self.df_data.amplitudes, N=10)
            
            self.plot_0.canvas.ax.cla()
            self.plot_0.setTitle("{}° Relative".format(self.df_data.angle_pt), fontsize=10)
            self.plot_0.setLabels('Frequency (MHz)', 'Amplitude (dBm)', fontsize=10)
            # self.plot_0.canvas.ax.plot(self.frequencies, self.df_data.amplitudes, 'y')
            self.plot_0.canvas.ax.plot(self.avg_freqs, avg_amplitudes, 'y')
            self.plot_0.canvas.draw()

            time.sleep(0.5)

    def update_scan_history(self):

        while self.run_threads:
            for i in range(self.df_data.n_sectors+1):
                amplitudes = self.df_data.matrix[:, i]
                avg_amplitudes = self.df_data.averaging(array=amplitudes, N=10)

                plot = self.plot_matrix[i]
                plot.canvas.ax.cla()
                plot.setTitle("{}° Relative".format((i*self.df_data.beam_width)+self.df_data.alpha1), fontsize=10)
                plot.setLabels('Frequency (MHz)', 'Amplitude (dBm)', fontsize=5)
                # plot.canvas.ax.plot(self.frequencies, amplitudes, 'y')
                plot.canvas.ax.plot(self.avg_freqs, avg_amplitudes, 'y')
                plot.canvas.draw()

                time.sleep(0.5)

    def initialize_plots(self):
        self.plot_0_degrees.setBackgroundColor('k')
        self.plot_0_degrees.canvasBackgroundColor('#53847F')
        self.plot_0_degrees.setTickcolor('#D9D9D9')
        
        self.plot_0.setBackgroundColor('k')

        for plot in self.plot_matrix:
            plot.setBackgroundColor('k')

    def get_tank_heading(self):
        if self.df_data.heading > 180:
            return int(self.df_data.heading)-360
        else:
            return int(self.df_data.heading)
        
    def get_antenna_heading(self):
        antenna_heading = (self.get_tank_heading() + self.df_data.angle_pt)

        if antenna_heading > 180:
            return int(antenna_heading-360)
        elif antenna_heading < -180:
            return int(antenna_heading + 360)
        else:
            return int(antenna_heading)
         

    


if __name__ == '__main__':

    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.showMaximized()
    # main_window.show()
    sys.exit(app.exec_())