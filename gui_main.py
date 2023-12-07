import sys
import numpy as np
import threading

from PyQt5.QtWidgets import QApplication, QMainWindow, QSizePolicy, QVBoxLayout, QWidget
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer, pyqtSignal
from PyQt5.QtGui import QIcon,QPainter,QBrush,QPen,QPolygon, QColor, QFont
from PyQt5.QtCore import Qt, QPointF,QLineF
# from PyQt5.QtChart import QScatterSeries, QPolarChart, QChart, QChartView, QValueAxis

# generate this file using the command 'pyrcc5 -o resources.py resources.qrc' in your terminal
import resources

# from RadarPlot import *
from mplwidget import *

from xilinx import *
from DF_Antenna_Data import *
from PTZ_Controller import *

FPGA_PORT = '/dev/ttyUSB0'      # port for Linux / Ubuntu
FPGA_BAUD = 115200

PTZ_PORT = '/dev/ttyCH341USB0'
PTZ_BAUD = 9600


class MainWindow(QMainWindow):

    initialization_complete = pyqtSignal()

    def __init__(self):
        super(MainWindow, self).__init__()

        uic.loadUi('main_window.ui', self)  # Load the UI file
        self.setWindowTitle("DF Tank - Graphical Interface")

        self.df_static = DF_Data_Static()
        self.df_dynamic = DF_Data_Dynamic()

        # self.radarplot = RadarPlot(layout=self.plot_layout)

        self.show_page('splash_screen')
        self.timer = QTimer()
        self.progressValue = 0
        self.timer.timeout.connect(self.splash_screen_timer)
        self.timer.start(100)

        self.initialization_complete.connect(self.goto_visualization)

        self.initialize()

    def show_page(self, page_name):
        target_page=self.stackedWidget.findChild(QWidget, page_name)
        self.stackedWidget.setCurrentWidget(target_page)

    def splash_screen_timer(self):
        self.progressValue += 1  # Adjust increment value as needed
        self.progressBar.setValue(self.progressValue)
        if self.progressValue >= 100:
            self.timer.stop()

    def goto_visualization(self):

        # show the visualization page
        self.show_page('visualization_page')
        # show the live spectrum by default
        self.TabWidget.setCurrentIndex(1)

        # start visualization timer
        self.live_spectrum_update = QTimer()
        self.live_spectrum_update.timeout.connect(self.redraw_spectrum)
        self.live_spectrum_update.start(100)

        self.spectrum_history_update = QTimer()
        self.spectrum_history_update.timeout.connect(self.update_scan_history)
        self.spectrum_history_update.start(100)


        # start the continuous data acquisition thread
        print("Starting Data Acquisition Thread...")
        self.daq_thread = threading.Thread(target=self.request_data_continuously, daemon=True)
        self.daq_thread.start()

    
    def initialize_system(self):

        update_text = ""

        print("Connecting to FPGA...")
        self.fpga = Xilinx_Antenna()
        self.fpga.connect(port=FPGA_PORT, baud=FPGA_BAUD)

        print("Connecting to Pan Tilt Mechanism...")
        self.pantilt = PTZ_Controller()
        self.pantilt.connect(port=PTZ_PORT, baud=PTZ_BAUD)
        self.pantilt.mode_fast = True

        if self.fpga.is_connected():

            print("Reading Antenna data....")
            data = self.fpga.get_static_data()

            if data != -1:
                self.df_static.f1 = data["_f1"]
                self.df_static.f2 = data["_f2"]
                self.df_static.n_samples = data["_n_samples"]
                self.df_dynamic.amplitudes = self.fpga.dynamic_data.amplitudes
                self.frequencies = list(np.arange(start=self.df_static.f1, stop=self.df_static.f2, step=((self.df_static.f2-self.df_static.f1)/self.df_static.n_samples)))

                print(self.df_static.__dict__)
            else:
                update_text = update_text + "Error acquiring Antenna Data...\n"
                self.progress_update_label.setText(update_text)
        else:
            update_text = update_text + "Error connecting to FPGA!\n"
            self.progress_update_label.setText(update_text)

        print("System Initialization complete!")

        self.initialization_complete.emit()

    def initialize(self):
        self.gui_init_thread = threading.Thread(target=self.initialize_system, daemon=True)
        self.gui_init_thread.start()

    def request_data_continuously(self):
        while True:
            if self.fpga.is_connected() and self.pantilt.is_connected():

                self.pantilt.set_pan_position(90)
                self.df_dynamic.angle_pt = 90
                self.df_dynamic.amplitudes = self.fpga.read_data()
                self.df_dynamic.angle_pt = 90

                self.pantilt.set_pan_position(67.5)
                self.df_dynamic.angle_pt = 67.5
                self.df_dynamic.amplitudes = self.fpga.read_data()
                self.df_dynamic.angle_pt = 67.5

                self.pantilt.set_pan_position(45)
                self.df_dynamic.angle_pt = 45
                self.df_dynamic.amplitudes = self.fpga.read_data()
                self.df_dynamic.angle_pt = 45

                self.pantilt.set_pan_position(22.5)
                self.df_dynamic.angle_pt = 22.5
                self.df_dynamic.amplitudes = self.fpga.read_data()
                self.df_dynamic.angle_pt = 22.5

                self.pantilt.set_pan_position(0)
                self.df_dynamic.angle_pt = 0
                self.df_dynamic.amplitudes = self.fpga.read_data()
                self.df_dynamic.angle_pt = 0

                self.pantilt.set_pan_position(337.5)
                self.df_dynamic.angle_pt = -22.5
                self.df_dynamic.amplitudes = self.fpga.read_data()
                self.df_dynamic.angle_pt = -22.5

                self.pantilt.set_pan_position(315)
                self.df_dynamic.angle_pt = -45
                self.df_dynamic.amplitudes = self.fpga.read_data()
                self.df_dynamic.angle_pt = -45

                self.pantilt.set_pan_position(292.5)
                self.df_dynamic.angle_pt = -67.5
                self.df_dynamic.amplitudes = self.fpga.read_data()
                self.df_dynamic.angle_pt = -67.5

                self.pantilt.set_pan_position(270)
                self.df_dynamic.angle_pt = -90
                self.df_dynamic.amplitudes = self.fpga.read_data()
                self.df_dynamic.angle_pt = -90

                self.pantilt.set_pan_position(292.5)
                self.df_dynamic.angle_pt = -67.5
                self.df_dynamic.amplitudes = self.fpga.read_data()
                self.df_dynamic.angle_pt = -67.5

                self.pantilt.set_pan_position(315)
                self.df_dynamic.angle_pt = -45
                self.df_dynamic.amplitudes = self.fpga.read_data()
                self.df_dynamic.angle_pt = -45

                self.pantilt.set_pan_position(337.5)
                self.df_dynamic.angle_pt = -22.5
                self.df_dynamic.amplitudes = self.fpga.read_data()
                self.df_dynamic.angle_pt = -22.5

                self.pantilt.set_pan_position(0)
                self.df_dynamic.angle_pt = 0
                self.df_dynamic.amplitudes = self.fpga.read_data()
                self.df_dynamic.angle_pt = 0

                self.pantilt.set_pan_position(22.5)
                self.df_dynamic.angle_pt = 22.5
                self.df_dynamic.amplitudes = self.fpga.read_data()
                self.df_dynamic.angle_pt = 22.5

                self.pantilt.set_pan_position(45)
                self.df_dynamic.angle_pt = 45
                self.df_dynamic.amplitudes = self.fpga.read_data()
                self.df_dynamic.angle_pt = 45

                self.pantilt.set_pan_position(67.5)
                self.df_dynamic.angle_pt = 67.5
                self.df_dynamic.amplitudes = self.fpga.read_data()
                self.df_dynamic.angle_pt = 67.5

    def get_data(self):
        self.data_thread = threading.Thread(target=self.request_data_continuously, daemon=True)

    def redraw_spectrum(self):
        self.plot_0.canvas.ax.cla()
        self.plot_0.setTitle("{}° Relative".format(self.df_dynamic.angle_pt), fontsize=10)
        self.plot_0.setBackgroundColor('k')
        self.plot_0.setLabels('Frequency (GHz)', 'Amplitude (dBm)', fontsize=10)
        # self.plot_0.setLimits()
        self.plot_0.canvas.ax.plot(self.frequencies, self.df_dynamic.amplitudes, 'y')
        self.plot_0.canvas.draw()

    def update_scan_history(self):
        current_angle = self.df_dynamic.angle_pt
        amplitudes = self.df_dynamic.amplitudes

        if current_angle == -90:
            self.plot_11.canvas.ax.cla()
            self.plot_11.setTitle("{}° Relative".format(current_angle), fontsize=10)
            self.plot_11.setBackgroundColor('k')
            self.plot_11.setLabels('Frequency (GHz)', 'Amplitude (dBm)', fontsize=10)
            self.plot_11.canvas.ax.plot(self.frequencies, amplitudes, 'y')
            self.plot_11.canvas.draw()

        elif current_angle == -67.5:
            self.plot_12.canvas.ax.cla()
            self.plot_12.setTitle("{}° Relative".format(current_angle), fontsize=10)
            self.plot_12.setBackgroundColor('k')
            self.plot_12.setLabels('Frequency (GHz)', 'Amplitude (dBm)', fontsize=10)
            self.plot_12.canvas.ax.plot(self.frequencies, amplitudes, 'y')
            self.plot_12.canvas.draw()

        elif current_angle == -45:
            self.plot_13.canvas.ax.cla()
            self.plot_13.setTitle("{}° Relative".format(current_angle), fontsize=10)
            self.plot_13.setBackgroundColor('k')
            self.plot_13.setLabels('Frequency (GHz)', 'Amplitude (dBm)', fontsize=10)
            self.plot_13.canvas.ax.plot(self.frequencies, amplitudes, 'y')
            self.plot_13.canvas.draw()

        elif current_angle == -22.5:
            self.plot_21.canvas.ax.cla()
            self.plot_21.setTitle("{}° Relative".format(current_angle), fontsize=10)
            self.plot_21.setBackgroundColor('k')
            self.plot_21.setLabels('Frequency (GHz)', 'Amplitude (dBm)', fontsize=10)
            self.plot_21.canvas.ax.plot(self.frequencies, amplitudes, 'y')
            self.plot_21.canvas.draw()

        elif current_angle == 0 or current_angle == 360:
            current_angle = 0
            self.plot_22.canvas.ax.cla()
            self.plot_22.setTitle("{}° Relative".format(current_angle), fontsize=10)
            self.plot_22.setBackgroundColor('k')
            self.plot_22.setLabels('Frequency (GHz)', 'Amplitude (dBm)', fontsize=10)
            self.plot_22.canvas.ax.plot(self.frequencies, amplitudes, 'y')
            self.plot_22.canvas.draw()

        elif current_angle == 22.5:
            self.plot_23.canvas.ax.cla()
            self.plot_23.setTitle("{}° Relative".format(current_angle), fontsize=10)
            self.plot_23.setBackgroundColor('k')
            self.plot_23.setLabels('Frequency (GHz)', 'Amplitude (dBm)', fontsize=10)
            self.plot_23.canvas.ax.plot(self.frequencies, amplitudes, 'y')
            self.plot_23.canvas.draw()

        elif current_angle == 45:
            self.plot_31.canvas.ax.cla()
            self.plot_31.setTitle("{}° Relative".format(current_angle), fontsize=10)
            self.plot_31.setBackgroundColor('k')
            self.plot_31.setLabels('Frequency (GHz)', 'Amplitude (dBm)', fontsize=10)
            self.plot_31.canvas.ax.plot(self.frequencies, amplitudes, 'y')
            self.plot_31.canvas.draw()

        elif current_angle == 67.5:
            self.plot_32.canvas.ax.cla()
            self.plot_32.setTitle("{}° Relative".format(current_angle), fontsize=10)
            self.plot_32.setBackgroundColor('k')
            self.plot_32.setLabels('Frequency (GHz)', 'Amplitude (dBm)', fontsize=10)
            self.plot_32.canvas.ax.plot(self.frequencies, amplitudes, 'y')
            self.plot_32.canvas.draw()

        elif current_angle == 90:
            self.plot_33.canvas.ax.cla()
            self.plot_33.setTitle("{}° Relative".format(current_angle), fontsize=10)
            self.plot_33.setBackgroundColor('k')
            self.plot_33.setLabels('Frequency (GHz)', 'Amplitude (dBm)', fontsize=10)
            self.plot_33.canvas.ax.plot(self.frequencies, amplitudes, 'y')
            self.plot_33.canvas.draw()

        else:
            print("Some other angle is specified... Cannot update spectrum history")


if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.showMaximized()
    # main_window.show()
    sys.exit(app.exec_())
