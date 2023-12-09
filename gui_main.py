import sys
import numpy as np
import threading

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget
from PyQt5 import uic
from PyQt5.QtCore import QTimer, pyqtSignal

# generate this file using the command 'pyrcc5 -o resources.py resources.qrc' in your terminal
import resources

import rospy
from std_msgs.msg import Float64

from mplwidget import *
from mplradar import *

from xilinx import *
from DF_Antenna_Data import *
from PTZ_Controller import *

FPGA_PORT = '/dev/ttyUSB0'      # port for Linux / Ubuntu
FPGA_BAUD = 115200

PTZ_PORT = '/dev/ttyUSB1'
PTZ_BAUD = 9600


class MainWindow(QMainWindow):

    initialization_complete = pyqtSignal()

    def __init__(self):
        super(MainWindow, self).__init__()

        uic.loadUi('main_test.ui', self)  # Load the UI file
        self.setWindowTitle("DF Tank - Graphical Interface")

        self.df_data = DF_Data()

        self.show_page('splash_screen')
        self.timer = QTimer()
        self.progressValue = 0
        self.timer.timeout.connect(self.splash_screen_timer)
        self.timer.start(100)

        self.initialization_complete.connect(self.goto_visualization)

        self.cycle_complete = True

        self.init_ros_heading_subscriber()

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

        self.radar_plot_update = QTimer()
        self.radar_plot_update.timeout.connect(self.update_radar_plot)
        self.radar_plot_update.start(100)

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
                self.df_data.f1 = data["_f1"]
                self.df_data.f2 = data["_f2"]
                self.df_data.n_samples = data["_n_samples"]
                self.df_data.amplitudes = self.fpga.dynamic_data.amplitudes
                self.frequencies = list(np.arange(start=self.df_data.f1, stop=self.df_data.f2, step=((self.df_data.f2-self.df_data.f1)/self.df_data.n_samples)))

                self.df_data.angle_pt = 0

                self.df_data.initialize_matrix()

                self.df_data.matrix[:, self.df_data.current_sector] = self.df_data.amplitudes

                self.radar_plot.set_colorbar(self.frequencies)

            else:
                update_text = update_text + "Error acquiring Antenna Data...\n"
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
                self.df_data.angle_pt = 90
                self.df_data.amplitudes = self.fpga.read_data()
                self.df_data.angle_pt = 90
                self.df_data.matrix[:, self.df_data.current_sector] = self.df_data.amplitudes

                self.pantilt.set_pan_position(67.5)
                self.df_data.angle_pt = 67.5
                self.df_data.amplitudes = self.fpga.read_data()
                self.df_data.angle_pt = 67.5
                self.df_data.matrix[:, self.df_data.current_sector] = self.df_data.amplitudes

                self.pantilt.set_pan_position(45)
                self.df_data.angle_pt = 45
                self.df_data.amplitudes = self.fpga.read_data()
                self.df_data.angle_pt = 45
                self.df_data.matrix[:, self.df_data.current_sector] = self.df_data.amplitudes

                self.pantilt.set_pan_position(22.5)
                self.df_data.angle_pt = 22.5
                self.df_data.amplitudes = self.fpga.read_data()
                self.df_data.angle_pt = 22.5
                self.df_data.matrix[:, self.df_data.current_sector] = self.df_data.amplitudes

                self.pantilt.set_pan_position(0)
                self.df_data.angle_pt = 0
                self.df_data.amplitudes = self.fpga.read_data()
                self.df_data.angle_pt = 0
                self.df_data.matrix[:, self.df_data.current_sector] = self.df_data.amplitudes

                self.pantilt.set_pan_position(337.5)
                self.df_data.angle_pt = -22.5
                self.df_data.amplitudes = self.fpga.read_data()
                self.df_data.angle_pt = -22.5
                self.df_data.matrix[:, self.df_data.current_sector] = self.df_data.amplitudes

                self.pantilt.set_pan_position(315)
                self.df_data.angle_pt = -45
                self.df_data.amplitudes = self.fpga.read_data()
                self.df_data.angle_pt = -45
                self.df_data.matrix[:, self.df_data.current_sector] = self.df_data.amplitudes

                self.pantilt.set_pan_position(292.5)
                self.df_data.angle_pt = -67.5
                self.df_data.amplitudes = self.fpga.read_data()
                self.df_data.angle_pt = -67.5
                self.df_data.matrix[:, self.df_data.current_sector] = self.df_data.amplitudes

                self.pantilt.set_pan_position(270)
                self.df_data.angle_pt = -90
                self.df_data.amplitudes = self.fpga.read_data()
                self.df_data.angle_pt = -90
                self.df_data.matrix[:, self.df_data.current_sector] = self.df_data.amplitudes

                self.cycle_complete = True

                self.pantilt.set_pan_position(292.5)
                self.df_data.angle_pt = -67.5
                self.df_data.amplitudes = self.fpga.read_data()
                self.df_data.angle_pt = -67.5
                self.df_data.matrix[:, self.df_data.current_sector] = self.df_data.amplitudes

                self.pantilt.set_pan_position(315)
                self.df_data.angle_pt = -45
                self.df_data.amplitudes = self.fpga.read_data()
                self.df_data.angle_pt = -45
                self.df_data.matrix[:, self.df_data.current_sector] = self.df_data.amplitudes

                self.pantilt.set_pan_position(337.5)
                self.df_data.angle_pt = -22.5
                self.df_data.amplitudes = self.fpga.read_data()
                self.df_data.angle_pt = -22.5
                self.df_data.matrix[:, self.df_data.current_sector] = self.df_data.amplitudes

                self.pantilt.set_pan_position(0)
                self.df_data.angle_pt = 0
                self.df_data.amplitudes = self.fpga.read_data()
                self.df_data.angle_pt = 0
                self.df_data.matrix[:, self.df_data.current_sector] = self.df_data.amplitudes

                self.pantilt.set_pan_position(22.5)
                self.df_data.angle_pt = 22.5
                self.df_data.amplitudes = self.fpga.read_data()
                self.df_data.angle_pt = 22.5
                self.df_data.matrix[:, self.df_data.current_sector] = self.df_data.amplitudes

                self.pantilt.set_pan_position(45)
                self.df_data.angle_pt = 45
                self.df_data.amplitudes = self.fpga.read_data()
                self.df_data.angle_pt = 45
                self.df_data.matrix[:, self.df_data.current_sector] = self.df_data.amplitudes

                self.pantilt.set_pan_position(67.5)
                self.df_data.angle_pt = 67.5
                self.df_data.amplitudes = self.fpga.read_data()
                self.df_data.angle_pt = 67.5
                self.df_data.matrix[:, self.df_data.current_sector] = self.df_data.amplitudes

                
    def get_data(self):
        self.data_thread = threading.Thread(target=self.request_data_continuously, daemon=True)

    def ros_heading_cb(self, mssg):
        self.df_data.heading = mssg.data
        # rospy.loginfo("Heading %s", mssg.data)

    def init_ros_heading_subscriber(self):
        rospy.init_node('compass_data', anonymous=True)
        rospy.Subscriber('/mavros/global_position/compass_hdg',Float64, self.ros_heading_cb)

    def update_radar_plot(self):

        self.tank_heading.setText(str(self.get_tank_heading())+" °N")
        self.antenna_heading.setText(str(self.get_antenna_heading())+" °N")
            
        if self.cycle_complete:
            # get updated data
            self.df_data.normalize_matrix()
            updated_data = self.df_data.radar_plot_data()
            freqs = updated_data[0]
            angles = updated_data[1]
            amps = updated_data[2]

            for i in range(len(angles)):
                if angles[i] < 0:
                    angles[i] = 360 + angles[i]

            self.radar_plot.canvas.ax.cla()
            # self.radar_plot.plotScatterPoints(angles, amps, size=100, color='#1ba3b3', marker='o', label='Scatter Points', edgecolors='white')
            self.radar_plot.plotScatterPoints(angles, amps, size=100, color=freqs, marker='o', label='Scatter Points', edgecolors='white')
            self.radar_plot.canvas.draw()
    
    def redraw_spectrum(self):
        self.plot_0.canvas.ax.cla()
        self.plot_0.setTitle("{}° Relative".format(self.df_data.angle_pt), fontsize=10)
        self.plot_0.setBackgroundColor('k')
        self.plot_0.setLabels('Frequency (GHz)', 'Amplitude (dBm)', fontsize=10)
        # self.plot_0.setLimits()
        self.plot_0.canvas.ax.plot(self.frequencies, self.df_data.amplitudes, 'y')
        self.plot_0.canvas.draw()

    def update_scan_history(self):

        self.plot_matrix = [self.plot_11, self.plot_12, self.plot_13, self.plot_21, self.plot_22, self.plot_23, self.plot_31, self.plot_32, self.plot_33]

        for i in range(self.df_data.n_sectors+1):
            amplitudes = self.df_data.matrix[:, i]
            plot = self.plot_matrix[i]
            plot.canvas.ax.cla()
            plot.setTitle("{}° Relative".format((i*self.df_data.beam_width)+self.df_data.alpha1), fontsize=10)
            plot.setBackgroundColor('k')
            plot.setLabels('Frequency (GHz)', 'Amplitude (dBm)', fontsize=5)
            plot.canvas.ax.plot(self.frequencies, amplitudes, 'y')
            plot.canvas.draw()

    def get_tank_heading(self):
        if self.df_data.heading > 180:
            return int(self.df_data.heading)-360
        else:
            return int(self.df_data.heading)
        
    def get_antenna_heading(self):
        # antenna_heading = (self.df_data.heading + self.df_data.angle_pt)
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
    # main_window.showMaximized()
    main_window.show()
    sys.exit(app.exec_())
