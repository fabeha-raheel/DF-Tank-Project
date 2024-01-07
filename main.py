#!/usr/bin/python3

import sys
import numpy as np
import threading

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget
from PyQt5 import uic
from PyQt5.QtCore import QTimer, pyqtSignal, pyqtSlot, QThreadPool, QRunnable

# generate this file using the command 'pyrcc5 -o resources.py resources.qrc' in your terminal
import resources

import rospy
from std_msgs.msg import Float64
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool

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

    def __init__(self):
        super(MainWindow, self).__init__()

        uic.loadUi('gui.ui', self)  # Load the UI file
        self.setWindowTitle("DF Tank - Graphical Interface")

        self.df_data = DF_Data()

        self.plot_matrix = [self.plot_11, self.plot_12, self.plot_13, self.plot_21, self.plot_22, self.plot_23, self.plot_31, self.plot_32, self.plot_33]
        self.pan_sequence = [90, 67.5, 45, 22.5, 0, 337.5, 315, 292.5, 270, 292.5, 315, 337.5, 0, 22.5, 45, 67.5]

        self.show_page('splash_screen')
        self.timer = QTimer()
        self.progressValue = 0
        self.timer.timeout.connect(self.splash_screen_timer)
        self.timer.start(100)

        self.initialization_complete.connect(self.goto_visualization)

        self.cycle_complete = True

        self.threadpool = QThreadPool()
        self.run_threads = True

        self._throttle_channel = 1
        self._steering_channel = 0
        self._forward = False
        self._backward = False
        self._right = False
        self._left = False
        self._stop = False

        self.init_ros_heading_subscriber()
        self.init_ros_control_publisher()
        self.arm_ugv()

        self.forward_button.pressed.connect(self.move_forward)
        self.forward_button.released.connect(self.stop)
        self.backward_button.pressed.connect(self.move_backward)
        self.backward_button.released.connect(self.stop)
        self.left_button.pressed.connect(self.turn_left)
        self.left_button.released.connect(self.stop)
        self.right_button.pressed.connect(self.turn_right)
        self.right_button.released.connect(self.stop)
        self.stop_button.clicked.connect(self.stop)

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

        # start the continuous data acquisition thread
        print("Starting Data Acquisition Thread...")
        self.daq_thread = threading.Thread(target=self.request_data_continuously, daemon=True)
        self.daq_thread.start()

        self.controls_thread = threading.Thread(target=self.monitor_controls, daemon=True)
        self.controls_thread.start()
    
    def initialize_system(self):

        print("Connecting to FPGA...")
        self.fpga = Xilinx_Antenna()
        self.fpga.connect(port=FPGA_PORT, baud=FPGA_BAUD)

        print("Connecting to Pan Tilt Mechanism...")
        self.pantilt = PTZ_Controller()
        self.pantilt.connect(port=PTZ_PORT, baud=PTZ_BAUD)
        self.pantilt.mode_fast = True

        if self.fpga.is_connected():

            while True:
                print("Reading Antenna data....")
                data = self.fpga.get_static_data()

                if data != -1:
                    self.df_data.f1 = data["_f1"]
                    self.df_data.f2 = data["_f2"]
                    self.df_data.n_samples = data["_n_samples"]
                    self.df_data.amplitudes = self.fpga.dynamic_data.amplitudes

                if len(self.df_data.amplitudes) == self.df_data.n_samples:
                    break
            
            self.frequencies_range_hz = np.arange(start=self.df_data.f1, stop=self.df_data.f2, step=((self.df_data.f2-self.df_data.f1)/self.df_data.n_samples))

            self.frequencies_range_Mhz = self.frequencies_range_hz / 1000000

            self.frequencies = list(self.frequencies_range_Mhz)

            self.df_data.angle_pt = 0

            self.df_data.initialize_matrix()

            self.df_data.matrix[:, self.df_data.current_sector] = self.df_data.amplitudes

            self.radar_plot.set_colorbar(self.frequencies)

        print("System Initialization complete!")

        self.initialization_complete.emit()

    def initialize(self):
        self.gui_init_thread = threading.Thread(target=self.initialize_system, daemon=True)
        self.gui_init_thread.start()

    def request_data_continuously(self):
        
        while self.run_threads:

            for pan in self.pan_sequence:
                if self.fpga.is_connected() and self.pantilt.is_connected():
                    self.pantilt.set_pan_position(pan)
                    self.df_data.amplitudes = self.fpga.read_data()
                    if pan < 180:
                        self.df_data.angle_pt = pan
                    else:
                        self.df_data.angle_pt = pan - 360
                    self.df_data.matrix[:, self.df_data.current_sector] = self.df_data.amplitudes
                    if not self.run_threads:
                        break

    def ros_heading_cb(self, mssg):
        self.df_data.heading = mssg.data

    def init_ros_heading_subscriber(self):
        rospy.init_node('df_ugv', anonymous=True)
        rospy.Subscriber('/mavros/global_position/compass_hdg',Float64, self.ros_heading_cb)

    def init_ros_control_publisher(self):
        self.ugv_rc_override = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=1)

    def arm_ugv(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            armResponse = armService(True)
            rospy.loginfo(armResponse)
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)

    def update_radar_plot(self):

        while self.run_threads:
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
                self.radar_plot.plotScatterPoints(angles, amps, size=100, color=freqs, marker='o', label='Scatter Points', edgecolors='white')
                self.radar_plot.canvas.draw()

                self.plot_0_degrees.canvas.ax.cla()
                self.plot_0_degrees.canvas.ax.plot(self.frequencies, self.df_data.matrix[:, 4], 'y')
                self.plot_0_degrees.canvas.draw()
                
            time.sleep(0.5)
    
    def redraw_spectrum(self):
        
        while self.run_threads:
            self.plot_0.canvas.ax.cla()
            self.plot_0.setTitle("{}° Relative".format(self.df_data.angle_pt), fontsize=10)
            self.plot_0.setBackgroundColor('k')
            self.plot_0.setLabels('Frequency (MHz)', 'Amplitude (dBm)', fontsize=10)
            # self.plot_0.setLimits()
            self.plot_0.canvas.ax.plot(self.frequencies, self.df_data.amplitudes, 'y')
            self.plot_0.canvas.draw()

            time.sleep(0.5)

    def update_scan_history(self):

        while self.run_threads:
            for i in range(self.df_data.n_sectors+1):
                amplitudes = self.df_data.matrix[:, i]
                plot = self.plot_matrix[i]
                plot.canvas.ax.cla()
                plot.setTitle("{}° Relative".format((i*self.df_data.beam_width)+self.df_data.alpha1), fontsize=10)
                plot.setBackgroundColor('k')
                plot.setLabels('Frequency (MHz)', 'Amplitude (dBm)', fontsize=5)
                plot.canvas.ax.plot(self.frequencies, amplitudes, 'y')
                plot.canvas.draw()

                time.sleep(0.5)

    def initialize_plots(self):
        self.plot_0_degrees.setBackgroundColor('k')
        self.plot_0_degrees.canvasBackgroundColor('#53847F')
        self.plot_0_degrees.setLabels('Frequency (MHz)', 'Amplitude', fontsize=10)

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

    def move_forward(self):
        self._forward = True
        self._backward = False
        self._left = False
        self._right = False
        self._stop = False

    def move_backward(self):
        self._forward = False
        self._backward = True
        self._left = False
        self._right = False
        self._stop = False

    def turn_right(self):
        self._forward = False
        self._backward = False
        self._left = False
        self._right = True
        self._stop = False
    
    def turn_left(self):
        self._forward = False
        self._backward = False
        self._left = True
        self._right = False
        self._stop = False

    def stop(self):
        self._forward = False
        self._backward = False
        self._left = False
        self._right = False
        self._stop = True

    def monitor_controls(self):
        while self.run_threads:
            if self._forward:
                msg = OverrideRCIn()
                msg.channels[self._throttle_channel] = 1650
                msg.channels[self._steering_channel] = 1500
                self.ugv_rc_override.publish(msg)
                time.sleep(0.2)

            elif self._backward:
                msg = OverrideRCIn()
                msg.channels[self._throttle_channel] = 1350
                msg.channels[self._steering_channel] = 1500
                self.ugv_rc_override.publish(msg)
                time.sleep(0.2)

            elif self._left:
                msg = OverrideRCIn()
                msg.channels[self._throttle_channel] = 1500
                msg.channels[self._steering_channel] = 1350
                self.ugv_rc_override.publish(msg)
                time.sleep(0.2)

            elif self._right:
                msg = OverrideRCIn()
                msg.channels[self._throttle_channel] = 1500
                msg.channels[self._steering_channel] = 1650
                self.ugv_rc_override.publish(msg)
                time.sleep(0.2)

            elif self._stop:
                msg = OverrideRCIn()
                msg.channels[self._throttle_channel] = 1500
                msg.channels[self._steering_channel] = 1500
                self.ugv_rc_override.publish(msg)
                time.sleep(0.2)


if __name__ == '__main__':

    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.showMaximized()
    # main_window.show()
    sys.exit(app.exec_())