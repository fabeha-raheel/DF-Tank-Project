import sys
import numpy as np
import threading

from PyQt5.QtWidgets import QApplication, QMainWindow, QSizePolicy, QVBoxLayout, QWidget
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer, pyqtSignal
from PyQt5.QtGui import QIcon,QPainter,QBrush,QPen,QPolygon, QColor, QFont
from PyQt5.QtCore import Qt, QPointF,QLineF
from PyQt5.QtChart import QScatterSeries, QPolarChart, QChart, QChartView, QValueAxis

# generate this file using the command 'pyrcc5 -o resources.py resources.qrc' in your terminal
import resources

from RadarPlot import *

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

        uic.loadUi('main_window.ui', self)  # Load the UI file
        self.setWindowTitle("DF Tank - Graphical Interface")

        self.show_page('splash_screen')
        self.timer = QTimer()
        self.progressValue = 0
        self.timer.timeout.connect(self.splash_screen_timer)
        self.timer.start(100)

        self.radarplot = RadarPlot(layout=self.plot_layout)

        self.initialization_complete.connect(lambda: self.show_page('visualization_page'))

        self.initialize()

    def show_page(self, page_name):
        target_page=self.stackedWidget.findChild(QWidget, page_name)
        self.stackedWidget.setCurrentWidget(target_page)

    def splash_screen_timer(self):
        self.progressValue += 1  # Adjust increment value as needed
        self.progressBar.setValue(self.progressValue)
        if self.progressValue >= 100:
            self.timer.stop()
            # self.show_page('visualization_page')
    
    def initialize_system(self):

        update_text = ""

        print("Connecting to FPGA...")
        update_text = update_text + "Connecting to FPGA...\n"
        self.progress_update_label.setText(update_text)
        self.fpga = Xilinx_Antenna()
        self.fpga.connect(port=FPGA_PORT, baud=FPGA_BAUD)

        if self.fpga.is_connected():
            update_text = update_text + "FPGA successfully connected!\n"
            self.progress_update_label.setText(update_text)

            update_text = update_text + "Acquiring Data...\n"
            self.progress_update_label.setText(update_text)

            print("Reading Antenna data....")
            data = self.fpga.read_data()
            # print("Received {} amplitudes.".format(len(data.amplitudes)))
            # print("f1: ", data.f1)
            # print("f2: ", data.f2)
            # print("Bandwidth: ", data.bandwidth)
            # print("Sample Size: ", data.n_samples)

            print(self.fpga.return_data())

            if data != -1:
                update_text = update_text + "DF Antenna Data successfully acquired!\n"
                self.progress_update_label.setText(update_text)
            else:
                update_text = update_text + "Error acquiring Antenna Data...\n"
                self.progress_update_label.setText(update_text)
        else:
            update_text = update_text + "Error connecting to FPGA!\n"
            self.progress_update_label.setText(update_text)
        
        print("Connecting to Pan Tilt Mechanism...")
        update_text = update_text + "Connecting to Pan Tilt Mechanism...\n"
        self.progress_update_label.setText(update_text)
        self.pantilt = PTZ_Controller()
        self.pantilt.connect(port=PTZ_PORT, baud=PTZ_BAUD)
        self.pantilt.mode_fast = False

        if self.pantilt.is_connected():
            update_text = update_text + "Pantilt successfully connected!\n"
            self.progress_update_label.setText(update_text)

            update_text = update_text + "Setting Pantilt to Home Position...\n"
            self.progress_update_label.setText(update_text)
        
        update_text = update_text + "System Initialization Complete!\n"
        self.progress_update_label.setText(update_text)
        print("System Initialization complete!")
        self.initialization_complete.emit()

    def initialize(self):
        self.gui_init_thread = threading.Thread(target=self.initialize_system, daemon=True)
        self.gui_init_thread.start()



if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.showMaximized()
    # main_window.show()
    sys.exit(app.exec_())
