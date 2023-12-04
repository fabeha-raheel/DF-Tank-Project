import sys
import numpy as np
import threading

from PyQt5.QtWidgets import QApplication, QMainWindow, QSizePolicy, QVBoxLayout, QWidget
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer
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

PTZ_PORT = '/dev/ttyUSB0'
PTZ_BAUD = 9600


class MainWindow(QMainWindow):
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

    def show_page(self, page_name):
        target_page=self.stackedWidget.findChild(QWidget, page_name)
        self.stackedWidget.setCurrentWidget(target_page)

    def splash_screen_timer(self):
        self.progressValue += 1  # Adjust increment value as needed
        self.progressBar.setValue(self.progressValue)
        if self.progressValue >= 100:
            self.timer.stop()
            self.show_page('visualization_page')
    
    def initialize_antenna_system(self):

        print("Connecting to FPGA...")
        fpga = Xilinx_Antenna(port=FPGA_PORT, baud=FPGA_BAUD)
        fpga.connect()



if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.showMaximized()
    # main_window.show()
    sys.exit(app.exec_())
