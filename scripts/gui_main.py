import sys
import numpy as np
import random
from PyQt5.QtWidgets import QApplication, QMainWindow, QSizePolicy, QVBoxLayout, QWidget
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QIcon,QPainter,QBrush,QPen,QPolygon, QColor, QFont
from PyQt5.QtCore import Qt, QPointF,QLineF
from PyQt5.QtChart import QScatterSeries, QPolarChart, QChart, QChartView, QValueAxis

# generate this file using the command 'pyrcc5 -o resources.py resources.qrc' in your terminal
import resources

import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QSizePolicy, QVBoxLayout, QWidget
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from RadarPlot import *


class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        uic.loadUi('main_window.ui', self)  # Load the UI file
        self.setWindowTitle("DF Tank - Graphical Interface")

        self.show_page('splash_screen')
        self.timer = QTimer()
        self.progressValue = 0
        self.timer.timeout.connect(self.splash_screen_timer)
        self.timer.start(10)

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
    


if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
