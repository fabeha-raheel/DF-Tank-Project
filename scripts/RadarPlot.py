import typing
from PyQt5 import QtCore
from PyQt5.QtWidgets import QWidget
import sys
import random
import math

from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from PyQt5.QtChart import QScatterSeries, QPolarChart, QChart, QChartView, QValueAxis, QLineSeries

class RadarPlot(QWidget):

    def __init__(self, layout=None) -> None:
        super(RadarPlot, self).__init__()

        self.plotlayout = layout

        self.create_plot()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(2000)

        self.timer2 = QTimer()
        # self.timer.timeout.connect(self.update_plot)
        self.timer2.timeout.connect(self.update_scanner)
        self.timer2.start(100)

    def update_plot(self):
        self.scatterSeries.clear()  # Clear existing data
        for value in range(1, 50):
            self.scatterSeries.append(value, random.random() * 10)
        # No need to add a new series or set the chart again, just update the data

    def create_plot(self):

        self.polarChart = QPolarChart()
        self.chartView = QChartView()

        self.scatterSeries = QScatterSeries()

        for value in range(1,50):
            self.scatterSeries.append(value, random.random()*10)
            
        self.scatterSeries.setMarkerSize(10)
        self.scatterSeries.setColor(Qt.red)
        self.scatterSeries.setBorderColor(Qt.yellow)
        self.scatterSeries.setMarkerShape(QScatterSeries.MarkerShape.MarkerShapeCircle)
        self.scatterSeries.setName("Plot")
        self.polarChart.legend().setVisible(False)

        self.polarChart.addSeries(self.scatterSeries)
        self.polarChart.setContentsMargins(0,0,0,0)
        self.polarChart.setTheme(QChart.ChartTheme.ChartThemeDark)
        background_brush = QBrush(QColor(60, 60, 60))  # Adjust the color as needed
        # self.polarChart.setBackgroundBrush(background_brush)

        angularAxis = QValueAxis()
        angularAxis.setTickCount(9)
        angularAxis.setLabelFormat("%.2f")
        angularAxis.setShadesVisible(True)
        angularAxis.setShadesBrush(QBrush(QColor(230,230,255)))
        grid_line_pen = QPen(Qt.DashDotLine)
        angularAxis.setGridLinePen(grid_line_pen)
        # angularAxis.setMinorTickCount(5)
        # angularAxis.setTickCount(20)
        self.polarChart.addAxis(angularAxis, QPolarChart.PolarOrientation.PolarOrientationAngular)
        angularAxis.setRange(0,20)

        radialAxis = QValueAxis()
        radialAxis.setTickCount(5)
        radialAxis.setLabelFormat("%d")
        self.polarChart.addAxis(radialAxis, QPolarChart.PolarOrientation.PolarOrientationRadial)
        radialAxis.setRange(0, 10)

        self.chartView.setChart(self.polarChart)
        self.chartView.setFocusPolicy(Qt.NoFocus)
        self.chartView.setRenderHint(QPainter.Antialiasing)

        # vbox = QVBoxLayout()
        self.plotlayout.addWidget(self.chartView)
        # self.setLayout(self.plotlayout)

    def update_scanner(self):
        # Update the colors of the scanner sectors
        color = QColor(255, 0, 0)  # Initial color
        for sector in self.scanner_sectors:
            sector.updateColor(color)
            color = color.lighter(10)  # Adjust the lightening factor as needed
