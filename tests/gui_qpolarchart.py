# Ref: https://www.jb51.net/article/270183.htm

import sys
import random
import math

from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from PyQt5.QtChart import QScatterSeries, QPolarChart, QChart, QChartView, QValueAxis

class MyPolarWindow(QWidget):
    def __init__(self, parent=None):
        super(MyPolarWindow, self).__init__(parent)

        polarChart = QPolarChart()
        chartView = QChartView()

        scatterSeries = QScatterSeries()

        for value in range(1,50):
            scatterSeries.append(value, random.random()*10)
            
        scatterSeries.setMarkerSize(10)
        scatterSeries.setColor(Qt.red)
        scatterSeries.setBorderColor(Qt.yellow)
        scatterSeries.setMarkerShape(QScatterSeries.MarkerShape.MarkerShapeCircle)
        scatterSeries.setName("Plot")

        polarChart.addSeries(scatterSeries)
        polarChart.setContentsMargins(0,0,0,0)
        polarChart.setTheme(QChart.ChartTheme.ChartThemeDark)

        angularAxis = QValueAxis()
        angularAxis.setTickCount(9)
        angularAxis.setLabelFormat("%.2f")
        angularAxis.setShadesVisible(True)
        angularAxis.setShadesBrush(QBrush(QColor(230,230,255)))
        polarChart.addAxis(angularAxis, QPolarChart.PolarOrientation.PolarOrientationAngular)
        angularAxis.setRange(0,20)

        radialAxis = QValueAxis()
        radialAxis.setTickCount(5)
        radialAxis.setLabelFormat("%d")
        polarChart.addAxis(radialAxis, QPolarChart.PolarOrientation.PolarOrientationRadial)
        radialAxis.setRange(0, 10)

        chartView.setChart(polarChart)
        chartView.setFocusPolicy(Qt.NoFocus)
        chartView.setRenderHint(QPainter.Antialiasing)

        vbox = QVBoxLayout()
        vbox.addWidget(chartView)
        self.setLayout(vbox)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MyPolarWindow()
    win.show()
    sys.exit(app.exec_())
