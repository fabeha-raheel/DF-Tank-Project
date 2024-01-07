from PyQt5.QtWidgets import*
from PyQt5 import QtGui

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import (NavigationToolbar2QT as NavigationToolbar)

from matplotlib.figure import Figure

    
class MplWidget(QWidget):
    
    def __init__(self, parent = None):

        QWidget.__init__(self, parent)

        self.xdata = []
        self.ydata = []
        
        self.canvas = MplCanvas()
        # self.mpl_toolbar = NavigationToolbar(self.canvas, self)

        vertical_layout = QVBoxLayout()
        # vertical_layout.addWidget(self.mpl_toolbar)
        vertical_layout.addWidget(self.canvas)
        self.setLayout(vertical_layout)

    def refresh_plot(self, xdata, ydata):
        self.line.set_data(xdata, ydata)
        self.canvas.ax.relim()
        self.canvas.ax.autoscale_view()
        self.canvas.fig.canvas.draw()
        
    def setTitle(self, title, fontsize):
        self.canvas.ax.set_title(title, fontsize=fontsize)
        
    def setLabels(self, xlabel, ylabel, fontsize):
        self.canvas.ax.set_ylabel(ylabel, fontsize=fontsize)
        self.canvas.ax.set_xlabel(xlabel, fontsize=fontsize)

    def setLimits(self, min, max):
        self.canvas.ax.set_ylim(min, max)

    def canvasBackgroundColor(self, color):
        self.canvas.fig.set_facecolor(color)

    def setBackgroundColor(self, color):
        self.canvas.ax.set_facecolor(color)

    def setTickcolor(self, color):
        self.canvas.ax.tick_params(colors=color, which='both')  # 'both' refers to minor and major axes
        self.canvas.ax.xaxis.label.set_color(color)
        self.canvas.ax.yaxis.label.set_color(color)
        
        
class MplCanvas(FigureCanvas):
    
    def __init__(self):
        self.fig = Figure(tight_layout = {'pad': 1})
        self.ax = self.fig.add_subplot(111)
        
        FigureCanvas.__init__(self, self.fig)
        # FigureCanvas.setSizePolicy(self,
        #                        QtGui.QSizePolicy.Expanding,
        #                        QtGui.QSizePolicy.Expanding)
        self.ax.tick_params(axis='both', which='major', labelsize=6)
        self.ax.tick_params(axis='both', which='minor', labelsize=6)  