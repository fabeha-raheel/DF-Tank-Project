from PyQt5.QtWidgets import*
from PyQt5 import QtGui

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import (NavigationToolbar2QT as NavigationToolbar)

from matplotlib.figure import Figure

    
class MplWidget(QWidget):
    
    def __init__(self, parent = None):

        QWidget.__init__(self, parent)
        
        self.canvas = MplCanvas()
        self.mpl_toolbar = NavigationToolbar(self.canvas, self)
        
        vertical_layout = QVBoxLayout()
        # vertical_layout.addWidget(self.mpl_toolbar)
        vertical_layout.addWidget(self.canvas)
        self.setLayout(vertical_layout)
        
    def setTitle(self, title):
        self.canvas.ax.set_title(title, fontsize=10)
        
    def setLabels(self, xlabel, ylabel):
        self.canvas.ax.set_ylabel(ylabel, fontsize=5)
        self.canvas.ax.set_xlabel(xlabel, fontsize=5)

    def setLimits(self, min, max):
        self.canvas.ax.set_ylim(min, max)
        
        
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
        
        
        
        
            
        

