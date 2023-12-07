from PyQt5.QtWidgets import*
from PyQt5 import QtGui

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import (NavigationToolbar2QT as NavigationToolbar)

from matplotlib.figure import Figure
import matplotlib.colors as mcolors

import numpy as np

    
class MplRadar(QWidget):
    
    def __init__(self, parent = None):

        QWidget.__init__(self, parent)
        
        self.canvas = MplRadarCanvas()
        # self.mpl_toolbar = NavigationToolbar(self.canvas, self)

        bkg_rgb_color = (34, 39, 62)
        # Set Fig Background Color
        normalized_color = tuple(component / 255 for component in bkg_rgb_color)
        self.canvas.fig.patch.set_facecolor(normalized_color)  # Set the face color of the entire figure
        # Set Plot Color
        self.canvas.ax.set_facecolor(normalized_color)

        # Set Grid Color
        self.setAxisColor('silver')
        
        vertical_layout = QVBoxLayout()
        # vertical_layout.addWidget(self.mpl_toolbar)
        vertical_layout.addWidget(self.canvas)
        self.setLayout(vertical_layout)
        
    def setTicks(self, array):
        theta = np.linspace(0, 2 * np.pi, len(array), endpoint=False)
        self.canvas.ax.set_xticks(theta)
        self.canvas.ax.set_xticklabels([f"{val:.2f} Hz" for val in theta]) 
    
    def setTitle(self, title, fontsize):
        self.canvas.ax.set_title(title, fontsize=fontsize)

    def setAxisColor(self, color):
        self.canvas.set_axis_color(color)
    
    def plotScatterPoints(self, theta, r, size, color='blue', marker='o', label=None, edgecolors=None):
        self.canvas.plot_scatter_points(theta, r, size, color, marker, label, edgecolors=edgecolors)
    
    def set_colorbar(self, freqs):
        self.map = self.canvas.ax.imshow(np.stack([freqs, freqs]),cmap='gist_rainbow')
        self.canvas.fig.colorbar(self.map, ax=self.canvas.ax)
        
        
class MplRadarCanvas(FigureCanvas):
    
    def __init__(self):
        self.fig = Figure(tight_layout = {'pad': 1})
        self.ax = self.fig.add_subplot(111, projection='polar')
        
        FigureCanvas.__init__(self, self.fig)
        # FigureCanvas.setSizePolicy(self,
        #                        QtGui.QSizePolicy.Expanding,
        #                        QtGui.QSizePolicy.Expanding)
        # self.ax.tick_params(axis='both', which='major', labelsize=6)
        # self.ax.tick_params(axis='both', which='minor', labelsize=6) 

        

        # self.ax.set_rmax(2)
        # self.ax.set_rticks([0.5, 1, 1.5, 2])  # Less radial ticks
        # self.ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line
        self.ax.grid(True)
        

    def set_axis_color(self, color):
        self.ax.spines['polar'].set_color(color)
        self.ax.spines['polar'].set_linewidth(2)
        self.ax.tick_params(axis='x', colors=color)
        self.ax.tick_params(axis='y', colors=color)

    def plot_scatter_points(self, theta, r, size, color, marker, label, edgecolors=None):
        self.ax.set_ylim(0, 1)
        self.ax.scatter(np.radians(theta), r, s=size, c=color, cmap='gist_rainbow', marker=marker, label=label, edgecolors=edgecolors)
        