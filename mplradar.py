from PyQt5.QtWidgets import*
from PyQt5 import QtGui

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import (NavigationToolbar2QT as NavigationToolbar)

from matplotlib.figure import Figure
from matplotlib.collections import LineCollection

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
        colorbar = self.canvas.fig.colorbar(self.map, ax=self.canvas.ax)

        tick_labels = colorbar.ax.get_yticklabels()
        for label in tick_labels:
            label.set_color('silver')
        colorbar.set_ticks(colorbar.get_ticks())
        colorbar.set_ticklabels([f"{val:.0f} MHz" for val in colorbar.get_ticks()])
        
class MplRadarCanvas(FigureCanvas):
    
    def __init__(self):
        self.fig = Figure(tight_layout = {'pad': 1})
        self.ax = self.fig.add_subplot(111, projection='polar')
        
        FigureCanvas.__init__(self, self.fig)

        self.ax.grid(True)

        # Fig Background Color
        bkg_rgb_color = (34, 39, 62)
        self.bkg_color_normalized = tuple(component / 255 for component in bkg_rgb_color)
        
    def set_axis_color(self, color):
        self.ax.spines['polar'].set_color(color)
        self.ax.spines['polar'].set_linewidth(4)
        self.ax.tick_params(axis='both', which='major', colors=color, direction='inout', length=50)

    def plot_scatter_points(self, theta, r, size, color, marker, label, edgecolors=None):
        self.ax.set_ylim(0, 1)

        # Set the new location for the 0-degree tick
        self.ax.set_theta_zero_location('N')
        self.ax.set_theta_offset(np.pi/2)
        self.ax.set_theta_direction(-1)

        # Hide radial labels and lines
        self.ax.set_yticklabels([])
    
        self.ax.scatter(np.radians(theta), r, s=size, c=color, cmap='gist_rainbow', marker=marker, label=label, edgecolors=edgecolors)

    def update_scan_line(self, angle):
        angle_rad = np.radians(angle)
        self.ax.plot([angle_rad, angle_rad], [0, 1], color=(0.1, 0.8, 0, 0.5), linestyle='solid', linewidth=3)

    def update_scan_profile(self, angle, linewidth=5):

        lines = []
        angles = np.arange(angle, angle + 15, 0.5)

        # Create an array of alpha values for varying opacity
        alphas = np.linspace(0, 1, len(angles))

        for i, angle in enumerate(angles):
            angle_rad = np.radians(angle)
            theta = np.array([angle_rad, angle_rad])
            r = np.array([0, 1])
            points = np.column_stack((theta, r))
            lines.append(points)

        # Specify lime green color with varying alpha values directly
        line_colors = [(50 / 255, 205 / 255, 50 / 255, alpha) for alpha in alphas]

        # Create a LineCollection with the specified colors
        lc = LineCollection(lines, color=line_colors, linewidth=linewidth)

        # Add the LineCollection to the plot
        self.ax.add_collection(lc)
        