import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QSizePolicy, QVBoxLayout, QWidget
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class PolarPlotWidget(QWidget):
    def __init__(self):
        super(PolarPlotWidget, self).__init__()

        self.figure, self.ax = self.create_polar_plot()
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        layout = QVBoxLayout()
        layout.addWidget(self.canvas)

        self.setLayout(layout)

    def create_polar_plot(self):
        figure = Figure()
        ax = figure.add_subplot(111, projection='polar')

        # Example data (replace this with your actual data)
        angles = np.linspace(0, 2 * np.pi, 100)
        values = np.random.rand(100)

        ax.plot(angles, values, label='Example Data')
        ax.legend()

        return figure, ax


class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        central_widget = PolarPlotWidget()
        self.setCentralWidget(central_widget)

        self.setWindowTitle("Polar Plot Example")
        self.setGeometry(100, 100, 800, 600)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
