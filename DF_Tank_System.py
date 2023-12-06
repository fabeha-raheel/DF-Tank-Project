from PyQt5.QtCore import pyqtSlot, pyqtSignal, QObject

from xilinx import *
from PTZ_Controller import *

class DF_Tank_System(QObject):

    def __init__(self):
        super().__init__()

        self.df_antenna = Xilinx_Antenna()
        self.pantilt = PTZ_Controller()

        self.antenna_data = None

    def connect_system(self, antenna_port, antenna_baud, pantilt_port, pantilt_baud):

        if not self.df_antenna.is_connected():
            print("Connecting to DF Antenna...")
            self.df_antenna.connect(port=antenna_port, baud=antenna_baud)
        else:
            print("DF Antenna is already connected!")
        
        if not self.pantilt.is_connected():
            print("Connecting to Pantilt...")
            self.pantilt.connect(port=pantilt_port, baud=pantilt_baud)
        else:
            print("Pantilt is already connected!")
    
    def disconnect_system(self):

        if self.df_antenna.is_connected():
            print("Disconnecting DF Antenna...")
            self.df_antenna.disconnect()
        else:
            print("DF Antenna is already disconnected!")
        
        if self.pantilt.is_connected():
            print("Disconnecting Pantilt....")
            self.pantilt.disconnect()
        else:
            print("Pantilt is already disconnected!")
    
    def get_antenna_data(self):
        pass

