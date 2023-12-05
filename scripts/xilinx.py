import serial
import time
import re

from DF_Antenna_Data import *

class Xilinx_Antenna:
    def __init__(self) -> None:

        self.data = DF_Antenna_Data()

    def connect(self, **kwargs):

        if kwargs.get("port") == None:
            print("[XILINX] Connection Failed! Serial Port not specified.")
        else:
            self.port = kwargs.get("port")

        self.baud = kwargs.get("baud", 115200)

        try:
            self.xilinx = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                parity=serial.PARITY_NONE,
                timeout = None
            )
            time.sleep(1)
        except serial.SerialException or serial.SerialTimeoutException:
            print("[XILINX] Error connecting to port. Please try again.")
            pass
        else:
            print("[XILINX] Connection successfully established.")
            print()
    
    def disconnect(self):

        if not self.xilinx.is_open:
            print("[XILINX] Port is already closed!")
        else:
            self.xilinx.close()


    def read_data(self):
        datastream = ""
        startSequence = r"11111111 >> ([0-9]*)"
        stopSequence = r"([0-9]*) >> ([0-9]*)"
        self.data.amplitudes = []

        if self.xilinx.is_open:
            
            command = '\n'
            command = command.encode()
            self.xilinx.write(command)

            while True:
                bytes = self.xilinx.read(self.xilinx.in_waiting)
                decoded_bytes = bytes.decode('utf-8')
                if decoded_bytes != '':
                    datastream = datastream + decoded_bytes
                    if "*" in datastream:
                        break
                    else:
                        continue
                
            datastream = datastream.split("\r\n")

            for line in datastream:
                startresult = re.search(startSequence, line)
                stopresult = re.search(stopSequence, line)

                if line.strip().isnumeric():
                    self.data.amplitudes.append(int(line))
                elif startresult is not None:
                    self.data.f1 = int(startresult.groups()[0])
                elif stopresult is not None:
                    self.data.f2 = int(stopresult.groups()[1])
                    self.data.n_samples = int(stopresult.groups()[0])
                if "*" in line:
                    return self.data
        
        else:
            print("[XILINX] Cannot retrieve data from unavailable port!")
            return -1
    
    def return_data(self):
        return self.data.__dict__
    
    def is_connected(self):
        return self.xilinx.is_open
                    
if __name__ == "__main__":

    FPGA_PORT = '/dev/ttyUSB0'      # port for Linux / Ubuntu
    FPGA_BAUD = 115200

    print("Connecting to FPGA...")
    fpga = Xilinx_Antenna()
    fpga.connect(port=FPGA_PORT, baud=FPGA_BAUD)

    if fpga.is_connected() == True:
        print("Reading Antenna data....")
        data = fpga.read_data()
        # print("Received {} amplitudes.".format(len(data.amplitudes)))
        # print("f1: ", data.f1)
        # print("f2: ", data.f2)
        # print("Bandwidth: ", data.bandwidth)
        # print("Sample Size: ", data.n_samples)

        print(fpga.return_data())
