import time
import serial

Serial1 = serial.Serial('COM13') # COMxx  format on Windows
                  # ttyUSBx format on Linux
Serial1.baudrate = 115200  # set Baud rate to 9600
Serial1.bytesize = 8   # Number of data bits = 8
Serial1.parity  ='N'   # No parity
Serial1.stopbits = 1   # Number of Stop bits = 1

Serial2 = serial.Serial('COM14') # COMxx  format on Windows
                  # ttyUSBx format on Linux
Serial2.baudrate = 115200  # set Baud rate to 9600
Serial2.bytesize = 8   # Number of data bits = 8
Serial2.parity  ='N'   # No parity
Serial2.stopbits = 1   # Number of Stop bits = 1
time.sleep(3)


file = open('FPGA Data.txt', 'r')
while True:
    line = file.readline()
    if not line:
        file = open('FPGA Data.txt', 'r')
        line = file.readline()
        # break
    # print(line)
    Serial1.write(str.encode(line))    #transmit 'A' (8bit) to micro/Arduino
    # SerialObj.write(b'A')    #transmit 'A' (8bit) to micro/Arduino
    # time.sleep(0.001)
    data = Serial2.readline()
    if not data:
        break
    print(data)
print('completed')   
Serial1.close()      # Close the port
Serial2.close()  
    