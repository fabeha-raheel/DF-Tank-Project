import time
import serial
import struct

class PTZ_Controller():

    def __init__(self) -> None:
        
        self.pan_angle = None
        self.tilt_angle = None
        
        self.address = 0x01
        self.comm_delay = 2
        self.mode_fast = False

    def connect(self, **kwargs):

        if kwargs.get("port") == None:
            print("[PTZ] Connection Failed! Serial Port not specified.")
        else:
            self.port = kwargs.get("port")

        self.baud = kwargs.get("baud", 115200)

        try:
            self.ptz = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                parity=serial.PARITY_NONE,
                timeout = None
            )
            time.sleep(1)
        except serial.SerialException:
            print("[PTZ] Error connecting to port. Please try again.")
        else:
            print("[PTZ] Connection successfully established.")
            print()

        print("[PTZ] to home position...")
        self.goto_zero_pan()
        self.goto_zero_tilt()
        print("[PTZ] Ready")
    
    def disconnect(self):

        if not self.ptz.is_open:
            print("[PTZ] Port is already closed!")
        else:
            self.ptz.close()
    
    def goto_zero_pan(self):
        cksm = (self.address + 0x00 + 0x4B + 0x00 + 0x00) % 256
        cksm = int(hex(cksm), 16)

        cmd = struct.pack("BBBBBBB", 0xFF, self.address, 0x00, 0x4B, 0x00, 0x00, cksm)

        try:
            if self.ptz.is_open:
                self.ptz.write(cmd)
                self.pan_angle = 0
                if self.mode_fast == False:
                    time.sleep(self.comm_delay)
            else:
                print("[PTZ] Port not opened.")
        except:
            print("[PTZ] Writing previous command failed.")
    
    def goto_zero_tilt(self):
        cksm = (self.address + 0x00 + 0x4D + 0x00 + 0x00) % 256
        cksm = int(hex(cksm), 16)

        cmd = struct.pack("BBBBBBB", 0xFF, self.address, 0x00, 0x4D, 0x00, 0x00, cksm)

        try:
            if self.ptz.is_open:
                self.ptz.write(cmd)
                self.til_angle = 0
                if self.mode_fast == False:
                    time.sleep(self.comm_delay)
            else:
                print("[PTZ] Port not opened.")
        except:
            print("[PTZ] Writing previous command failed.")

    def set_pan_position(self, degrees):
        angle = degrees*100
        hex_angle = '{:04x}'.format(angle)
        data1 = int(hex_angle[0] + hex_angle[1], 16)
        data2 = int(hex_angle[2] + hex_angle[3], 16)
        cksm = (self.address + 0x00 + 0x4B + data1 + data2) % 256
        cksm = int(hex(cksm), 16)

        cmd = struct.pack("BBBBBBB", 0xFF, self.address, 0x00, 0x4B, data1, data2, cksm)

        try:
            if self.ptz.is_open:
                self.ptz.write(cmd)
                if self.mode_fast == False:
                    time.sleep(self.comm_delay)
                self.pan_angle = degrees
            else:
                print("[PTZ] Port not opened.")
        except:
            print("[PTZ] Writing previous command failed.")
    
    def set_tilt_position(self, degrees):
        angle = degrees*100
        hex_angle = '{:04x}'.format(angle)
        data1 = int(hex_angle[0] + hex_angle[1], 16)
        data2 = int(hex_angle[2] + hex_angle[3], 16)
        cksm = (self.address + 0x00 + 0x4D + data1 + data2) % 256
        cksm = int(hex(cksm), 16)

        cmd = struct.pack("BBBBBBB", 0xFF, self.address, 0x00, 0x4D, data1, data2, cksm)

        try:
            if self.ptz.is_open:
                self.ptz.write(cmd)
                if self.mode_fast == False:
                    time.sleep(self.comm_delay)
                self.tilt_angle = degrees
            else:
                print("[PTZ] Port not opened.")
        except:
            print("[PTZ] Writing previous command failed.")

    def is_connected(self):
        return self.ptz.is_open
    
    def current_pan_angle(self):
        
        if self.pan_angle is not None:
            return self.pan_angle
        else:
            return -1
    
    def current_tilt_angle(self):

        if self.tilt_angle is not None:
            return self.tilt_angle
        else:
            return -1
    

if __name__ == "__main__":

    PTZ_PORT = '/dev/ttyUSB1'      
    PTZ_BAUD = 9600

    print("Connecting to Pantilt...")
    pantilt = PTZ_Controller()
    pantilt.connect(port=PTZ_PORT, baud=PTZ_BAUD)

    pantilt.mode_fast = False   # Wait 2s after sending each command to pantilt. If set to True, then functions will not block for 2s after sending commands.

    # Set Pan Angle
    desired_pan_angle = 15      # in degrees
    pantilt.set_pan_position(desired_pan_angle)

    # Return Current angles
    print("Current Pan Position: ", pantilt.current_pan_angle())
    print("Current Tilt Position: ", pantilt.current_tilt_angle())

    # Set Tilt Angle
    desired_tilt_angle = 45     # in degrees
    pantilt.set_tilt_position(desired_tilt_angle)

    # Return Current angles
    print("Current Pan Position: ", pantilt.current_pan_angle())
    print("Current Tilt Position: ", pantilt.current_tilt_angle())

    # Go to Zero Pan Position
    pantilt.goto_zero_pan()

    # Go to Zero Tilt Position
    pantilt.goto_zero_tilt()

    # Return Current angles
    print("Current Pan Position: ", pantilt.current_pan_angle())
    print("Current Tilt Position: ", pantilt.current_tilt_angle())

    