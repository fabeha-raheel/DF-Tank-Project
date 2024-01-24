#!/usr/bin/python3

import sys
import numpy as np
import threading
import random
import websocket #pip3 install websocket-client
import json
import signal  # Import the signal module for handling Ctrl+C

import rospy
from std_msgs.msg import Float64
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode, SetModeRequest

from xilinx import *
from DF_Antenna_Data import *
from PTZ_Controller import *

TEST = False

FPGA_PORT = '/dev/ttyUSB-FPGA'
FPGA_BAUD = 115200

# PTZ_PORT = '/dev/ttyCH341USB0'
PTZ_PORT = '/dev/ttyUSB-PTZ'
PTZ_BAUD = 9600

# websocket_url = "ws://localhost:8000/"
websocket_url = "ws://192.168.0.116:8000/"

class DF_System():

    def __init__(self, test=False, ws_url="ws://localhost:8000/"):
        
        self.df_data = DF_Data()

        self.test = test

        self.pan_sequence = [90, 67.5, 45, 22.5, 0, 337.5, 315, 292.5, 270, 292.5, 315, 337.5, 0, 22.5, 45, 67.5]

        self.run_threads = True

        self._throttle_channel = 1
        self._steering_channel = 0
        self._forward = False
        self._backward = False
        self._right = False
        self._left = False
        self._stop = False

        self.ws_url = ws_url
        self.ws_connected = False

    def initialize(self):

        if self.test:
            rospy.init_node('df_ugv', anonymous=True)
            self.init_ros_heading_subscriber()
            self.init_ros_control_publisher()
            self.disarm_ugv()

            self.dummy_thread = threading.Thread(target=self.request_dummy_data_continuously, daemon=True)
            self.dummy_thread.start()

            self.controls_thread = threading.Thread(target=self.monitor_controls, daemon=True)
            self.controls_thread.start()

        else:
            rospy.init_node('df_ugv', anonymous=True)
            self.init_ros_heading_subscriber()
            self.init_ros_control_publisher()
            self.disarm_ugv()

            self.initialize_system()

            # start the continuous data acquisition thread
            print("Starting Data Acquisition and Monitor Controls Thread...")
            self.daq_thread = threading.Thread(target=self.request_data_continuously, daemon=True)
            self.daq_thread.start()

            self.controls_thread = threading.Thread(target=self.monitor_controls, daemon=True)
            self.controls_thread.start()
        
        self.ws_thread = threading.Thread(target=self.initialize_ws_client, daemon=True)
        self.ws_thread.start()

        # Register a signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.handle_interrupt)

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Ctrl+C pressed. Cleaning up...")
            self.cleanup()

    def cleanup(self):
        # Close the WebSocket connection
        if self.ws_connected:
            self.ws.close()

        # Stop all threads
        self.run_threads = False  # Stop the threads gracefully
        self.ws_thread.join()  # Wait for the WebSocket thread to finish

        # ... (add more cleanup if needed)

        print("Clean exit.")

    def handle_interrupt(self, signum, frame):
        # Handle Ctrl+C by triggering the cleanup process
        self.cleanup()
        sys.exit(0)
    
    def initialize_ws_client(self):
        
        self.ws = websocket.WebSocketApp(self.ws_url,
                                on_message=self.on_message,
                                on_error=self.on_error,
                                on_close=self.on_close)
        
        # Set the on_open callback to handle the opening of the connection
        self.ws.on_open = self.on_open

        # Start the WebSocket connection (this will run the on_open callback)
        self.ws.run_forever(ping_interval=13, ping_timeout=10)

    def on_message(self, ws, message):
        # print(f"Received message: {message}")
        command = json.loads(message)

        if command['command'] == 'forward':
            print("Moving forward...")
            self.move_forward()
        elif command['command'] == 'backward':
            print("Moving backward...")
            self.move_backward()
        elif command['command'] == 'left':
            print("Turning left...")
            self.turn_left()
        elif command['command'] == 'right':
            print("Turning right...")
            self.turn_right()
        elif command['command'] == 'arm':
            print("Arming ugv...")
            self.arm_ugv()
        elif command['command'] == 'disarm':
            print("Disarming ugv...")
            self.disarm_ugv()
        elif command['command'] == 'manual':
            print("Setting Manual Mode")
            self.set_mode_ugv(mode='MANUAL')
        elif command['command'] == 'hold':
            print("Setting Hold Mode...")
            self.set_mode_ugv(mode='HOLD')
        else:
            print("Stopping UGV...")
            self.stop()


    def on_error(self, ws, error):
        print(f"Error: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        print(f"Closed with status code {close_status_code}: {close_msg}")
        self.ws_connected = False
        self.ws_timer +=1
        print("Reconnecting after ", self.ws_timer)
        time.sleep(self.ws_timer)
        self.handle_websocket_closing()

    def handle_websocket_closing(self):
        print("******reconnecting to WS server******")
        t = threading.Thread(target=self.initialize_ws_client)
        t.start()

    def on_open(self, ws):
        print("WebSocket connection opened")
        self.ws_timer = 0
        time.sleep(1)
        self.ws_connected = True

    def send_data(self):
        # print("Sending data....")
        data = {
            "type": 'data',
            "f1": self.df_data.f1,
            "f2": self.df_data.f2,
            "n_samples": self.df_data.n_samples,
            "beam_width": self.df_data.beam_width,
            "amplitudes": self.df_data.amplitudes,
            "angle_pt": self.df_data.angle_pt,
            "heading": self.df_data.heading
        }

        # Convert sensor data to JSON
        json_data = json.dumps(data)

        # Send the sensor data over the WebSocket connection
        self.ws.send(json_data)

    def request_dummy_data_continuously(self):

        self.df_data.f1 = 400000000
        self.df_data.f2 = 5900000000
        self.df_data.n_samples = 100
        
        self.df_data.amplitudes = self.generate_dummy_data()
        self.frequencies_range_hz = np.arange(start=self.df_data.f1, stop=self.df_data.f2, step=((self.df_data.f2-self.df_data.f1)/self.df_data.n_samples))

        self.frequencies_range_Mhz = self.frequencies_range_hz / 1000000

        self.frequencies = list(self.frequencies_range_Mhz)

        self.avg_freqs = self.df_data.averaging(array=self.frequencies, N=10)

        self.df_data.angle_pt = 0

        self.df_data.initialize_matrix()

        self.df_data.matrix[:, self.df_data.current_sector] = self.df_data.amplitudes

        while True:
            for pan in self.pan_sequence:
                self.df_data.amplitudes = self.generate_dummy_data()
                if pan < 180:
                    self.df_data.angle_pt = pan
                else:
                    self.df_data.angle_pt = pan - 360
                self.df_data.matrix[:, self.df_data.current_sector] = self.df_data.amplitudes
                if self.ws_connected:
                    self.send_data()
                if not self.run_threads:
                    break
                time.sleep(0.5)
    
    def initialize_system(self):

        print("Connecting to FPGA...")
        self.fpga = Xilinx_Antenna()
        self.fpga.connect(port=FPGA_PORT, baud=FPGA_BAUD)

        print("Connecting to Pan Tilt Mechanism...")
        self.pantilt = PTZ_Controller()
        self.pantilt.connect(port=PTZ_PORT, baud=PTZ_BAUD)
        self.pantilt.mode_fast = True

        if self.fpga.is_connected():

            while True:
                print("Reading Antenna data....")
                data = self.fpga.get_static_data()

                if data != -1:
                    self.df_data.f1 = data["_f1"]
                    self.df_data.f2 = data["_f2"]
                    self.df_data.n_samples = data["_n_samples"]
                    self.df_data.amplitudes = self.fpga.dynamic_data.amplitudes
                    print("Static data received.")
                else:
                    print("Error receiving static antenna data")

                if len(self.df_data.amplitudes) == self.df_data.n_samples:
                    break
            
            self.frequencies_range_hz = np.arange(start=self.df_data.f1, stop=self.df_data.f2, step=((self.df_data.f2-self.df_data.f1)/self.df_data.n_samples))

            self.frequencies_range_Mhz = self.frequencies_range_hz / 1000000

            self.frequencies = list(self.frequencies_range_Mhz)

            self.avg_freqs = self.df_data.averaging(array=self.frequencies, N=10)

            self.df_data.angle_pt = 0

            self.df_data.initialize_matrix()

            self.df_data.matrix[:, self.df_data.current_sector] = self.df_data.amplitudes

        print("System Initialization complete!")

    def request_data_continuously(self):
        
        while self.run_threads:

            for pan in self.pan_sequence:
                if self.fpga.is_connected() and self.pantilt.is_connected():
                    self.pantilt.set_pan_position(pan)
                    self.df_data.amplitudes = self.fpga.read_data()
                    if pan < 180:
                        self.df_data.angle_pt = pan
                    else:
                        self.df_data.angle_pt = pan - 360
                    self.df_data.matrix[:, self.df_data.current_sector] = self.df_data.amplitudes
                    if self.ws_connected:
                        self.send_data()
                    if not self.run_threads:
                        break

    def ros_heading_cb(self, mssg):
        self.df_data.heading = mssg.data

    def init_ros_heading_subscriber(self):
        rospy.Subscriber('/mavros/global_position/compass_hdg',Float64, self.ros_heading_cb)

    def init_ros_control_publisher(self):
        self.ugv_rc_override = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=1)

    def arm_ugv(self):
        rospy.wait_for_service('/mavros/cmd/arming', timeout=3)
        try:
            armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            armResponse = armService(True)
            rospy.loginfo(armResponse)
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)

    def disarm_ugv(self):
        rospy.wait_for_service('/mavros/cmd/arming', timeout=3)
        try:
            armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            armResponse = armService(False)
            rospy.loginfo(armResponse)
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)

    def set_mode_ugv(self, mode):
        rospy.wait_for_service('/mavros/set_mode', timeout=3)
        try:
            data = SetModeRequest()
            data.custom_mode = mode
            modeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            modeResponse = modeService(data)
            rospy.loginfo(modeResponse)
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)

    def get_tank_heading(self):
        if self.df_data.heading > 180:
            return int(self.df_data.heading)-360
        else:
            return int(self.df_data.heading)
        
    def get_antenna_heading(self):
        antenna_heading = (self.get_tank_heading() + self.df_data.angle_pt)

        if antenna_heading > 180:
            return int(antenna_heading-360)
        elif antenna_heading < -180:
            return int(antenna_heading + 360)
        else:
            return int(antenna_heading)

    def move_forward(self):
        self._forward = True
        self._backward = False
        self._left = False
        self._right = False
        self._stop = False

    def move_backward(self):
        self._forward = False
        self._backward = True
        self._left = False
        self._right = False
        self._stop = False

    def turn_right(self):
        self._forward = False
        self._backward = False
        self._left = False
        self._right = True
        self._stop = False
    
    def turn_left(self):
        self._forward = False
        self._backward = False
        self._left = True
        self._right = False
        self._stop = False

    def stop(self):
        self._forward = False
        self._backward = False
        self._left = False
        self._right = False
        self._stop = True

    def monitor_controls(self):
        while self.run_threads:
            if self._forward:
                msg = OverrideRCIn()
                msg.channels[self._throttle_channel] = 1650
                msg.channels[self._steering_channel] = 1500
                self.ugv_rc_override.publish(msg)
                time.sleep(0.2)

            elif self._backward:
                msg = OverrideRCIn()
                msg.channels[self._throttle_channel] = 1350
                msg.channels[self._steering_channel] = 1500
                self.ugv_rc_override.publish(msg)
                time.sleep(0.2)

            elif self._left:
                msg = OverrideRCIn()
                msg.channels[self._throttle_channel] = 1500
                msg.channels[self._steering_channel] = 1350
                self.ugv_rc_override.publish(msg)
                time.sleep(0.2)

            elif self._right:
                msg = OverrideRCIn()
                msg.channels[self._throttle_channel] = 1500
                msg.channels[self._steering_channel] = 1650
                self.ugv_rc_override.publish(msg)
                time.sleep(0.2)

            elif self._stop:
                msg = OverrideRCIn()
                msg.channels[self._throttle_channel] = 1500
                msg.channels[self._steering_channel] = 1500
                self.ugv_rc_override.publish(msg)
                time.sleep(0.2)

    def generate_dummy_data(self, start= 0, stop=100, n=100):
        return random.sample(range(start, stop), n)


if __name__ == '__main__':

    df_system = DF_System(test=TEST)
    while True:
        time.sleep(1)