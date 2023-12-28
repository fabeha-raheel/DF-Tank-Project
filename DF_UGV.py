import rospy
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool
from std_msgs.msg import Float64
import time


class DF_UGV():

    def __init__(self) -> None:

        rospy.init_node('DF_UGV', anonymous=True)
        
        self.compass_heading = None

        self._throttle_channel = 1
        self._steering_channel = 0

        self.speeds = ["SLOW", "MEDIUM", "FAST"]

    def initialize_subscribers(self):
        rospy.Subscriber('/mavros/global_position/compass_hdg',Float64, self.compass_heading_cb)

    def initialize_publishers(self):
        self.rc_override = rospy.Publisher('mavros/rc/override', OverrideRCIn)

    def arm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            armResponse = armService(True)
            rospy.loginfo(armResponse)
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)
        time.sleep(1)

    def disarm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            armResponse = armService(False)
            rospy.loginfo(armResponse)
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)

    def move_forward(self, speed):
        msg = OverrideRCIn()

        if speed == 'MEDIUM':
            msg.channels[self._throttle_channel] = 1800
        elif speed == 'FAST':
            msg.channels[self._throttle_channel] = 2000
        else:
            msg.channels[self._throttle_channel] = 1650

        self.rc_override.publish(msg)

    def move_backward(self, speed):
        msg = OverrideRCIn()

        if speed == 'MEDIUM':
            msg.channels[self._throttle_channel] = 1200
        elif speed == 'FAST':
            msg.channels[self._throttle_channel] = 1000
        else:
            msg.channels[self._throttle_channel] = 1350

        self.rc_override.publish(msg)

    def rotate_left(self, speed):
        msg = OverrideRCIn()

        if speed == 'MEDIUM':
            msg.channels[self._steering_channel] = 1200
        elif speed == 'FAST':
            msg.channels[self._steering_channel] = 1000
        else:
            msg.channels[self._steering_channel] = 1350

        self.rc_override.publish(msg)

    def rotate_right(self, speed):
        msg = OverrideRCIn()

        if speed == 'MEDIUM':
            msg.channels[self._steering_channel] = 1800
        elif speed == 'FAST':
            msg.channels[self._steering_channel] = 2000
        else:
            msg.channels[self._steering_channel] = 1650

        self.rc_override.publish(msg)

    def stop(self):
        msg = OverrideRCIn()

        msg.channels[self._steering_channel] = 1500
        msg.channels[self._throttle_channel] = 1500

        self.rc_override.publish(msg)

    def compass_heading_cb(self, mssg):
        self.compass_heading = mssg.data

if __name__ == '__main__':
    myRover = DF_UGV()

    print("Arming Rover...")
    myRover.arm()

    for i in range(5):
        print("Moving forward...")
        myRover.move_forward(speed=myRover.speeds[2])
        time.sleep(1)
    print("Stopping Rover")
    myRover.stop()
    time.sleep(1) 
    for i in range(5):
        print("Moving backward...")
        myRover.move_backward(speed=myRover.speeds[2])
        time.sleep(1)
    print("Stopping Rover")
    myRover.stop()
    time.sleep(1) 
    for i in range(5):
        print("Rotating Right...")
        myRover.rotate_right(speed=myRover.speeds[2])
        time.sleep(1)
    print("Stopping Rover")
    myRover.stop()
    time.sleep(1) 
    for i in range(5):
        print("Rotating Left...")
        myRover.rotate_left(speed=myRover.speeds[2])
        time.sleep(1)
    print("Stopping Rover")
    myRover.stop()        