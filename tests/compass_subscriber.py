#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64


def Compass_Subscriber_callback(mssg):

    rospy.loginfo("Heading %s", mssg.data)
    # compass_data = mssg
    # print(compass_data)

def listener():
    rospy.init_node('compass_data', anonymous=True) 
    rospy.Subscriber('/mavros/global_position/compass_hdg',Float64, Compass_Subscriber_callback)
    rospy.spin()


if __name__ == "__main__":
    listener()
    

    # Compass_Subscriber=rospy.Subscriber('/mavros/global_position/compass_hdg',Float64, Compass_Subscriber_callback)
