#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import  BoolStamped


class arduinoROS(object):
    def __init__(self):
        self.button_a = False
        self.button_b = False

        # =========== publisher ===========
        self.pub_mid_grab = rospy.Publisher("/arduino/sub/grab", BoolStamped, queue_size=1)
        self.pub_mid_drop = rospy.Publisher("/arduino/sub/drop", BoolStamped, queue_size=1)

        # =========== subscriber ===========
        self.sub_mid_A = rospy.Subscriber("~mid_A", BoolStamped, self.a_process, queue_size=1)
        self.sub_mid_B = rospy.Subscriber("~mid_B", BoolStamped, self.b_process, queue_size=1)


   # =========== subscribe button a===========
    def a_process(self, msg):
        self.button_a = msg.data
        print "Button_A = ", self.button_a     
        self.pub_mid_grab.publish(button_a)
        

    # =========== subscribe button b ===========
    def b_process(self, msg):
        self.button_b = msg.data
        print "Button_B = ", self.button_b     
        self.pub_mid_grab.publish(button_b)


if __name__ == "__main__":
    rospy.init_node("arduino_ros", anonymous = False)
    arduino_node = arduinoROS()
    rospy.spin()
