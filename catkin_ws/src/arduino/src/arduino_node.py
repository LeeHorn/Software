#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import  BoolStamped


class arduinoROS(object):
    def __init__(self):
        self.button_a = BoolStamped()
        self.button_b = BoolStamped()

        # =========== publisher ===========
        self.pub_mid_grab = rospy.Publisher("/arduino/sub/grab", BoolStamped, queue_size=1)
        self.pub_mid_drop = rospy.Publisher("/arduino/sub/drop", BoolStamped, queue_size=1)

        # =========== subscriber ===========
        self.sub_mid_A = rospy.Subscriber("~mid_A", BoolStamped, self.a_process, queue_size=1)
        self.sub_mid_B = rospy.Subscriber("~mid_B", BoolStamped, self.b_process, queue_size=1)


   # =========== subscribe button a===========
    def a_process(self, msg):
        state = BoolStamped()
        state.header.stamp = msg.header.stamp
        state.data = msg
        print "Button_A = ", state.data
        self.pub_mid_grab.publish(state)
        

    # =========== subscribe button b ===========
    def b_process(self, msg):
        state = BoolStamped()
        state.header.stamp = msg.header.stamp
        state.data = msg
        print "Button_B = ", state.data
        self.pub_mid_drop.publish(state)

if __name__ == "__main__":
    rospy.init_node("arduino_ros", anonymous = False)
    arduino_node = arduinoROS()
    rospy.spin()
