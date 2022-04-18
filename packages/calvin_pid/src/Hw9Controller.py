#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

import calvin_controller

class Hw9Controller:
    def __init__(self, kp, ki, kd):
        rospy.Subscriber("error", Float32, self.callback)
        self.publisher = rospy.Publisher("control_input", Float32, queue_size=10)

        self.controller = calvin_controller.PID(kp=kp, ki=ki, kd=kd)

    def callback(self, error):
        control = self.controller.update(error)
        self.publisher.Publish(control)

if __name__ == '__main__':
    rospy.init_node('calvin_hw9')
    myNode = Hw9Controller()
    rospy.spin()
