#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

import calvin_controller

class Hw9Controller:
    def __init__(self):
        rospy.Subscriber("error", Float32, self.callback)
        self.publisher = rospy.Publisher("control_input", Float32, queue_size=10)

        self.controller = calvin_controller.PID()

    def callback(self, errorMsg):
        control = self.controller.update(errorMsg.data)
        self.publisher.publish(control)

if __name__ == '__main__':
    rospy.init_node('calvin_hw9')
    myNode = Hw9Controller()
    myNode.controller.kp = 0.5
    myNode.controller.ki = 0.1
    myNode.controller.kd = 10
    myNode.controller.window = 4
    rospy.set_param("controller_ready", "true")
    rospy.spin()
