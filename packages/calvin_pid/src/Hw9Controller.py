#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

import calvin_controller

class Hw9Controller:
    def __init__(self):
        rospy.Subscriber("error", Float32, self.positionUpdater)
        rospy.Subscriber("velocity", Float32, self.velocityUpdater)
        self.publisher = rospy.Publisher("control_input", Float32, queue_size=10)

        self.positionController = calvin_controller.PID(kp = 0.45, ki = 0.1, kd = 2, window = 5)
        self.velocityController = calvin_controller.PID(kp = 1)

        self.desiredVelocity = 0

    def positionUpdater(self, msg):
        self.desiredVelocity = self.positionController.update(msg.data)

    def velocityUpdater(self, msg):
        velocityError = self.desiredVelocity - msg.data
        control = self.velocityController.update(velocityError)
        self.publisher.publish(control)
        rospy.logwarn("ve" + str(velocityError) + "  v" + str(self.desiredVelocity))

if __name__ == '__main__':
    rospy.init_node('calvin_hw9')
    myNode = Hw9Controller()
    rospy.set_param("controller_ready", "true")
    rospy.spin()
