#!/usr/bin/env python3

import numpy as np
import rospy
from odometry_hw.msg import DistWheel
from odometry_hw.msg import Pose2D

class calvin_hw6:

    robot = np.array(
        [
            [0],
            [0],
            [1]
        ]
    )
    theta = 0

    wheelbase_width = 0.1

    def __init__(self):
        rospy.Subscriber('/dist_wheel', DistWheel, self.callback)

        self.publisher = rospy.Publisher('/pose', Pose2D, queue_size=10)
        self.pub_msg = Pose2D()

    def callback(self, msg):
        theta = self.theta
        robot = self.robot
        wheelbase_width = self.wheelbase_width

        x_left = msg.dist_wheel_left
        x_right = msg.dist_wheel_right

        # ---- robot frame ----
        dx = (x_right + x_left) / 2
        dy = 0
        dtheta = (x_right - x_left) / wheelbase_width

        delta = np.array(
            [
                [dx],
                [dy],
                [1]
            ]
        )

        theta = theta + dtheta

        # ---- transformation ----
        tf_robot_world = np.array(
            [
                [np.cos(theta), -1 * np.sin(theta), robot[0, 0]],
                [np.sin(theta), np.cos(theta),      robot[1, 0]],
                [0            , 0,                  1]
            ]
        )

        robot = tf_robot_world.dot(delta)

        self.theta = theta
        self.robot = robot

        self.pub_msg.x = robot[0, 0]
        self.pub_msg.y = robot[1, 0]
        self.pub_msg.theta = theta

        self.publisher.publish(self.pub_msg)

if __name__ == '__main__':
    rospy.init_node('calvin_hw6')
    myNode = calvin_hw6()
    rospy.spin()
