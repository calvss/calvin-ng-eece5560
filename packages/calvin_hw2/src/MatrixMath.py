#!/usr/bin/env python3

import numpy as np
import rospy
from duckietown_msgs.msg import Vector2D

class MatrixMath:
    def __init__(self):
        rospy.Subscriber('/sensor_coord', Vector2D, self.callback)
        self.publisherRobot = rospy.Publisher('/robot_coord', Vector2D, queue_size=10)
        self.publisherWorld = rospy.Publisher('/world_coord', Vector2D, queue_size=10)

    def callback(self, msg):
        tf_sensorRobot = [[-1,  0, -1],
                          [ 0, -1,  0],
                          [ 0,  0,  1]]

        tf_robotWorld = [[np.cos(np.radians(135)), -1 * np.sin(np.radians(135)), 3],
                         [np.sin(np.radians(135)),      np.cos(np.radians(135)), 2],
                         [                      0,                            0, 1]]

        sensorCoord = [[0], [0], [1]]
        sensorCoord[0] = [msg.x]
        sensorCoord[1] = [msg.y]
        robotCoord = tf_sensorRobot * sensorCoord

        v_robotCoord = Vector2D()
        v_robotCoord.x = robotCoord[0][0]
        v_robotCoord.y = robotCoord[1][0]

        self.publisherRobot.publish(v_robotCoord)

if __name__ ==  '__main__':
    rospy.init_node('MatrixMath')
    myNode = MatrixMath()
    rospy.spin()
