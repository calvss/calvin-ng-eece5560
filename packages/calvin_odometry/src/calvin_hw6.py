#!/usr/bin/env python3

import rospy
from odometry_hw.msg import DistWheel
from odometry_hw.msg import Pose2D

class Calvin_hw6:
    def __init__(self):
        rospy.Subscriber('/dist_wheel', DistWheel, self.callback)

        self.publisher = rospy.Publisher('convertedOutput', Pose2D, queue_size=10)
        self.pub_msg = Pose2D()

    def callback(self, msg):
        # define conversion factors (meters per X)
        pass

if __name__ == '__main__':
    rospy.init_node('calvin_hw6')
    myNode = Calvin_hw6()
    rospy.spin()
