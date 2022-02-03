#!/usr/bin/env python3

import rospy
from mystery_package.msg import UnitsLabelled

class UnitsConverter:
    def __init__(self):
        rospy.Subscriber('/mystery/output2', UnitsLabelled, self.callback)

        self.publisher = rospy.Publisher('outputFeet', UnitsLabelled, queue_size=10)
        self.pub_msg = UnitsLabelled()
        self.pub_msg.units = "feet"

    def callback(self, msg):
        metersPerFoot = 0.3048
        self.meters = msg.value
        self.feet = self.meters / metersPerFoot
        self.pub_msg.value = self.feet
        self.publisher.publish(self.pub_msg)

if __name__ == '__main__':
    rospy.init_node('UnitsConverter')
    myNode = UnitsConverter()
    rospy.spin()
