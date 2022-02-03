#!/usr/bin/env python3

import rospy
from mystery_package.msg import UnitsLabelled

class UnitsConverter:
    def __init__(self):
        rospy.Subscriber('/mystery/output2', UnitsLabelled, self.callback)

    def callback(self, msg):
        units = msg.units
        value = msg.value
        rospy.loginfo("I got " + str(units) + " units and " + str(value) + " data")

if __name__ == '__main__':
    rospy.init_node('UnitsConverter')
    myNode = UnitsConverter()
    rospy.spin()
