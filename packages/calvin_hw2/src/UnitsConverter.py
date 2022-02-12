#!/usr/bin/env python3

import rospy
from mystery_package.msg import UnitsLabelled

class UnitsConverter:
    def __init__(self):
        rospy.Subscriber('/mystery/output2', UnitsLabelled, self.callback)

        self.publisher = rospy.Publisher('convertedOutput', UnitsLabelled, queue_size=10)
        self.pub_msg = UnitsLabelled()

    def callback(self, msg):
        # define conversion factors (meters per X)
        factorsDict = {
            'meters': 1.0,
            'smoots': 1.7018,
            'feet': 0.3048
        }

        if rospy.has_param("output_units"):
            self.outputUnits = rospy.get_param("output_units")
        else:
            self.outputUnits = "smoots" # default to smoots

        conversionFactor = factorsDict[self.outputUnits]

        self.meters = msg.value
        self.output = self.meters / conversionFactor
        self.pub_msg.value = self.output
        self.pub_msg.units = self.outputUnits
        self.publisher.publish(self.pub_msg)

if __name__ == '__main__':
    rospy.init_node('UnitsConverter')
    myNode = UnitsConverter()
    rospy.spin()
