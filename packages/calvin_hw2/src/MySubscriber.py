#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class MySubscriber:
    def __init__(self):
        rospy.Subscriber('/mystery/output1', Float32, self.callback)

    def callback(self, msg):
        infoString = '\'/mystery/output1\' published ' + str(msg.data)
        rospy.loginfo(infoString)

if __name__ == '__main__':
    rospy.init_node('MySubscriber')
    myNode = MySubscriber()
    rospy.spin()
