#!/usr/bin/env python3

import random

import rospy
from std_msgs.msg import Float32

class MyPublisher:
    def __init__(self):
        self.publisher = rospy.Publisher('/mystery/input', Float32, queue_size=10)

if __name__ == '__main__':
    random.seed()
    rospy.init_node('MyPublisher')
    rate = rospy.Rate(1)

    myNode = MyPublisher()

    rospy.logdebug("starting loop")
    while not rospy.is_shutdown():
        n = randint(0, 10)
        myNode.publisher.publish(n)
        rate.sleep()
