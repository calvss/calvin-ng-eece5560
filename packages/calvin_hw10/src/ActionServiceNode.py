#!/usr/bin/env python3

import rospy
from example_service.srv import *

def fibonacciClientTimer(x):
    rospy.wait_for_service('calc_fibonacci')
    try:
        fibonacciProxy = rospy.ServiceProxy('calc_fibonacci', Fibonacci)

        startTime = rospy.get_rostime()
        serviceResponse = fibonacciProxy(x)
        elapsedTime = rospy.get_rostime() - startTime
        return elapsedTime
    except rospy.ServiceException as e:
        print("Service call failed: %s", e)

if __name__ == '__main__':
    rospy.init_node('ActionServiceNode')
    rospy.logwarn(fibonacciClientTimer(3))
    rospy.logwarn(fibonacciClientTimer(15))
    rospy.spin()
