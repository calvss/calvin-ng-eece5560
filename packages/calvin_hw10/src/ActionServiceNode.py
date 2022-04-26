#!/usr/bin/env python3

import rospy
from example_service.srv import *

def fibonacciClient(x):
    rospy.wait_for_service('calc_fibonacci')
    try:
        fibonacciProxy = rospy.ServiceProxy('calc_fibonacci', Fibonacci)
        serviceResponse = fibonacciProxy(x)
        return serviceResponse
    except rospy.ServiceException as e:
        print("Service call failed: %s", e)

if __name__ == '__main__':
    rospy.init_node('ActionServiceNode')
    rospy.logwarn(fibonacciClient(3))
    rospy.spin()
