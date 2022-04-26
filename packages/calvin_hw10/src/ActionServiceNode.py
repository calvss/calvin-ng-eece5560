#!/usr/bin/env python3

import rospy

from example_service.srv import *
import actionlib
import example_action_server.msg

def fibonacciServiceTimer(x):
    rospy.wait_for_service('calc_fibonacci')
    try:
        fibonacciProxy = rospy.ServiceProxy('calc_fibonacci', Fibonacci)

        startTime = rospy.get_time()
        serviceResponse = fibonacciProxy(x)
        elapsedTime = rospy.get_time() - startTime
        return [elapsedTime, serviceResponse]
    except rospy.ServiceException as e:
        print("Service call failed: %s", e)

def fibonacciActionTimer(x):
    client = actionlib.SimpleActionClient('fibonacci', example_action_server.msg.FibonacciAction)
    client.wait_for_server()
    goal = example_action_server.msg.FibonacciGoal(order=x)

    startTime = rospy.get_time()
    client.send_goal(goal)
    blockTime = rospy.get_time()
    client.wait_for_result()
    resultTime = rospy.get_time()
    return [blockTime - startTime, resultTime - blockTime, client.get_result()]

if __name__ == '__main__':
    rospy.init_node('ActionServiceNode')
    rospy.sleep(5)

    rospy.logwarn("Sending service request for order(3). Blocking. . .")
    rospy.logwarn(fibonacciServiceTimer(3))

    rospy.logwarn("Sending service request for order(15). Blocking. . .")
    rospy.logwarn(fibonacciServiceTimer(15))

    rospy.logwarn("Timing action for order(3). . .")
    result = fibonacciActionTimer(3)
    rospy.logwarn(result[2])
    rospy.logwarn(str(result[0]) + " seconds to return from request.")
    rospy.logwarn(str(result[1]) + " seconds to receive response.")

    rospy.logwarn("Timing action for order(15). . .")
    result = fibonacciActionTimer(15)
    rospy.logwarn(result[2])
    rospy.logwarn(str(result[0]) + " seconds to return from request.")
    rospy.logwarn(str(result[1]) + " seconds to receive response.")

    rospy.spin()
