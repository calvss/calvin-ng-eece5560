#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

class ImageThresholder:
    WHITE = {
        "hueL" : 0,
        "hueH" : 180,
        "satL" : 0,
        "satH" : 35,
        "valL" : 235,
        "valH" : 255
    }

    YELLOW = {
        "hueL" : 27,
        "hueH" : 32,
        "satL" : 140,
        "satH" : 255,
        "valL" : 225,
        "valH" : 255
    }

    def __init__(self):

        self.bridge = CvBridge()

        rospy.Subscriber('/image', Image, self.callback)
        self.cropPublisher = rospy.Publisher("/image_cropped", Image, queue_size = 10)
        self.yellowPublisher = rospy.Publisher("/image_yellow", Image, queue_size = 10)
        self.whitePublisher = rospy.Publisher("/image_white", Image, queue_size = 10)

    def callback(self, msg):

        cvImage = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        rows, columns, _ = cvImage.shape
        croppedImage = cvImage[int(rows/2):]

        HSVImage = cv2.cvtColor(croppedImage, cv2.COLOR_BGR2HSV)

        element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        cleanImage = cv2.morphologyEx(HSVImage, cv2.MORPH_CLOSE, element)

        element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
        cleanImage = cv2.morphologyEx(cleanImage, cv2.MORPH_OPEN, element)

        whiteImage = cv2.inRange(cleanImage, (self.WHITE["hueL"], self.WHITE["satL"], self.WHITE["valL"]), (self.WHITE["hueH"], self.WHITE["satH"], self.WHITE["valH"]))
        yellowImage = cv2.inRange(cleanImage, (self.YELLOW["hueL"], self.YELLOW["satL"], self.YELLOW["valL"]), (self.YELLOW["hueH"], self.YELLOW["satH"], self.YELLOW["valH"]))

        element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))

        ros_cropped = self.bridge.cv2_to_imgmsg(croppedImage, "bgr8")
        ros_white = self.bridge.cv2_to_imgmsg(whiteImage, "mono8")
        ros_yellow = self.bridge.cv2_to_imgmsg(yellowImage, "mono8")

        self.cropPublisher.publish(ros_cropped)
        self.whitePublisher.publish(ros_white)
        self.yellowPublisher.publish(ros_yellow)

if __name__=="__main__":
    rospy.init_node("ImageThresholder")
    myNode = ImageThresholder()
    rospy.spin()
