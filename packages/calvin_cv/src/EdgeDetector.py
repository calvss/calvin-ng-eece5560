#!/usr/bin/env python3

import rospy
import cv2
import queue
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

class EdgeDetector:
    def __init__(self):
        self.bridge = CvBridge()

        rospy.Subscriber('/image_yellow', Image, self.yellowCallback)
        rospy.Subscriber('/image_white', Image, self.whiteCallback)
        rospy.Subscriber('/image_cropped', Image, self.croppedCallback)
        self.edgesPublisher = rospy.Publisher("/image_edges", Image, queue_size = 10)
        self.yellowPublisher = rospy.Publisher("/image_lines_yellow", Image, queue_size = 10)
        self.whitePublisher = rospy.Publisher("/image_lines_white", Image, queue_size = 10)

        self.yellowEdges = queue.SimpleQueue()
        self.whiteEdges = queue.SimpleQueue()
        self.croppedImages = queue.SimpleQueue()

    def yellowCallback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        houghLines = self.getHough(image, 5)
        self.yellowEdges.put(houghLines)

    def whiteCallback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        houghLines = self.getHough(image, 15)
        self.whiteEdges.put(houghLines)

    def croppedCallback(self, msg):
        whiteEdges = self.whiteEdges.get()
        yellowEdges = self.yellowEdges.get()

        croppedImage = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        edges = cv2.Canny(croppedImage, 100, 200)

        whiteLines = self.drawLines(croppedImage, whiteEdges)
        yellowLines = self.drawLines(croppedImage, yellowEdges)

        ros_all_edges = self.bridge.cv2_to_imgmsg(edges, "mono8")
        ros_white_edges = self.bridge.cv2_to_imgmsg(whiteLines, "bgr8")
        ros_yellow_edges = self.bridge.cv2_to_imgmsg(yellowLines, "bgr8")

        self.edgesPublisher.publish(ros_all_edges)
        self.whitePublisher.publish(ros_white_edges)
        self.yellowPublisher.publish(ros_yellow_edges)

    def getHough(self, image, length):
        #element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (thickness, thickness))
        #binaryEdges = cv2.morphologyEx(image, cv2.MORPH_GRADIENT, element)

        binaryEdges = cv2.Canny(image, 100, 200)

        houghLines = cv2.HoughLinesP(binaryEdges, 1, 3.14159/180, 15, None, length, 10)
        return houghLines

    def drawLines(self, image, lines):
        output =  image.copy()
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output

if __name__=="__main__":
    rospy.init_node("EdgeDetector")
    myNode = EdgeDetector()
    rospy.spin()
