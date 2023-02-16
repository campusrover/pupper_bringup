#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge
import numpy
import rospy
from sensor_msgs.msg import Image

bridge = CvBridge()
cam = cv2.VideoCapture(0)
img_pub = rospy.Publisher("/image/raw", Image, queue_size=10)
rate = rospy.Rate(30)


while not rospy.is_shutdown():
    rate.sleep()
    ret, image = cam.read()
    img_pub.publish(bridge.cv2_to_imgmsg(image))

cam.release()
