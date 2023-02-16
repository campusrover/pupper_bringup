#!/usr/bin/env python3

import cv2
import numpy
import rospy
from sensor_msgs.msg import CompressedImage

cam = cv2.VideoCapture(0)
img_pub = rospy.Publisher("/image/compressed", CompressedImage, queue_size=10)


while not rospy.is_shutdown():
    ret, image = cam.read()
    print(type(image))

cam.release()
