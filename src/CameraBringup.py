#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image




def numpy_to_imgmsg(im):
    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.height = im.shape[0]
    msg.width = im.shape[1]
    msg.encoding = "bgr8"
    msg.is_bigendian = False
    msg.step = 3 * msg.width
    msg.data = im.tobytes()
    return msg

rospy.init_node("pupper_camera_bringup")
cam = cv2.VideoCapture(0)
img_pub = rospy.Publisher("/image/raw", Image, queue_size=10)
rate = rospy.Rate(30)


while not rospy.is_shutdown():
    rate.sleep()
    ret, image = cam.read()
    img_pub.publish(numpy_to_imgmsg(image))

cam.release()
