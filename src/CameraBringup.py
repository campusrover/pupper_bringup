#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage




def numpy_to_imgmsg(im):
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', im)[1]).tobytes()
    return msg

rospy.init_node("pupper_camera_bringup")
cam = cv2.VideoCapture(0)
img_pub = rospy.Publisher("/image/compressed", CompressedImage, queue_size=10)
rate = rospy.Rate(30)


while not rospy.is_shutdown():
    rate.sleep()
    ret, image = cam.read()
    img_pub.publish(numpy_to_imgmsg(image))

cam.release()
