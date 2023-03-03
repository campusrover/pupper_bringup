#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from utils.bridge import numpy_to_imgmsg
import sys
import cv2
import numpy as np
import ArducamDepthCamera as ac

if __name__ == "__main__":
    rospy.init_node("depth_camera")
    pub = rospy.Publisher("/depth_cam/compressed", CompressedImage, queue_size=1)
    cam = ac.ArducamCamera()
    rate = rospy.Rate(30)
    if cam.init(ac.TOFConnect.CSI,1) != 0 :
        rospy.logerr("initialization failed")
    if cam.start(ac.TOFOutput.RAW) != 0 :
        rospy.logerr("Failed to start camera")

    while not rospy.is_shutdown():
        frame = cam.requestFrame(200)
        rate.sleep()
        if frame != None:
            buf = frame.getRawData()
            cam.releaseFrame(frame)
            img = buf.astype(np.float32)
            # rospy.loginfo(img.shape)
            pub.publish(numpy_to_imgmsg(img))
        else:
            rospy.logwarn("Did not recieve frame")
    cam.stop()
