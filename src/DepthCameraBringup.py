#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import ArducamDepthCamera as ac
from sensor_msgs.msg import CompressedImage
from utils.bridge import numpy_to_imgmsg


if __name__ == "__main__":
    rospy.init_node("depth_camera")

    pub = rospy.Publisher("/depth_camera", CompressedImage, queue_size=1)

    cam = ac.ArducamCamera()
    if cam.init(ac.TOFConnect.CSI,0) != 0 :
        print("initialization failed")
    if cam.start(ac.TOFOutput.RAW) != 0 :
        print("Failed to start camera")
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        rate.sleep()
        frame = cam.requestFrame(200)
        if frame != None:
            buf = frame.getRawData()
            cam.releaseFrame(frame)
            image = buf.astype(np.float32)
            pub.publish(numpy_to_imgmsg(image))
    
    cam.stop()
