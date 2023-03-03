#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from utils.bridge import numpy_to_imgmsg
import sys
import cv2
import numpy as np
import ArducamDepthCamera as ac

MAX_DISTANCE=4

def process_frame(depth_buf: np.ndarray, amplitude_buf: np.ndarray) -> np.ndarray:
        
    depth_buf = np.nan_to_num(depth_buf)

    amplitude_buf[amplitude_buf<=7] = 0
    amplitude_buf[amplitude_buf>7] = 255

    depth_buf = (1 - (depth_buf/MAX_DISTANCE)) * 255
    depth_buf = np.clip(depth_buf, 0, 255)
    result_frame = depth_buf.astype(np.uint8)  & amplitude_buf.astype(np.uint8)
    return result_frame




if __name__ == "__main__":
    rospy.init_node("depth_camera")
    pub = rospy.Publisher("/depth_cam/compressed", CompressedImage, queue_size=1)
    cam = ac.ArducamCamera()
    rate = rospy.Rate(30)
        
    if cam.init(ac.TOFConnect.CSI,1) != 0 :
        rospy.logerr("initialization failed")
    if cam.start(ac.TOFOutput.RAW) != 0 :
        rospy.logerr("Failed to start camera")
    
    cam.setControl(ac.TOFControl.RANG,MAX_DISTANCE)
    while not rospy.is_shutdown():
        frame = cam.requestFrame(200)
        rate.sleep()
        if frame != None:
            depth_buf = frame.getDepthData()
            amplitude_buf = frame.getAmplitudeData()
            cam.releaseFrame(frame)
            amplitude_buf*=(255/1024)
            amplitude_buf = np.clip(amplitude_buf, 0, 255)
            img = process_frame(depth_buf,amplitude_buf)
            # rospy.loginfo(img.shape)
            pub.publish(numpy_to_imgmsg(img))
        else:
            rospy.logwarn("Did not recieve frame")
    # finally:
    #     cam.stop()
