import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage

def numpy_to_imgmsg(im):
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', im)[1]).tobytes()
    return msg


def imgmsg_to_numpy(msg):
    np_arr = np.fromstring(msg.data, np.uint8)
    return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)