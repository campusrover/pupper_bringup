import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32

fx = 240 / (2 * np.tan(0.5 * np.pi * 64.3 / 180))
fy = 180 / (2 * np.tan(0.5 * np.pi * 50.4 / 180))


def numpy_to_imgmsg(im):
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', im)[1]).tobytes()
    return msg


def imgmsg_to_numpy(msg):
    np_arr = np.fromstring(msg.data, np.uint8)
    return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

def numpy_to_pcmsg(depth, amplitude):
    pc_msg = PointCloud()
    pc_msg.points = [Point32()]*43200
    pc_msg.channels = [ChannelFloat32()]
    pc_msg.channels[0].values = [0]*43200

    for row_idx in range(180):
        for col_idx in range(240):
            if amplitude[row_idx][col_idx]> 30:
                zz = depth[row_idx][col_idx]
                pc_msg.points[col_idx].x = (((120 - col_idx)) / fx) * zz
                pc_msg.points[col_idx].y = ((90 - row_idx) / fy) * zz
                pc_msg.points[col_idx].z = zz
                pc_msg.channels[0].values[col_idx] = depth[col_idx]
            else:
                pc_msg.points[col_idx].x = 0
                pc_msg.points[col_idx].y = 0
                pc_msg.points[col_idx].z = 0
                pc_msg.channels[0].values[col_idx] = 0
    pc_msg.header.stamp = rospy.Time.now()
    return pc_msg
