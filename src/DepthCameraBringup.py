#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import Point32
from utils.bridge import numpy_to_compressed_imgmsg
from cv_bridge import CvBridge
import sys
import cv2
import numpy as np
import ArducamDepthCamera as ac


class PointCloudComputer:

    def __init__(self, nrows: int, ncols: int, vertical_fov: float, horizontal_fov: float):
        self.nrows = nrows
        self.ncols = ncols
        self.fy = nrows/(2*np.tan((0.5*np.pi)*(vertical_fov/nrows)))
        self.fx = ncols/(2*np.tan((0.5*np.pi)*(horizontal_fov/nrows)))

        self.row_arr = np.tile(np.arange(self.nrows), (self.ncols, 1)).T
        self.col_arr = np.tile(np.arange(self.ncols), (self.nrows, 1))
        self.zero = np.zeros((nrows, ncols))
        self.msg = PointCloud2()
        self.msg.height=1
        self.msg.width=ncols*nrows
        self.msg.fields = [PointField()]*3
        self.msg.fields[0].name = "x"
        self.msg.fields[1].name = "y"
        self.msg.fields[2].name = "z"
        for i in range(3):
            self.msg.fields[i].offset = i*(nrows*ncols)
            self.msg.fields[i].datatype=7
            self.msg.fields[i].count = nrows*ncols
        # self.msg.points = [Point32()]*(self.nrows*self.ncols)
        # self.msg.channels = [ChannelFloat32()]
        # self.msg.channels[0].values = [0]*(self.nrows*self.ncols)


    def camera_info_msg(self):
        cinfo = CameraInfo()
        cinfo.height = self.nrows
        cinfo.width = self.ncols
        # cinfo.distortion_model=[self.fx, 0, 0, 0, self.ncols/2]
        cinfo.K = [self.fx, 0, self.ncols/2, 0, self.fy, self.nrows/2, 0, 0, 1]
        cinfo.header.stamp = rospy.Time().now()
        return cinfo




    '''
    Takes in the depth and amplitude data from the depth camera and uses it to compute
    a pointcloud relative to the camera. Returns the PointCloud ROS message as output
    '''
    def numpy_to_pcmsg(self, depth: np.ndarray, amplitude: np.ndarray) -> PointCloud2:
        valid_idxs = np.where(amplitude > 30)
        z_arr = np.where(amplitude > 30, depth, self.zero)
        x_arr = np.where(amplitude > 30, (((self.ncols/2) - self.col_arr) / self.fx) * z_arr, self.zero)
        y_arr = np.where(amplitude > 30, (((self.nrows/2) - self.row_arr) / self.fy) * z_arr, self.zero)
        self.msg.data = list(x_arr.flatten()) + list(y_arr.flatten()) + list(z_arr.flatten())
        # count = 0
        # self.msg.channels[0].values = list(depth.flatten())
        # for row_idx in range(self.nrows):
        #     for col_idx in range(self.ncols):
        #         self.msg.points[count].x = x_arr[row_idx, col_idx]
        #         self.msg.points[count].y = y_arr[row_idx, col_idx]
        #         self.msg.points[count].z = z_arr[row_idx, col_idx]
        #         count += 1
                
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = "pointcloud"
        return self.msg





MAX_DEPTH = 4




'''
Used to process the amplitude and depth data from the depth camera into an
image in the form of a numpy array.
'''
def process_frame(depth_buf: np.ndarray, amplitude_buf: np.ndarray) -> np.ndarray:
        
    depth_buf = np.nan_to_num(depth_buf)

    amplitude_buf[amplitude_buf<=7] = 0
    amplitude_buf[amplitude_buf>7] = 255

    depth_buf = (1 - (depth_buf/MAX_DEPTH)) * 255
    depth_buf = np.clip(depth_buf, 0, 255)
    result_frame = depth_buf.astype(np.uint8)  & amplitude_buf.astype(np.uint8)
    return result_frame


if __name__ == "__main__":
    rospy.init_node("pupper_depth_camera")
    pub = rospy.Publisher("camera/depth/image_raw", Image, queue_size=1)
    info_pub = rospy.Publisher("camera/depth/camera_info", CameraInfo, queue_size=1)
    point_cloud_pub = rospy.Publisher("/pointcloud2", PointCloud2, queue_size=1)
    bridge = CvBridge()
    dev = rospy.get_param("dev")
    cam = ac.ArducamCamera()
    rate = rospy.Rate(30)

    calc = PointCloudComputer(180, 240, 50.4, 64.3)
    
        
    if cam.init(ac.TOFConnect.CSI,dev) != 0 :
        rospy.logerr("initialization failed")
    if cam.start(ac.TOFOutput.DEPTH) != 0 :
        rospy.logerr("Failed to start camera")
    
    cam.setControl(ac.TOFControl.RANG,MAX_DEPTH)
    while not rospy.is_shutdown():
        frame = cam.requestFrame(200)
        rate.sleep()
        if frame != None:
            depth_buf = frame.getDepthData()
            amplitude_buf = frame.getAmplitudeData()
            cam.releaseFrame(frame)
            point_cloud_msg = calc.numpy_to_pcmsg(depth_buf, amplitude_buf)
            point_cloud_pub.publish(point_cloud_msg)
            amplitude_buf*=(255/1024)
            amplitude_buf = np.clip(amplitude_buf, 0, 255)
            rospy.loginfo(str(depth_buf.dtype))
            info = calc.camera_info_msg()
            img = process_frame(depth_buf,amplitude_buf)
            # rospy.loginfo(img.shape)
            # info_pub.publish(info)
            pub.publish(bridge.cv2_to_imgmsg(img, encoding="mono8"))
            
        else:
            rospy.logwarn("Did not recieve frame")
    # finally:
    #     cam.stop()
