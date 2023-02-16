#!/usr/bin/env python3

import numpy as np
import time
import rospy
from pupper_bringup.msg import PupperJointAngles
from pupper_bringup.msg import PupperFootPositions
import HardwareInterface
import argparse

import datetime
import os
import msgpack

DIRECTORY = "logs/"
FILE_DESCRIPTOR = "walking"

SERIAL_PORT = "/dev/ttyACM0"
class PupperBringup:

    def __init__(self):


        self.hardware_interface = HardwareInterface.HardwareInterface(port=SERIAL_PORT)

        rospy.sleep(0.1)

        self.setup_hardware()

        rospy.sleep(0.1)

        self.angle_sub = rospy.Subscriber("/joint_angles", PupperJointAngles, callback=self.joint_cb)
        self.foot_pos_sub = rospy.Subscriber("/foot_positions", PupperFootPositions, callback=self.foot_pos_cb)

        self.joint_angles = None
        self.foot_positions = None

        rospy.sleep(0.1)


    def setup_hardware(self):
        print("Setting up hardware interface")

        rospy.sleep(0.1)
        self.hardware_interface.serial_handle.reset_input_buffer()
        rospy.sleep(0.1)
        self.hardware_interface.activate()
        print("Hardware interface activated")
        rospy.sleep(0.1)


    def joint_cb(self, msg):
        self.joint_angles = np.array([[leg.x, leg.y, leg.z] for leg in msg.joint_angles]).T
        self.hardware_interface.set_actuator_positions(self.joint_angles)

    def foot_pos_cb(self, msg):
        self.foot_positions = np.array([[foot.x, foot.y, foot.z] for foot in msg.foot_positions]).T
        self.hardware_interface.set_cartesian_positions(self.foot_positions)  

    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.01)
        print("Deactivating Robot")
        rospy.sleep(0.1)
        self.hardware_interface.deactivate()
        rospy.sleep(0.1)


if __name__ == "__main__":
    
    rospy.init_node("pupper_bringup")
    pupper_bringup = PupperBringup()

    pupper_bringup.run()

