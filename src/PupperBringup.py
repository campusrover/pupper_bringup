#/usr/bin/env python3

import numpy as np
import time
import rospy
from pupper_bringup.msg import PupperJointAngles
from pupper_bringup.msg import PupperFootPositions
from pupper_bringup import HardwareInterface
import argparse

import datetime
import os
import msgpack

DIRECTORY = "logs/"
FILE_DESCRIPTOR = "walking"

SERIAL_PORT = "/dev/ttyACM0"
class PupperBringup:

    def __init__(self, flags):

        self.FLAGS = flags

        self.hardware_interface = HardwareInterface.HardwareInterface(port=SERIAL_PORT)

        rospy.sleep(0.1)

        self.setup_hardware()

        rospy.sleep(0.1)

        self.angle_sub = rospy.subscriber("/joint_angles", PupperJointAngles, cb=self.joint_cb)
        self.foot_pos_sub = rospy.subscriber("/foot_positions", PupperFootPositions, cb=self.foot_pos_cb)

        self.joint_angles = None
        self.foot_positions = None

        rospy.sleep(0.1)


    def setup_hardware(self):

        if self.FLAGS.log:
            today_string = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
            filename = os.path.join(
                DIRECTORY, FILE_DESCRIPTOR + "_" + today_string + ".csv"
            )
            self.log_file = open(filename, "w")
            self.hardware_interface.write_logfile_header(self.log_file)

        if self.FLAGS.zero:
            self.hardware_interface.set_joint_space_parameters(0, 4.0, 4.0)
            self.hardware_interface.set_actuator_postions(np.zeros((3,4)))
            input(
                "Do you REALLY want to calibrate? Press enter to continue or ctrl-c to quit."
            )
            print("Zeroing motors...", end="")
            self.hardware_interface.zero_motors()
            self.hardware_interface.set_max_current_from_file()
            print("Done.")
        else:
            print("Not zeroing motors!")

        if self.FLAGS.home:
            print("Homing motors...", end="", flush=True)
            self.hardware_interface.home_motors()
            time.sleep(5)
            print("Done.")
        rospy.sleep(0.1)
        self.hardware_interface.serial_handle.reset_input_buffer()
        rospy.sleep(0.1)
        self.hardware_interface.activate()
        rospy.sleep(0.1)


    def joint_cb(self, msg):
        self.joint_angles = np.array([
            [msg.leg_0.x, msg.leg_1.x, msg.leg_2.x, msg.leg_3.x],
            [msg.leg_0.y, msg.leg_1.y, msg.leg_2.y, msg.leg_3.y],
            [msg.leg_0.z, msg.leg_1.z, msg.leg_2.z, msg.leg_3.z]
        ])
        self.hardware_interface.set_actuator_positions(self.joint_angles)

    def foot_pos_cb(self, msg):
        self.foot_positions = np.array([
            [msg.foot_0.x, msg.foot_1.x, msg.foot_2.x, msg.foot_3.x],
            [msg.foot_0.y, msg.foot_1.y, msg.foot_2.y, msg.foot_3.y],
            [msg.foot_0.z, msg.foot_1.z, msg.foot_2.z, msg.foot_3.z]
        ])
        self.hardware_interface.set_cartesian_positions(self.foot_positions)  

    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.01)
            if self.FLAGS.log:
                any_data = self.hardware_interface.log_incoming_data(self.log_file)
            # if self.joint_angles is not None:
            #     self.hardware_interface.set_actuator_positions(self.joint_angles)
            # elif self.foot_positions is not None:
            #     self.hardware_interface.set_cartesian_positions(self.foot_positions)
                
        print("Deactivating Robot")
        rospy.sleep(0.1)
        self.hardware_interface.deactivate()
        rospy.sleep(0.1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--zero", help="zero the motors", action="store_true")
    parser.add_argument("--log", help="log pupper data to file", action="store_true")
    parser.add_argument("--home", help="home the motors (moves the legs)", action="store_true")
    FLAGS = parser.parse_args()
    
    rospy.init_node("pupper_bringup")
    pupper_bringup = PupperBringup(FLAGS)

    pupper_bringup.run()

