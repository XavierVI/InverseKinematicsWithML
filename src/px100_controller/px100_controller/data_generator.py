#!/usr/bin/env python3

import argparse
import sys
import time

from interbotix_common_modules.common_robot.robot import (
    robot_shutdown, robot_startup
)
from interbotix_common_modules.angle_manipulation import angle_manipulation as ang
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
from rclpy.utilities import remove_ros_args
import modern_robotics as mr
from arm_controller_msgs.msg import Configuration as C

class DataGenerator(InterbotixManipulatorXS):
    def __init__(self, pargs, args=None):
        InterbotixManipulatorXS.__init__(
            self,
            robot_model=pargs.robot_model,
            robot_name=pargs.robot_name,
            args=args,
        )
        self.verbose = pargs.verbose
        self.publisher = self.core.get_node().create_publisher(
            C,
            'position_data',
            10
        )        
        time.sleep(0.5)
        self.core.get_node().loginfo(f'Robot name: {pargs.robot_name}')

    def start_robot(self) -> None:
        try:
            robot_startup()
            self.generate_data()
            robot_shutdown()
        except KeyboardInterrupt:
            robot_shutdown()


    def generate_data(self):
        # lower and upper limits for the joint angles
        lower_limits = self.arm.group_info.joint_lower_limits
        upper_limits = self.arm.group_info.joint_upper_limits
        # home position matrix and screw-axis list
        M = self.arm.robot_des.M
        Slist = self.arm.robot_des.Slist
        # size of the dataset
        size = 1_000

        # uniform set of joint angles
        joint_angles_list = np.random.uniform(
            low=lower_limits,  # Use the entire lower_limits array
            high=upper_limits,  # Use the entire upper_limits array
            # Generate a (size x num_joints) array
            size=(size, len(lower_limits))
        )
        # keep the waist angle at 0
        joint_angles_list[:, 0] = 0

        self.log_info(f'{joint_angles_list}')

        for joint_angles in joint_angles_list:
            self.publish_data(M, Slist, joint_angles)

        self.log_info('Done')

                    
    def publish_data(self, M, Slist, thetalist):
        # if the joint angles are not a valid configuration
        # return
        if not self.arm.set_joint_positions(thetalist):
            if self.verbose:
                self.log_info(f'Joint angles: {thetalist} failed')
            return

        # a transformation matrix representing the pose of the robot arm
        T_sb = mr.FKinSpace(M, Slist, thetalist)
        rot_matrix = np.eye(4)
        rot_matrix[:3, :3] = T_sb[:3, :3] # columns 0,1,2 and rows 0,1,2
        rpy = ang.rotation_matrix_to_euler_angles(rot_matrix)

        msg = C()
        msg.x = T_sb[0, 3]
        msg.y = T_sb[1, 3]
        msg.z = T_sb[2, 3]
        msg.pitch = rpy[1]
        msg.yaw = rpy[2]
        msg.waist = thetalist[0]
        msg.shoulder = thetalist[1]
        msg.elbow = thetalist[2]
        msg.wrist = thetalist[3]
        
        if self.verbose:
            self.log_info(f'publishing {thetalist}')

        self.publisher.publish(msg)

    def log_info(self, msg):
        self.core.get_node().get_logger().info(f'{msg}')


def main(args=None):
    p = argparse.ArgumentParser()
    p.add_argument('--robot_model')
    p.add_argument('--robot_name', default=None)
    p.add_argument('--size', default=1_000)
    p.add_argument('--verbose', default=False)
    p.add_argument('args', nargs=argparse.REMAINDER)

    command_line_args = remove_ros_args(args=sys.argv)[1:]
    ros_args = p.parse_args(command_line_args)

    node = DataGenerator(ros_args, args=args)
    node.start_robot()


if __name__ == '__main__':
    main()
