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
import rclpy
from rclpy.utilities import remove_ros_args
import modern_robotics as mr
from arm_controller.msg import PositionAndJoints as P

class DataGenerator(InterbotixManipulatorXS):
    csv_path = '/home/xavier/datasets/'

    def __init__(self, pargs, args=None):
        InterbotixManipulatorXS.__init__(
            self,
            robot_model=pargs.robot_model,
            robot_name=pargs.robot_name,
            args=args,
        )    
        
        self.publisher = self.core.get_node().create_publisher(
            P,
            'position_data',
            10
        )
        time.sleep(0.5)
        self.core.get_node().loginfo(f'Robot name: {pargs.robot_name}')

    def start_robot(self) -> None:
        try:
            robot_startup()
            self.generate_data()
        except KeyboardInterrupt:
            robot_shutdown()


    def generate_data(self):
        lower_limits = self.arm.group_info.joint_lower_limits
        upper_limits = self.arm.group_info.joint_upper_limits
        M = self.arm.robot_des.M
        Slist = self.arm.robot_des.Slist
        size = 20_000

        joint_angles_list = np.random.uniform(
            low=lower_limits,  # Use the entire lower_limits array
            high=upper_limits,  # Use the entire upper_limits array
            # Generate a (size x num_joints) array
            size=(size, len(lower_limits))
        )

        for joint_angle in joint_angles_list:
                        self.publish_data(M, Slist, joint_angle)

                    
    def publish_data(self, M, Slist, thetalist):
        fk_position = mr.FKinSpace(M, Slist, thetalist)
        rot_matrix = np.eye(4)
        rot_matrix[:3, :3] = fk_position[:3, :3] # columns 0,1,2 and rows 0,1,2
        rpy = ang.rotation_matrix_to_euler_angles(rot_matrix)
        
        # computing the virtual frame position
        virtual_fk_position = np.linalg.inv(rot_matrix) @ fk_position

        msg = P()
        msg.x = virtual_fk_position[0, 3]
        # msg.position.y = cartesian_position[1]
        # msg.position.z = cartesian_position[2]
        msg.y = virtual_fk_position[2, 3] # z-axis is our vertical
        msg.pitch = rpy[1]
        # msg.yaw = rpy[2]
        # msg.waist = thetalist[0]
        msg.shoulder = thetalist[1]
        msg.elbow = thetalist[2]
        msg.wrist = thetalist[3]
        # self.publisher.publish(msg)

    def log_info(self, msg):
        self.core.get_node().get_logger().info(f'{msg}')


def main(args=None):
    p = argparse.ArgumentParser()
    p.add_argument('--robot_model')
    p.add_argument('--robot_name', default=None)
    p.add_argument('csv_path', default='/home/xavier/projects/datasets/data.csv')
    p.add_argument('args', nargs=argparse.REMAINDER)

    command_line_args = remove_ros_args(args=sys.argv)[1:]
    ros_args = p.parse_args(command_line_args)

    bot = DataGenerator(ros_args, args=args)
    bot.start_robot()


if __name__ == '__main__':
    main()
