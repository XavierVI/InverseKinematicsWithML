#!/usr/bin/env python3

import argparse
import sys
import time
import copy
from threading import Lock

from interbotix_common_modules.common_robot.robot import (
    robot_shutdown, robot_startup
)
from interbotix_common_modules.angle_manipulation import angle_manipulation as ang
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
import rclpy
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R

import torch
from torch import nn

import os


class ArmControllerWithML(nn.Module):
    def __init__(self):
        super(ArmControllerWithML, self).__init__()
        self.hidden_layer = nn.Linear(3, 100)
        self.output_layer = nn.Linear(100, 3)
        self.activation_function = nn.Tanh()

    def forward(self, x):
        x = self.activation_function(self.hidden_layer(x))
        x = self.output_layer(x)
        return x

class ArmController(InterbotixManipulatorXS):
    current_loop_rate = 10 # in Hz
    moving_time = 0.2
    accel_time = 0.1
    lock = Lock()
    desired_pose = Pose()
    model = ArmControllerWithML()

    def __init__(self, pargs, args=None):
        InterbotixManipulatorXS.__init__(
            self,
            robot_model=pargs.robot_model,
            robot_name=pargs.robot_name,
            moving_time=self.moving_time,
            accel_time=self.accel_time,
            args=args,
        )
        # -------------------------------------
        # Loading the model's weights
        # -------------------------------------
        self.model.load_state_dict(torch.load(
            os.path.join(
                os.getcwd(),
                'models',
                'fnn_100.pth'
            ), weights_only=True))
        self.model.eval()
        
        # sets the rate at which the control loop will run
        self.rate = self.core.get_node().create_rate(self.current_loop_rate)
        
        # set the upper and lower limits for the waist joint
        self.waist_index = self.arm.group_info.joint_names.index('waist')
        self.waist_ll = self.arm.group_info.joint_lower_limits[self.waist_index]
        self.waist_ul = self.arm.group_info.joint_upper_limits[self.waist_index]

        # The transformation matrix between the space frame and the virtual frame
        self.T_sy = np.identity(4)

        # The transformation matrix between the virtual frame and the body frame
        self.T_yb = np.identity(4)

        self.update_T_virtual_to_body()
        self.core.get_node().create_subscription(
            Pose,
            'px100_target_pose',
            self.update_desired_pose_cb,
            10
        )
        time.sleep(0.5)
        self.core.get_node().loginfo(f'Robot name: {pargs.robot_name}')

    def start_robot(self) -> None:
        self.arm.go_to_home_pose(
            moving_time=1.5,
            accel_time=0.75
        )
        self.update_T_virtual_to_body()

        try:
            robot_startup()
            while rclpy.ok():
                self.move_end_effector()
                self.rate.sleep()
        
        except KeyboardInterrupt:
            robot_shutdown()


    def update_T_virtual_to_body(self) -> None:
        """
        Helper function that calculates the pose of
        the end effector w.t.t the virtual frame.
        """
        # Get the latest command
        T_sb = self.arm.get_ee_pose_command()
        rpy = ang.rotation_matrix_to_euler_angles(T_sb[:3, :3])
        # update yaw angle
        self.T_sy[:2, :2] = ang.yaw_to_rotation_matrix(rpy[2])
        # Updates the end-effectors position relative to base frame
        T_sy_inv = ang.trans_inv(self.T_sy)
        self.T_yb = T_sy_inv @ T_sb

    def move_end_effector(self) -> None:
        """
        This function moves the end effector to the desired position
        by computing the inverse kinematics.
        """
        # create a local copy of the desired pose using a lock to avoid race conditions
        with self.lock:
            desired_pose = self.desired_pose

        # creates a rotation matrix from the desired pose's quaternion
        desired_rotation_matrix = np.eye(4)
        desired_rotation_matrix[:3, :3] = R.from_quat([
            desired_pose.orientation.x,
            desired_pose.orientation.y,
            desired_pose.orientation.z,
            desired_pose.orientation.w,
        ]).as_matrix()

        # get the desired roll, pitch, yaw angles from the rotation matrix
        desired_rpy = ang.rotation_matrix_to_euler_angles(
            desired_rotation_matrix)

        # create a transformation matrix from the desired pose and the rotation matrix
        desired_trans_matrix = np.eye(4)
        desired_trans_matrix[:3, :3] = desired_rotation_matrix[:3, :3]
        desired_trans_matrix[0, 3] = desired_pose.position.x
        desired_trans_matrix[1, 3] = desired_pose.position.y
        desired_trans_matrix[2, 3] = desired_pose.position.z

        # compute the transformation matrix for the desired end-effector position
        # w.r.t. the virtual frame
        desired_T_yb = np.linalg.inv(
            desired_rotation_matrix) @ desired_trans_matrix

        with torch.no_grad():
            input_tensor = torch.tensor([
                # x and y coordinates
                desired_T_yb[0, 3],
                desired_T_yb[2, 3],
                # pitch angle
                desired_rpy[1]
            ], dtype=torch.float32).unsqueeze(0)

            joints = self.model(input_tensor)

            success = self.arm.set_joint_positions(
                joint_positions=[
                    desired_rpy[2],  # waist angle
                    joints[0, 0].item(),
                    joints[0, 1].item(),
                    joints[0, 2].item()
                ]
            )

            if not success:
                self.core.get_node().logwarn(f'Failed to set joint positions: {joints}')
                
    def update_desired_pose_cb(self, msg: Pose):
        # self.log_info(f'Thread ID (desired pose cb): {get_ident()}')

        # if the pose isn't different from the current one, just return
        if self.desired_pose.position == msg.position \
             and self.desired_pose.orientation == msg.orientation:
            return

        with self.lock:
            self.desired_pose = copy.deepcopy(msg)

    def log_info(self, msg):
        self.core.get_node().get_logger().info(f'{msg}')


def main(args=None):
    p = argparse.ArgumentParser()
    p.add_argument('--robot_model')
    p.add_argument('--robot_name', default=None)
    p.add_argument('args', nargs=argparse.REMAINDER)

    command_line_args = remove_ros_args(args=sys.argv)[1:]
    ros_args = p.parse_args(command_line_args)

    bot = ArmController(ros_args, args=args)
    bot.start_robot()


if __name__ == '__main__':
    main()
