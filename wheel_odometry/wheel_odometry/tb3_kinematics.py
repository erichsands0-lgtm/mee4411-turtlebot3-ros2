from rclpy.time import Time
from sensor_msgs.msg import JointState

import numpy as np
from typing import List

from tb3_utils import TB3Params


class TB3Kinematics(TB3Params):

    def __init__(self, robot_model) -> None:
        super().__init__(robot_model)

    def calculate_wheel_change(
            self,
            new_joint_states: JointState,
            prev_joint_states: JointState) -> List[float]:
        """
        Calculate the change in wheel angles and time between two joint state messages.

        Inputs:
            new_joint_states    new joint states (sensor_msgs/msg/JointState)
            prev_joint_states   previous joint states (sensor_msgs/msg/JointState)
        Outputs:
            delta_wheel_l       change in left wheel angle [rad]
            delta_wheel_r       change in right wheel angle [rad]
            delta_time          change in time [s]
        """
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Calculate the output values
        delta_wheel_l = 0.0
        delta_wheel_r = 0.0
        delta_time = 0.0
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266

        # Data validation
        if np.isnan(delta_wheel_l):
            delta_wheel_l = 0.0
        if np.isnan(delta_wheel_r):
            delta_wheel_r = 0.0

        return (delta_wheel_l, delta_wheel_r, delta_time)

    def calculate_displacement(
            self,
            delta_wheel_l: float,
            delta_wheel_r: float) -> List[float]:
        """
        Calculate the displacement of the robot based on the change in wheel angles.

        Inputs:
            delta_wheel_l       change in left wheel angle [rad]
            delta_wheel_r       change in right wheel angle [rad]
        Outputs:
            delta_s         linear displacement [m]
            delta_theta     angular displacement [rad]
        """
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Calculate the output values
        delta_s = 0.0  # linear displacement [m]
        delta_theta = 0.0  # angular displacement [rad]
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266

        return (delta_s, delta_theta)

    def calculate_pose(
            self,
            prev_pose: List[float],
            delta_s: float,
            delta_theta: float) -> List[float]:
        """
        Calculate the new pose of the robot based on the previous pose and the displacement.

        Inputs:
            prev_pose       input pose in format (x, y, theta) [m, m, rad]
            delta_s         linear displacement [m]
            delta_theta     angular displacement [rad]
        Outputs:
            pose            output pose in format (x, y, theta) [m, m, rad]
        """
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Calculate the output values
        pose = prev_pose  # FILL THIS IN
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266

        return pose
