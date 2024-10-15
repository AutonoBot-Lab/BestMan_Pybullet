# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : Pose.py
# @Time           : 2024-08-03 15:08:38
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : POse
"""


import numpy as np
from scipy.spatial.transform import Rotation as R


class Pose:
    """
    A class representing a 3D pose consisting of position and orientation.
    """

    def __init__(self, position, orientation):
        """
        Initialize a new Pose object.

        Args:
            position (list / np.ndarray): A list or array of three floats representing the position in 3D space.
            orientation (list, tuple, np.ndarray): A list, tuple, or array representing the orientation in 3D space.
                It can be either Euler angles (3 elements), a quaternion (4 elements), or a 3x3 rotation matrix.
        """

        self.position = list(position)
        self.x, self.y, self.z = position

        # Rotation matrix
        if isinstance(orientation, np.ndarray) and orientation.shape == (3, 3):
            r = R.from_matrix(orientation)
            self.orientation = list(r.as_quat())
        # Quaternion
        elif (
            isinstance(orientation, (tuple, list, np.ndarray)) and len(orientation) == 4
        ):
            self.orientation = list(orientation)
        # Euler angles
        elif (
            isinstance(orientation, (tuple, list, np.ndarray)) and len(orientation) == 3
        ):
            r = R.from_euler("xyz", orientation, degrees=False)
            self.orientation = list(r.as_quat())
        else:
            raise ValueError(
                "[Pose] \033[31merror\033[0m: Orientation input must be Rotation matrix / Quaternion / Euler angles"
            )

    def get_position(self):
        """
        get position

        Returns:
            list: position
        """
        return self.position

    def get_orientation(self, type="quaternion"):
        """
        get orientation

        Returns:
            list: orientation
        """
        if type == "quaternion":
            return self.orientation
        elif type == "euler":
            return R.from_quat(self.orientation).as_euler("xyz", degrees=False)
        elif type == "rotation_matrix":
            return R.from_quat(self.orientation).as_matrix()
        else:
            raise ValueError(
                "[Pose] \033[31merror\033[0m: Orientation type must be 'quaternion', 'euler', or 'rotation_matrix'"
            )

    def print(self, pose_description="", type="quaternion"):
        """
        print pose
        """
        if type == "quaternion":
            print(
                f"[Pose] \033[34mInfo\033[0m: {pose_description} position: [{self.position[0]:.3f}, {self.position[1]:.3f}, {self.position[2]:.3f}], orientation: [{', '.join([f'{x:.3f}' for x in self.get_orientation('quaternion')])}]"
            )
        elif type == "euler":
            print(
                f"[Pose] \033[34mInfo\033[0m: {pose_description} position: [{self.position[0]:.3f}, {self.position[1]:.3f}, {self.position[2]:.3f}], orientation: [{', '.join([f'{x:.3f}' for x in self.get_orientation('euler')])}]"
            )
        elif type == "rotation_matrix":
            np.set_printoptions(precision=3, suppress=True)
            print(
                f"[Pose] \033[34mInfo\033[0m: {pose_description} position: [{self.position[0]:.3f}, {self.position[1]:.3f}, {self.position[2]:.3f}], orientation: {np.array2string(self.get_orientation('rotation_matrix'), formatter={'float_kind':lambda x: f'{x:.3f}'})}"
            )
        else:
            raise ValueError(
                "[Pose] \033[31merror\033[0m: Orientation type must be 'quaternion', 'euler', or 'rotation_matrix'"
            )
