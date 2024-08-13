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
            self.orientation = list(r.as_euler("xyz", degrees=False))
        # Quaternion
        elif (
            isinstance(orientation, (tuple, list, np.ndarray)) and len(orientation) == 4
        ):
            r = R.from_quat(orientation)
            self.orientation = list(r.as_euler("xyz", degrees=False))
        # Euler angles
        elif (
            isinstance(orientation, (tuple, list, np.ndarray)) and len(orientation) == 3
        ):
            self.orientation = list(orientation)
        else:
            raise ValueError(
                "[Pose] \033[31merror\033[0m: Orientation input must be Rotation matrix / Quaternion / Euler angles"
            )

        self.roll, self.pitch, self.yaw = self.orientation
        
    def get_position(self):
        """
        get position
        
        Returns:
            list: position
        """
        return self.position
        
    def get_orientation(self, type="euler"):
        """
        get orientation
        
        Returns:
            list: orientation
        """
        if type=="euler":
            return self.orientation
        elif type=="quaternion":
            return R.from_euler('xyz', self.orientation).as_quat()
        elif type=="rotation_matrix":
            return R.from_euler('xyz', self.orientation).as_matrix()
        else:
            raise ValueError(
                "[Pose] \033[31merror\033[0m: Orientation input must be Rotation matrix / Quaternion / Euler angles"
            )
            
    def print(self, pose_description=""):
        """
        print pose
        """
        print(f"[Pose] \033[34mInfo\033[0m: {pose_description} position: [{self.position[0]:.3f}, {self.position[1]:.3f}, {self.position[2]:.3f}], orientation: [{self.orientation[0]:.3f}, {self.orientation[1]:.3f}, {self.orientation[2]:.3f}]")
