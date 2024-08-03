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
            self.orientation = list(r.as_euler('xyz', degrees=False))
        # Quaternion
        elif isinstance(orientation, (tuple, list, np.ndarray)) and len(orientation) == 4:
            r = R.from_quat(orientation)
            self.orientation = list(r.as_euler('xyz', degrees=False))
        # Euler angles
        elif isinstance(orientation, (tuple, list, np.ndarray)) and len(orientation) == 3:
            self.orientation = list(orientation)
        else:
            raise ValueError("Orientation input must be Rotation matrix / Quaternion / Euler angles")
        
        self.roll, self.pitch, self.yaw = self.orientation