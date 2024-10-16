# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : Bestman_sim_elephant.py
# @Time           : 2024-08-03 15:08:13
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : elephant robot
"""

import math
import time

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R

from .Bestman_sim import Bestman_sim
from .Pose import Pose


class Bestman_sim_elephant(Bestman_sim):
    """
    A class representing a simulation for the Bestman robot equipped with a elephant arm.
    """

    def __init__(self, client, visualizer, cfg):
        """
        Initialize the Bestman_sim_elephant with the given parameters.

        Args:
            client (int): The PyBullet client ID.
            visualizer (bool): Flag indicating whether visualization is enabled.
            cfg (dict): Configuration settings.
        """

        # Init parent class: BestMan_sim
        super().__init__(client, visualizer, cfg)

        # Init arm
        arm_pose = self.sim_get_sync_arm_pose()
        self.arm_id = self.client.load_object(
            obj_name="arm",
            model_path=self.robot_cfg.arm_urdf_path,
            object_position=arm_pose.get_position(),
            object_orientation=arm_pose.get_orientation(),
            fixed_base=True,
        )
        self.arm_jointInfo = self.sim_get_arm_all_jointInfo()
        self.arm_lower_limits = [info.lowerLimit for info in self.arm_jointInfo]
        self.arm_upper_limits = [info.upperLimit for info in self.arm_jointInfo]
        self.arm_joint_ranges = [
            info.upperLimit - info.lowerLimit for info in self.arm_jointInfo
        ]

        # Add constraint between base and arm
        p.createConstraint(
            parentBodyUniqueId=self.base_id,
            parentLinkIndex=-1,
            childBodyUniqueId=self.arm_id,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0],
            physicsClientId=self.client_id,
        )

        # Init arm joint angle
        self.sim_set_arm_to_joint_values(self.robot_cfg.arm_init_jointValues)

        # change robot color
        # self.visualizer.change_robot_color(self.base_id, self.arm_id, False)
        self.visualizer.set_object_color(self.base_id, "light_white")

    # ----------------------------------------------------------------
    # Functions for arm
    # ----------------------------------------------------------------

    def sim_get_sync_arm_pose(self):
        """
        Get synchronized pose of the robot arm with the base.
        """
        base_pose = self.sim_get_current_base_pose()
        arm_pose = Pose(
            [*base_pose.get_position()[:2], self.arm_place_height],
            base_pose.get_orientation(),
        )
        return arm_pose

    # ----------------------------------------------------------------
    # Functions for gripper
    # ----------------------------------------------------------------
