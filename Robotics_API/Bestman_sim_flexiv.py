# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : Bestman_sim_flexiv.py
# @Time           : 2024-08-03 15:08:13
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : flexiv robot
"""

import math
import time

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R

from .Bestman_sim import Bestman_sim
from .Pose import Pose


class Bestman_sim_flexiv(Bestman_sim):
    """
    A class representing a simulation for the Bestman robot equipped with a flexiv arm.
    """

    def __init__(self, client, visualizer, cfg):
        """
        Initialize the Bestman_sim_flexiv with the given parameters.

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
        self.visualizer.set_object_color(self.base_id, "light_white")
        
        # gripper constraints
        c = p.createConstraint(self.arm_id, 10, self.arm_id, 11, p.JOINT_POINT2POINT, [0, 0, 0], [0, -0.014, 0.043], [0, -0.034, 0.021])
        p.changeConstraint(c, erp=0.1, maxForce=1000)
        
        c = p.createConstraint(self.arm_id, 12, self.arm_id, 13, p.JOINT_POINT2POINT, [0, 0, 0], [0, -0.014, 0.043], [0, -0.034, 0.021])
        p.changeConstraint(c, erp=0.1, maxForce=1000)
        
        p.setJointMotorControl2(self.arm_id, 10, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
        p.setJointMotorControl2(self.arm_id, 11, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
        p.setJointMotorControl2(self.arm_id, 12, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
        p.setJointMotorControl2(self.arm_id, 13, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
        
        
        c = p.createConstraint(self.arm_id, 8, self.arm_id, 14, p.JOINT_GEAR, [1, 0, 0], [0, 0, 0], [0, 0, 0])
        p.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=50)
        
        # gripper control
        p.setJointMotorControlMultiDofArray(self.arm_id, [8, 14], p.POSITION_CONTROL, [[q], [q]], forces=[[t], [t]])

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
