# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : Bestman_sim_ur5e_vacuum_long.py
# @Time           : 2024-08-03 15:08:23
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : Ur5e robot
"""


import math

import pybullet as p

from .Bestman_sim import Bestman_sim
from .Pose import Pose


class Bestman_sim_ur5e_vacuum_long(Bestman_sim):
    """
    A class representing a simulation for the Bestman robot equipped with a UR5e arm and a vacuum gripper.
    """

    def __init__(self, client, visualizer, cfg):
        """
        Initialize the Bestman_sim_ur5e_vacuum_long with the given parameters.

        Args:
            client (int): The PyBullet client ID.
            visualizer (bool): Flag indicating whether visualization is enabled.
            cfg (dict): Configuration settings.
        """
        super().__init__(client, visualizer, cfg)

    # ----------------------------------------------------------------
    # Functions for gripper
    # ----------------------------------------------------------------
    
    def sim_create_fixed_constraint(self, object):
        """
        Activate or deactivate the gripper for a movable object.

        Args:
            object (str): The name or ID of the object related to gripper action.
            link_id (int): The ID of the link on the object to be grasped.
            value (int): 0 or 1, where 0 means deactivate (ungrasp) and 1 means activate (grasp).
        """
        object_id = self.client.resolve_object_id(object)
        
        # cretae constraint
        if self.constraint_id is None:
            cube_orn = p.getQuaternionFromEuler([0, math.pi, 0])
            if self.tcp_link != -1:
                self.constraint_id = p.createConstraint(
                    self.arm_id,
                    self.tcp_link,
                    object_id,
                    -1,
                    p.JOINT_FIXED,
                    [0, 0, 0],
                    [0, 0, 0],
                    [0, 0, 0],
                    childFrameOrientation=cube_orn,
                    physicsClientId=self.client_id,
                )
            else:
                self.gripper_id = p.createConstraint(
                    self.arm_id,
                    self.end_effector_index,
                    object_id,
                    -1,
                    p.JOINT_FIXED,
                    [0, 0, 0],
                    [0, 0, 0],
                    [0, 0, 0],
                    childFrameOrientation=cube_orn,
                    physicsClientId=self.client_id,
                )
            self.client.run(40)
            print(
                "[BestMan_Sim][Sucker] \033[34mInfo\033[0m: Sucker fixed constraint has been created!"
            )
            
    def sim_remove_fixed_constraint(self): 
        """remove fixed constraint
        """
        if self.constraint_id != None:
            p.removeConstraint(self.constraint_id, physicsClientId=self.client_id)
            self.client.run(40)
            self.constraint_id = None
            print(
                "[BestMan_Sim][Sucker] \033[34mInfo\033[0m: Sucker fixed constraint has been removed!"
            )

    def sim_create_movable_constraint(self, object, link_id):
        """create constraint between end effector and joint

        Args:
            object (str): The name or ID of the object related to gripper action.
            link_id (int): The ID of the link on the object to create constraint.
        """
        object_id = self.client.resolve_object_id(object)
        if self.constraint_id is None:
            link_state = p.getLinkState(object_id, link_id)
            vec_inv, quat_inv = p.invertTransform(link_state[0], link_state[1])
            current_pose = self.client.get_object_link_pose(self.arm_id, self.tcp_link)
            transform_start_to_link = p.multiplyTransforms(
                vec_inv,
                quat_inv,
                current_pose.position,
                p.getQuaternionFromEuler(current_pose.orientation),
            )
            self.constraint_id = p.createConstraint(
                parentBodyUniqueId=object_id,
                parentLinkIndex=link_id,
                childBodyUniqueId=self.arm_id,
                childLinkIndex=self.tcp_link,
                jointType=p.JOINT_POINT2POINT,
                jointAxis=[0, 0, 0],
                parentFramePosition=transform_start_to_link[0],
                parentFrameOrientation=transform_start_to_link[1],
                childFramePosition=[0, 0, 0],
            )
            p.changeConstraint(self.constraint_id, maxForce=2000)
            print(
                "[BestMan_Sim][Sucker] \033[34mInfo\033[0m: Sucker movable constraint has been created!"
            )
            
    def sim_remove_movable_constraint(self):
        """remove constraint between end effector and joint
        """
        if self.constraint_id is not None:
                p.removeConstraint(self.constraint_id, physicsClientId=self.client_id)
                self.client.run(40)
                self.constraint_id = None
                print(
                    "[BestMan_Sim][Sucker] \033[34mInfo\033[0m: Sucker movable constraint has been removed!"
                )

    # ----------------------------------------------------------------
    # Functions for pick and place actions
    # ----------------------------------------------------------------

    def pick(self, object):
        """
        Pick up the specified object.

        Args:
            object (str): The name or ID of the object to be picked up.
        """
        init_pose = self.sim_get_current_end_effector_pose()
        object_id = self.client.resolve_object_id(object)
        min_x, min_y, _, max_x, max_y, max_z = self.client.get_bounding_box(object_id)
        goal_pose = Pose(
            [(min_x + max_x) / 2, (min_y + max_y) / 2, max_z + self.tcp_height],
            [0.0, math.pi / 2.0, 0.0],
        )
        self.sim_move_end_effector_to_goal_pose(goal_pose)
        self.sim_create_fixed_constraint(object_id)
        self.sim_move_end_effector_to_goal_pose(init_pose)

    def place(self, goal_pose):
        """
        Place an object at the specified goal pose.

        Args:
            goal_pose (Pose): The pose where the object will be placed.
        """
        init_pose = self.sim_get_current_end_effector_pose()
        self.sim_move_end_effector_to_goal_pose(goal_pose)
        self.sim_remove_fixed_constraint()
        self.sim_move_end_effector_to_goal_pose(init_pose)

    def pick_place(self, object, goal_pose):
        """
        Perform a pick and place operation.

        Args:
            object (str): The name or ID of the object to be picked up.
            goal_pose (Pose): The pose where the object will be placed.
        """
        self.pick(object)
        self.place(goal_pose)
