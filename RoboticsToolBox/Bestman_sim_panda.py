# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : Bestman_sim_panda.py
# @Time           : 2024-08-03 15:08:13
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : Panda robot
"""

import math
import pybullet as p

from .Bestman_sim import Bestman_sim
from .Pose import Pose

class Bestman_sim_panda(Bestman_sim):
    """
    A class representing a simulation for the Bestman robot equipped with a Panda arm.
    """
    
    def __init__(self, client, visualizer,  cfg):
        """
        Initialize the Bestman_sim_panda with the given parameters.

        Args:
            client (int): The PyBullet client ID.
            visualizer (bool): Flag indicating whether visualization is enabled.
            cfg (dict): Configuration settings.
        """
        
        # Init parent class: BestMan_sim
        super().__init__(client, visualizer,  cfg)
        
        # Create a gear constraint to keep the fingers symmetrically centered
        c = p.createConstraint(
            self.arm_id,
            9,
            self.arm_id,
            10,
            jointType=p.JOINT_GEAR,
            jointAxis=[1, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0]
        )
        
        # Constraint parameters
        p.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=50)

    
    # ----------------------------------------------------------------
    # functions for gripper
    # ----------------------------------------------------------------
    
    def sim_active_gripper_fixed(self, value):
        """
        Activate or deactivate the gripper.

        Args:
            value (int): 0 or 1, where 0 means deactivate (ungrasp) and 1 means activate (grasp).
        """

        if value == 1:
            for i in [9, 10]:
                p.setJointMotorControl2(self.arm_id, i, p.POSITION_CONTROL, 0.04, force=10)
                print("[BestMan_Sim][Gripper] \033[34mInfo\033[0m: Gripper has been deactivated!")
        elif value == 0:
            for i in [9, 10]:
                p.setJointMotorControl2(self.arm_id, i, p.POSITION_CONTROL, 0.001, force=10)
                print("[BestMan_Sim][Gripper] \033[34mInfo\033[0m: Gripper has been activated!")
        else:
            raise(ValueError("[BestMan_Sim][Gripper] \033[31merror\033[0m: gripper value must be 0 / 1 !"))
        
        self.client.run(30)
        
    def sim_active_gripper_movable(self, object, link_id, value):
        """
        Activate or deactivate the gripper for a movable object.

        Args:
            object (str): The name or ID of the object related to gripper action.
            link_id (int): The ID of the link on the object to be grasped.
            value (int): 0 or 1, where 0 means deactivate (ungrasp) and 1 means activate (grasp).
        """

        object_id = self.client.resolve_object_id(object)
        
        # ungrasp
        if value == 0 and self.gripper_id != None:
            p.removeConstraint(self.gripper_id, physicsClientId=self.client_id)
            self.gripper_id = None
            print("[BestMan_Sim][Gripper] Gripper has been deactivated!")
        
        # grasp
        if value == 1 and self.gripper_id == None:
            link_state = p.getLinkState(object_id, link_id)
            vec_inv, quat_inv = p.invertTransform(link_state[0], link_state[1])
            if self.tcp_link != -1:
                current_pose = self.client.get_object_link_pose(self.arm_id, self.tcp_link)
                transform_start_to_link = p.multiplyTransforms(vec_inv, quat_inv, current_pose.position, p.getQuaternionFromEuler(current_pose.orientation))
                self.gripper_id = p.createConstraint(
                        parentBodyUniqueId=object_id,
                        parentLinkIndex=link_id,
                        childBodyUniqueId=self.arm_id,
                        childLinkIndex=self.tcp_link,
                        jointType=p.JOINT_POINT2POINT,
                        jointAxis=[0, 0, 0],
                        parentFramePosition=transform_start_to_link[0],
                        parentFrameOrientation=transform_start_to_link[1],
                        childFramePosition=[0, 0, 0]
                    )
            else:
                current_pose = self.client.get_object_link_pose(self.arm_id, self.end_effector_index)
                transform_start_to_link = p.multiplyTransforms(vec_inv, quat_inv, current_pose.position, p.getQuaternionFromEuler(current_pose.orientation))
                self.gripper_id = p.createConstraint(
                        parentBodyUniqueId=object_id,
                        parentLinkIndex=link_id,
                        childBodyUniqueId=self.arm_id,
                        childLinkIndex=self.tcp_link,
                        jointType=p.JOINT_POINT2POINT,
                        jointAxis=[0, 0, 0],
                        parentFramePosition=transform_start_to_link[0],
                        parentFrameOrientation=transform_start_to_link[1],
                        childFramePosition=[0, 0, 0]
                    )
            p.changeConstraint(self.gripper_id, maxForce=2000)
    
    
    # ----------------------------------------------------------------
    # functions for pick and place
    # ----------------------------------------------------------------
    
    def pick(self, object):
        """
        Pick up the specified object without collision.

        Args:
            object (str): The name or ID of the object to be picked up.
        """
        object_id = self.client.resolve_object_id(object)
        position, _ = p.getBasePositionAndOrientation(object_id)
        goal_pose = Pose([position[0], position[1], position[2]+0.015], [0, math.pi, 0])
        self.move_end_effector_to_goal_pose(goal_pose, 50)
        goal_pose = Pose([position[0], position[1], position[2]-0.005], [0, math.pi, 0])
        self.move_end_effector_to_goal_pose(goal_pose, 50)
        self.sim_active_gripper(0)
    
    def place(self, goal_pose):
        """
        Place an object at the specified goal pose without collision.

        Args:
            goal_pose (Pose): The pose where the object will be placed.
        """
        init_pose = self.get_current_end_effector_pose()
        init_pos, _ = init_pose.position, init_pose.orientation
        goal_pos, goal_orn = goal_pose.position, goal_pose.orientation
        tmp_pose = Pose([init_pos[0], init_pos[1], goal_pos[2]], goal_orn)
        self.move_end_effector_to_goal_pose(tmp_pose, 50)
        self.move_end_effector_to_goal_pose(goal_pose, 50)
        self.sim_active_gripper(1)
    
    def pick_place(self, object, goal_pose):
        """
        Perform a pick and place operation.

        Args:
            object (str): The name or ID of the object to be picked up.
            goal_pose (Pose): The pose where the object will be placed.
        """
        self.pick(object)
        self.place(goal_pose)