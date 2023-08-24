import cv2
import pybullet as p
import pybullet_data
import math
import time
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import random
import sys
import os
from matplotlib.colors import LinearSegmentedColormap


class UR5_2F85:
    def __init__(self, client_id, robot_id):
        """
        Initializes the UR5 with Robotiq 2f85 gripper.

        Args:
                client_id: Client ID of pybullet.
                robot_id: The URDF object features a Robotiq 2f85 gripper.
        """
        self._cid = client_id
        self.Rob2f85 = Robotiq2F85(self._cid, robotUID=robot_id)
        self.robot_id = self.Rob2f85.getUID()
        self.end_effector_link_index = self.get_tool_center_point()
        self.build_joint_index_from_name_dict()

    def set_gripper_max_force(self, max_force):
        """
        Set gripper max force.

        Args:
                max_force: The max force of gripper motor control.
        """
        self.Rob2f85.setMaxForce(max_force)

    def init_joints(self, init_pose):
        """
        Initialize joints.

        Args:
                init_pose: a list of two representing the gripper closing.
        """
        r2f85 = init_pose
        self.Rob2f85.initJoints(r2f85)

    def close_gripper(self):
        """
        Close gripper and go to 80%.
        """
        self.set_gripper_goal(80, 80)

    def open_gripper(self):
        """
        Opens gripper.
        """
        self.set_gripper_goal(0, 0)

    def get_tool_center_point(self):
        """
        Get the index of tcp of gripper.


        Returns:
                end_effector_link_index: the index of the end effector joint.
        """
        end_effector_link_index = self.get_joint_index_from_name("tool_center_point")
        return end_effector_link_index

    def get_end_effector_info(self):
        """
        Get information about the end effector.

        Returns:
               end_effector_position: the end effector position in world coordinates.
           end_effector_orientation: the end effector orientation in world coordinates.
        """
        joint_info = p.getJointInfo(self.robot_id, self.end_effector_link_index)
        end_effector_name = joint_info[1].decode("utf-8")
        end_effector_info = p.getLinkState(
            bodyUniqueId=self.robot_id, linkIndex=self.end_effector_link_index
        )
        end_effector_position = end_effector_info[0]
        end_effector_orientation = end_effector_info[1]
        return end_effector_position, end_effector_orientation

    def build_joint_index_from_name_dict(self):
        """
        Build joint index from name dictionary.
        This is used to find the names of joints.
        """
        self.joints_names_dict = {}
        num_joints = p.getNumJoints(self.robot_id, self._cid)
        # Get the joint information from the robot.
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i, self._cid)
            srt_joint_name = joint_info[1].decode("UTF-8")
            self.joints_names_dict[srt_joint_name] = joint_info[0]

    def get_joint_index_from_name(self, joint_name):
        """
        Get joint index from name.

        Args:
                joint_name: name of the joint to find.

        Returns:
                joint_id: index of the joint to find.
        """
        try:
            # build joint index from name dictionary
            if len(self.joints_names_dict) == 0:
                self.build_joint_index_from_name_dict()
        except AttributeError:
            self.build_joint_index_from_name_dict()

        try:
            joint_id = self.joints_names_dict.get(joint_name)
        except Exception as e:
            print(f"Cannot find a joint named: {e}")
        return joint_id

    def set_gripper_goal(self, left_finger_goal, right_finger_goal):
        """
        Set gripper goal in degrees.
        A percentage that specify how close each finger is (100 = Fully closed, 0 = Fully open).

        Args:
                left_finger_goal: the amount of left finger closing.
                right_finger_goal: the amount of right finger closing.
        """
        self.Rob2f85.setGoal(left_finger_goal, right_finger_goal)
