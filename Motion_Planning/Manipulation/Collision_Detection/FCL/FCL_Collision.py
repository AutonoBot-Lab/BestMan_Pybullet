# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : Collision.py
# @Time           : 2024-08-03 15:06:10
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : Collision detection module
"""

import pybullet as p

from ..utils import *
from .utils import *


class FCL_Collision:
    """A class for handling collision detection."""

    def __init__(self, robot):
        """
        Initializes the Collision class.

        Args:
            robot (Robot): The robot object.
        """
        self.robot = robot
        self.client = robot.client
        self.arm_id = robot.sim_get_arm_id()
        self.base_id = robot.sim_get_base_id()
        self.joint_idx = robot.sim_get_arm_joint_idx()
        self.obstacles = []
        self.setup()

    def add_obstacle(self, obstacle):
        """
        Add obstacle to the list of obstacles.
        This is useful for adding a specific obstacle to the list.

        Args:
            obstacle_id: id of the obstacle to add.
        """
        if obstacle is not None:
            obstacle_id = self.client.resolve_object_id(obstacle)
            self.obstacles.append(obstacle_id)
            self.setup()

    def remove_obstacle(self, obstacle):
        """
        Remove obstacle from the list of obstacles.
        This is useful for removing a specific obstacle from the list.

        Args:
            obstacle_id: id of the obstacle to remove.
        """
        if obstacle is not None:
            obstacle_id = self.client.resolve_object_id(obstacle)
            if obstacle_id in self.obstacles:
                self.obstacles.remove(obstacle_id)
                self.setup(self.obstacles)

    def get_obstacles_info(self):
        """
        Check obstacles in the scene and print them to the console.
        """

        if self.obstacles == []:
            print("[OMPL Planner] \033[33mwarning\033[0m: Obstacle list is empty")
        else:
            for obstacle_id in self.obstacles:
                item_info = p.getBodyInfo(obstacle_id)
                item_name = item_info[1].decode("utf-8")
                print(
                    f"[OMPL Planner] \033[34mInfo\033[0m: Obstacle Name: {item_name}, ID: {obstacle_id}"
                )

    def setup(self, self_collisions=True, allow_collision_links=[]):
        """
        Sets up collision detection.

        Args:
            self_collisions (bool, optional): Whether to check for self-collisions. Defaults to True.
            allow_collision_links (list, optional): Links that are allowed to collide. Defaults to [].
        """
        # set arm link pairs
        self.arm_link_pairs = (
            get_arm_link_pairs(self.arm_id, self.joint_idx) if self_collisions else []
        )

        # set arm obstacle pairs
        self.obstacles = list(range(p.getNumBodies()))
        self.obstacles.remove(self.arm_id)
        self.obstacles.remove(self.base_id)
        self.arm_obstacle_pairs = list(product([self.arm_id], self.obstacles))
        self.link_models = {}
        for body_id in p.getNumBodies():
            for link_id in get_all_links(body_id):
                self.link_models[f"{body_id}{link_id}"] = create_fcl_bvh_from_bullet(
                    body_id, link_id
                )

    def is_state_valid(self, state):
        """
        Checks if a given state is valid (i.e., collision-free).

        Args:
            state (list): The state to check.

        Returns:
            bool: True if the state is valid, False otherwise.
        """
        # check arm self-collision
        if self.check_arm_self_collision():
            return False

        # check arm collision against obstacle
        if self.check_arm_obstacle_collision():
            return False

        return True

    def check_arm_self_collision(self):
        """
        Checks if two specific links from two different bodies are within a specified distance.
        Args:
            body1: ID of the first body.
            link1: Link index of the first body.
            body2: ID of the second body.
            link2: Link index of the second body (default is BASE_LINK).
            max_distance: Maximum allowed distance to consider collision (default is MAX_DISTANCE).
        Returns:
            True if the links are in collision or within the distance threshold, otherwise False.
        """
        link1 = link_models[f"{body1}{link1}"]
        link_state1 = p.getLinkState(body1, link1)
        transform1 = fcl.Transform(link_state1[4], link_state1[5])
        link1.setTransform(transform1)

        link2 = link_models[f"{body2}{link2}"]
        link_state2 = p.getLinkState(body2, link2)
        transform2 = fcl.Transform(link_state2[4], link_state2[5])
        link2.setTransform(transform2)

        request = fcl.CollisionRequest()
        result = fcl.CollisionResult()

        collision = fcl.collide(link1, link2, request, result)

        return collision

    def check_arm_obstacle_collision(self, max_distance=MAX_DISTANCE):
        """
        Checks if two bodies are in collision or within a certain distance.
        Args:
            body1: ID of the first body.
            body2: ID of the second body.
            max_distance: Maximum allowed distance to consider collision.
        Returns:
            True if the bodies are in collision or within the distance threshold, otherwise False.
        """
