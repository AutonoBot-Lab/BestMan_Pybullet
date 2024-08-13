# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : Collision.py
# @Time           : 2024-08-03 15:06:10
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : Collision detection module
"""

from .utils import *


class Collision:
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
        self.joint_idx = robot.sim_get_arm_joint_idx()
        self.obstacles = []
        self.set_obstacles()

    def set_obstacles(self):
        """
        Add obstacles to the scene.
        """
        num_items = p.getNumBodies()

        # add object in scene, skip arm
        for item_id in range(num_items):
            if item_id == self.arm_id:
                continue
            self.obstacles.append(item_id)
            
        # update collision detect pair
        self.setup_collision_detection()

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
            self.setup_collision_detection()
            
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
                self.setup_collision_detection(self.obstacles)
            
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
                
    def setup_collision_detection(self, self_collisions=True, allow_collision_links=[]):
        """
        Sets up collision detection.

        Args:
            self_collisions (bool, optional): Whether to check for self-collisions. Defaults to True.
            allow_collision_links (list, optional): Links that are allowed to collide. Defaults to [].
        """
        all_joint_idx = self.robot.sim_get_arm_all_joint_idx()
        self.check_link_pairs = (
            get_self_link_pairs(self.arm_id, all_joint_idx) if self_collisions else []
        )

        moving_links = frozenset(
            [
                item
                for item in get_moving_links(self.arm_id, all_joint_idx)
                if not item in allow_collision_links
            ]
        )
        moving_bodies = [(self.arm_id, moving_links)]
        self.check_body_pairs = list(product(moving_bodies, self.obstacles))

    def is_state_valid(self, state):
        """
        Checks if a given state is valid (i.e., collision-free).

        Args:
            state (list): The state to check.

        Returns:
            bool: True if the state is valid, False otherwise.
        """
        self.robot.sim_set_arm_to_joint_values(state)

        # # check self-collision
        # for link1, link2 in self.check_link_pairs:
        #     if pairwise_link_collision(self.arm_id, link1, self.arm_id, link2):
        #         return False
        
        # check collision against environment
        for body1, body2 in self.check_body_pairs:
            if pairwise_collision(body1, body2):
                return False

        return True
