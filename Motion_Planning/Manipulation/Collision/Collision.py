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

    def __init__(self, robot, obstacles):
        """
        Initializes the Collision class.

        Args:
            robot (Robot): The robot object.
            obstacles (list): List of obstacles in the environment.
        """
        self.robot = robot
        self.arm_id = robot.get_arm_id()
        self.joint_idx = robot.get_joint_idx()
        self.obstacles = obstacles

    def setup_collision_detection(self, self_collisions=True, allow_collision_links=[]):
        """
        Sets up collision detection.

        Args:
            self_collisions (bool, optional): Whether to check for self-collisions. Defaults to True.
            allow_collision_links (list, optional): Links that are allowed to collide. Defaults to [].
        """
        all_joint_idx = self.robot.get_all_joint_idx()
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

        # check self-collision
        for link1, link2 in self.check_link_pairs:
            if pairwise_link_collision(self.arm_id, link1, self.arm_id, link2):
                return False

        # check collision against environment
        for body1, body2 in self.check_body_pairs:
            if pairwise_collision(body1, body2):
                return False

        return True
