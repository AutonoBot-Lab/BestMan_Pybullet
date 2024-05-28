# !/usr/bin/env python
# -*- encoding: utf-8 -*-

'''
@file: Collision.py
@Description: collision detection
@Date: 2024/04/07 16:49:51
@Author: yk
@version: 1.0
'''

from .utils import *

class Collision:
    
    def __init__(self, robot, obstacles):
        
        self.robot = robot
        self.arm_id = robot.get_arm_id()
        self.joint_idx = robot.get_joint_idx()
        self.obstacles = obstacles
    
    def setup_collision_detection(self, self_collisions=True, allow_collision_links=[]):
        
        self.check_link_pairs = (
            get_self_link_pairs(self.arm_id, self.joint_idx)
            if self_collisions
            else []
        )
        
        moving_links = frozenset(
            [
                item
                for item in get_moving_links(self.arm_id, self.joint_idx)
                if not item in allow_collision_links
            ]
        )
        moving_bodies = [(self.arm_id, moving_links)]
        self.check_body_pairs = list(product(moving_bodies, self.obstacles))
    
    def is_state_valid(self, state):
        
        # check self-collision
        self.robot.sim_set_arm_to_joint_values(state)
        for link1, link2 in self.check_link_pairs:
            if pairwise_link_collision(
                self.arm_id, link1, self.arm_id, link2
            ):
                return False

        # check collision against environment
        for body1, body2 in self.check_body_pairs:
            if pairwise_collision(body1, body2):
                return False
        return True

