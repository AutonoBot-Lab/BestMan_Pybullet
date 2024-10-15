# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : OMPL_Planner.py
# @Time           : 2024-08-03 15:06:26
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : OMPL planner
"""

import math

import pybullet as p
from ompl import base as ob
from ompl import geometric as og

from Robotics_API import Pose

from ..Collision_Detection import Basic_Collision


class OMPL_Planner:
    """Class for OMPL planning."""

    def __init__(self, robot, Planner_cfg):
        """
        Initialize the OMPL planner.

        Args:
            robot (Robot): The robot object.
            Planner_cfg (Config): The configuration for the planner.
        """

        # arm info
        self.robot = robot
        self.arm_id = robot.sim_get_arm_id()
        self.joint_idx = robot.sim_get_arm_joint_idx()
        self.tcp_link = robot.sim_get_tcp_link()
        self.DOF = robot.sim_get_DOF()

        # client info
        self.client = robot.client
        self.client_id = self.client.get_client_id()

        # obstacles
        self.target_id = None
        self.collision = Basic_Collision(robot)
        
        # preparation for OMPL planning
        self.space = ob.RealVectorStateSpace(self.DOF)  # construct the state space
        bounds = ob.RealVectorBounds(self.DOF)  # creating Boundary
        joint_bounds = self.robot.sim_get_joint_bounds()  # get joint boundaries
        for i, bound in enumerate(joint_bounds):
            bounds.setLow(i, bound[0])
            bounds.setHigh(i, bound[1])
        self.space.setBounds(bounds)  # set bounds

        self.ss = og.SimpleSetup(self.space)
        self.ss.setStateValidityChecker(
            ob.StateValidityCheckerFn(self.collision.is_state_valid)
        )

        self.si = self.ss.getSpaceInformation()

        # planner cfgs
        self.set_planner(Planner_cfg.planner)
        self.planning_time = Planner_cfg.planning_time
        self.interpolate_num = Planner_cfg.interpolate_num

    # ----------------------------------------------------------------
    # set planner / goal
    # ----------------------------------------------------------------

    def set_planner(self, planner_name):
        """
        Set planner for OMPL.

        Args:
            planner_name (str): The name of the planner to use.
        """

        if planner_name == "PRM":
            self.planner = og.PRM(self.ss.getSpaceInformation())
        elif planner_name == "RRT":
            self.planner = og.RRT(self.ss.getSpaceInformation())
        elif planner_name == "RRTConnect":
            self.planner = og.RRTConnect(self.ss.getSpaceInformation())
        elif planner_name == "RRTstar":
            self.planner = og.RRTstar(self.ss.getSpaceInformation())
        elif planner_name == "EST":
            self.planner = og.EST(self.ss.getSpaceInformation())
        elif planner_name == "FMT":
            self.planner = og.FMT(self.ss.getSpaceInformation())
        elif planner_name == "BITstar":
            self.planner = og.BITstar(self.ss.getSpaceInformation())
        else:
            print(
                "[OMPL Planner] \033[33mwarning\033[0m: {} not recognized, please add it first".format(
                    planner_name
                )
            )
            return

        self.ss.setPlanner(self.planner)

    def set_target(self, target):
        """
        Set the target object for the manipulation task.

        Args:
            target (Union[str, int]): The id or name of the target object.

        Returns:
            list: The goal state in joint space.
        """
        self.target_id = self.client.resolve_object_id(target)

        # get target object bounds
        min_x, min_y, _, max_x, max_y, max_z = self.client.get_bounding_box(
            self.target_id
        )

        # set target object Pose
        goal_pose = Pose(
            [
                (min_x + max_x) / 2,
                (min_y + max_y) / 2,
                max_z + self.robot.tcp_height + 0.05,
            ],
            [0.0, math.pi / 2.0, 0.0],
        )

        # get goal angle
        goal = self.robot.sim_cartesian_to_joints(goal_pose)

        return goal

    def set_target_pose(self, target_pose):
        """
        Set the target pose for the manipulation task.

        Args:        # init collision info
        self.collision.setup_collision_detection()
            target_pose (Pose): The pose of the target object.

        Returns:
            list: The goal state in joint space.
        """
        goal = self.robot.sim_cartesian_to_joints(target_pose)
        return goal

    # ----------------------------------------------------------------
    # obstacles
    # ----------------------------------------------------------------

    def set_obstacles(self):
        """
        Add obstacles to the scene.
        """
        self.collision.set_obstacles()

    def add_obstacle(self, obstacle):
        """
        Add obstacle to the list of obstacles.
        This is useful for adding a specific obstacle to the list.

        Args:
            obstacle_id: id of the obstacle to add.
        """
        self.collision.add_obstacle(obstacle)

    def remove_obstacle(self, obstacle):
        """
        Remove obstacle from the list of obstacles.
        This is useful for removing a specific obstacle from the list.

        Args:
            obstacle_id: id of the obstacle to remove.
        """
        self.collision.remove_obstacle(obstacle)

    def get_obstacles_info(self):
        """
        Check obstacles in the scene and print them to the console.
        """
        self.collision.get_obstacles_info()

    # ----------------------------------------------------------------
    # functions for plan
    # ----------------------------------------------------------------

    def plan(self, start, goal):
        """
        Plan grasp from start to goal using OMPL.

        Args:
            start (list): The start state in joint space.
            goal (list): The goal state in joint space.

        Returns:
            list: The planned path as a list of states.
        """

        print("[OMPL Planner] \033[34mInfo\033[0m: Start planning...")

        # remove target object
        self.remove_obstacle(self.target_id)

        # set the start and goal states
        s = ob.State(self.space)
        g = ob.State(self.space)
        for i in range(len(start)):
            s[i] = start[i]
            g[i] = goal[i]
        self.ss.setStartAndGoalStates(s, g)

        # attempt to solve the problem within allowed planning time
        try:
            self.ss.solve(self.planning_time)
            sol_path_geometric = self.ss.getSolutionPath()
            sol_path_geometric.interpolate(
                self.interpolate_num
            )  # Linear interpolation, Generate more intermediate states to make the path smoother and more refined
            sol_path_states = sol_path_geometric.getStates()
            path = [self.state_to_list(state) for state in sol_path_states]
            self.add_obstacle(self.target_id)
            print("[OMPL Planner] \033[34mInfo\033[0m: End planning!")
            return path
        except RuntimeError as _:
            print("[OMPL Planner] \033[31merror\033[0m: No solution found!")
    
    # ----------------------------------------------------------------
    # Utils
    # ----------------------------------------------------------------

    def state_to_list(self, state):
        """
        Convert state to list.

        Args:
            state (ob.State): The state to convert.

        Returns:
            list: The state as a list of values.
        """
        return [state[i] for i in range(self.DOF)]
