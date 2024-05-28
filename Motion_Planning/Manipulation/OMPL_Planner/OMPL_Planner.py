"""
@Description :   A few functions for planning
@Author      :   Yan Ding 
@Time        :   2023/08/30 22:52:39
"""


import math
import numpy as np
import pybullet as p
from ompl import base as ob
from ompl import geometric as og
from .utils import *
from ..Collision import Collision

INTERPOLATE_NUM = 500
DEFAULT_PLANNING_TIME = 5.0     # planning time threshold

class OMPL_Planner:
    def __init__(
        self,
        robot,
        Planner_cfg
    ):
        """
        Initialize the OMPL.

        Args:
           arm_id: ID of the robot to use.
           joint_idx: Index of the joints to use in planning.
           obstacles: List of obstacles to consider in the motion planning.
           planner: The name of the motion planner algorithm to use.
           threshold: The threshold value ONLY for repalnning.
        """
        
        # arm
        self.robot = robot
        self.arm_id = robot.get_arm_id()
        self.joint_idx = robot.get_joint_idx()
        self.tcp_link = robot.get_tcp_link()
        self.DOF = robot.get_DOF()
        
        # client
        self.client = robot.client
        self.client_id = self.client.get_client_id()
        
        # obstacles
        self.obstacles = []
        self.collision = Collision(robot, self.obstacles)
        self.set_obstacles()
        
        # preparation for Manipution planning
        self.space = ob.RealVectorStateSpace(self.DOF)
        bounds = ob.RealVectorBounds(self.DOF)
        joint_bounds = self.robot.get_joint_bounds()
        for i, bound in enumerate(joint_bounds):
            bounds.setLow(i, bound[0])
            bounds.setHigh(i, bound[1])
        self.space.setBounds(bounds)
        
        self.ss = og.SimpleSetup(self.space)
        self.ss.setStateValidityChecker(ob.StateValidityCheckerFn(self.collision.is_state_valid))
        self.si = self.ss.getSpaceInformation()
        
        # planner
        self.max_attempts = Planner_cfg.max_attempts
        self.threshold = Planner_cfg.threshold
        self.set_planner(Planner_cfg.planner)

    # ----------------------------------------------------------------
    # set planner / goal
    # ----------------------------------------------------------------
    
    def set_planner(self, planner_name):
        """
        Set Planner to OMPL.

        Args:
            planner_name: The planner to set to OMPL.
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
            print("{} not recognized, please add it first".format(planner_name))
            return

        self.ss.setPlanner(self.planner)

    def set_target(self, target_id):
        """
        Set the target to be used for the manipulation task.

        Args:
            target_id: id of the target object.
        """
              
        try:
            _ = p.getBodyInfo(target_id)
        except Exception as e:
            print(f"Target object not set: {e}")
        
        # get target object bounds
        min_x, min_y, _, max_x, max_y, max_z = self.client.get_bounding_box(target_id)
        
        # set target object postion
        self.target_pos = (
            (min_x + max_x) / 2,
            (min_y + max_y) / 2,
            max_z
        )
        target_orientation = [0.0, math.pi / 2.0, 0.0]      # vertical
        
        # set target grasp angle
        self.goal = self.robot.cartesian_to_joints(position=[self.target_pos[0], self.target_pos[1], max_z + self.robot.tcp_height + 0.01], 
                                                   orientation=target_orientation)
    
    # ----------------------------------------------------------------
    # obstacles
    # ----------------------------------------------------------------
    
    def set_obstacles(self):
        """
        Add obstacles to the scene.
        This is done by iterating over all items in the scene and adding them to the list of obstacles.
        """

        num_items = p.getNumBodies()

        # add object in scene, skip arm
        for item_id in range(num_items):
            if item_id == self.arm_id:
                continue
            self.obstacles.append(item_id)
        
        # init collision info
        self.collision.setup_collision_detection()
    
    def get_obstacles_info(self):
        """
        Check obstacles in the scene and print them to the console.
        This is a debugging function.
        """
        
        if self.obstacles == []:
            print("Obstacle list is empty")
        else:
            # print the IDs and names of all obstacles in the scene
            for obstacle_id in self.obstacles:
                item_info = p.getBodyInfo(obstacle_id)
                item_name = item_info[1].decode("utf-8")
                print(f"Obstacle Name: {item_name}, ID: {obstacle_id}")
    
    # ----------------------------------------------------------------
    # plan / execute
    # ----------------------------------------------------------------
    
    def compute_distance(self, end_effector_link_index):
        """
        Compute the distance between the end effector and the object.

        Args:
                end_effector_link_index: index of the end effector link.

        Returns:
                distance: distance between the end - effector and the object.
        """
        
        end_effector_pose = p.getLinkState(self.arm_id, end_effector_link_index)

        # Compute the distance between the end-effector and the object
        distance = np.linalg.norm(
            np.array(end_effector_pose[0]) - np.array(self.target_pos)
        )

        print(f'end_effector_pose:{end_effector_pose[0]}, target_pos:{self.target_pos}')
        
        return distance

    def plan(self, start, goal, allowed_time=DEFAULT_PLANNING_TIME):
        """
        Plan grasp from start to goal.
        This is a wrapper around OMPL grasp planning algorithm.

        Args:
            start: state to start planning from.
            goal: state to go to after planning has been completed.

        Returns:
            solved: Flag whether a feasible solution has been found
            path: a list of robot state.
        """
        
        # set arm joint angle to start state
        self.robot.set_arm_to_joint_values(start)
        
        print("start_planning")

        # set the start and goal states
        s = ob.State(self.space)
        g = ob.State(self.space)
        for i in range(len(start)):
            s[i] = start[i]
            g[i] = goal[i]
        self.ss.setStartAndGoalStates(s, g)

        # attempt to solve the problem within allowed planning time
        solved = self.ss.solve(allowed_time)
        path = []
        if solved:
            print(
                "Found solution: interpolating into {} segments".format(INTERPOLATE_NUM)
            )
            # print the path to screen
            sol_path_geometric = self.ss.getSolutionPath()
            sol_path_geometric.interpolate(INTERPOLATE_NUM)
            sol_path_states = sol_path_geometric.getStates()
            path = [state_to_list(state, self.DOF) for state in sol_path_states]
        else:
            print("No solution found")
        
        return solved, path

    def execute(self, path, dynamics=False):
        """
        arm execute a given path
        """
        
        for q in path:
            if dynamics:
                for i in self.joint_idx:
                    p.setJointMotorControl2(
                        self.arm_id, i, p.POSITION_CONTROL, q[i]
                    )
            else:
                self.robot.set_arm_to_joint_values(q)
            
            p.stepSimulation()
    
    def plan_execute(self):
        """
        Reach an object from start to goal
        """
        
        # start arm joint angles
        start = self.robot.get_current_joint_values()
        
        attempts = 0
        while attempts < self.max_attempts:
            
            attempts += 1
            
            # Manipution plan
            res, path = self.plan(start, self.goal)

            # Execute the path and attach the object to the robot
            if res:
                
                self.execute(path, True)
                # self.execute(path)
                
                # Check if the robot is close to the object
                if self.tcp_link != -1:
                    distance = self.compute_distance(self.tcp_link)
                else:
                    distance = self.compute_distance(self.robot.end_effector_link_index)
                    print("Attention, the distance is computed without tcp link")
                
                print("Distance to goal:{}".format(distance))
                
                # This method grasses the robot if the distance is below threshold.
                if distance <= self.threshold:
                    print(
                        "After {} trials, successfully grasped (error:{}).".format(
                            attempts, distance
                        )
                    )
                    
                    break

        if attempts >= self.max_attempts:
            print(
                "Could not reach target position without collision after {} attempts".format(
                    self.max_attempts
                )
            )  