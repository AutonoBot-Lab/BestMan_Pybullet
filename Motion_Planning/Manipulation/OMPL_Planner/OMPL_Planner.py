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

from RoboticsToolBox import Pose
from ..Collision import Collision

# INTERPOLATE_NUM = 500
INTERPOLATE_NUM = 20
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
        self.space = ob.RealVectorStateSpace(self.DOF)      # Creating a state space
        bounds = ob.RealVectorBounds(self.DOF)              # Creating Boundary
        joint_bounds = self.robot.get_joint_bounds()        # Get joint boundaries
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
    
    def set_target(self, target):
        """
        Set the target to be used for the manipulation task.

        Args:
            target_id: id of the target object.
        """
              
        if isinstance(target, str):
            if hasattr(self.client, target):
                target_id = getattr(self.client, target)
            else:
                raise AttributeError(f"scene has not {object} object!")
        else:
            target_id = target
        
        # get target object bounds
        min_x, min_y, _, max_x, max_y, max_z = self.client.get_bounding_box(target_id)
        
        # set target object Pose
        goal_pose = Pose([(min_x + max_x) / 2, (min_y + max_y) / 2, max_z + self.robot.tcp_height + 0.02], [0.0, math.pi / 2.0, 0.0])
        
        # get goal angle
        goal = self.robot.cartesian_to_joints(goal_pose)
        
        return goal
    
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
    # functions for plan 
    # ----------------------------------------------------------------

    def plan(self, start, goal, allowed_time=DEFAULT_PLANNING_TIME):
        """Plan grasp from start to goal.
        This is a wrapper around OMPL grasp planning algorithm.

        Args:
            start: state to start planning from.
            goal: state to go to after planning has been completed.

        Returns:
            path: a list of robot state.
        """
        
        print("\n" + "-" * 20 + "\n" + "Start planning"+ "\n" + "-" * 20 + "\n")

        # set the start and goal states
        s = ob.State(self.space)
        g = ob.State(self.space)
        for i in range(len(start)):
            s[i] = start[i]
            g[i] = goal[i]
        self.ss.setStartAndGoalStates(s, g)

        # attempt to solve the problem within allowed planning time
        solved = self.ss.solve(allowed_time)
        if solved:
            sol_path_geometric = self.ss.getSolutionPath()
            sol_path_geometric.interpolate(INTERPOLATE_NUM)     # Generate more intermediate states to make the path smoother and more refined
            sol_path_states = sol_path_geometric.getStates()
            path = [self.state_to_list(state) for state in sol_path_states]
            print("\n" + "-" * 20 + "\n" + "End planning"+ "\n" + "-" * 20 + "\n")
            return path
        else:
            raise RuntimeError("No solution found!")
    
    # ----------------------------------------------------------------
    # plan / execute
    # ----------------------------------------------------------------
    
    def state_to_list(self, state):
        return [state[i] for i in range(self.DOF)]