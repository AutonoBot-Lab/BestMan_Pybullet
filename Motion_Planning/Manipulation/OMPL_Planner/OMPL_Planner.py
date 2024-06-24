"""
@Description :   A few functions for planning
@Author      :   Yan Ding 
@Time        :   2023/08/30 22:52:39
"""


import math
import pybullet as p
from ompl import base as ob
from ompl import geometric as og

from RoboticsToolBox import Pose
from ..Collision import Collision


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
        
        # arm info
        self.robot = robot
        self.arm_id = robot.get_arm_id()
        self.joint_idx = robot.get_joint_idx()
        self.tcp_link = robot.get_tcp_link()
        self.DOF = robot.get_DOF()
        
        # client info
        self.client = robot.client
        self.client_id = self.client.get_client_id()
        
        # obstacles
        self.obstacles = []
        self.collision = Collision(robot, self.obstacles)
        self.set_obstacles()
        
        # preparation for OMPL planning
        self.space = ob.RealVectorStateSpace(self.DOF)      # construct the state space
        bounds = ob.RealVectorBounds(self.DOF)              # creating Boundary
        joint_bounds = self.robot.get_joint_bounds()        # get joint boundaries
        for i, bound in enumerate(joint_bounds):
            bounds.setLow(i, bound[0])
            bounds.setHigh(i, bound[1])
        self.space.setBounds(bounds)                        # set bounds
        
        self.ss = og.SimpleSetup(self.space)
        self.ss.setStateValidityChecker(ob.StateValidityCheckerFn(self.collision.is_state_valid))
        
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
            target: id / name of the target object.
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
        goal_pose = Pose([(min_x + max_x) / 2, (min_y + max_y) / 2, max_z + self.robot.tcp_height + 0.05], [0.0, math.pi / 2.0, 0.0])
        
        # get goal angle
        goal = self.robot.cartesian_to_joints(goal_pose)
        
        return goal
    
    def set_target_pose(self, target_pose):
        """
        Set the target to be used for the manipulation task.

        Args:
            target_pose: pose of the target object.
        """
        
        # get goal angle
        goal = self.robot.cartesian_to_joints(target_pose)
        
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
            print("\n" + "-" * 20 + "\n" + "Obstacle list is empty" + "\n" + "-" * 20 + "\n")
        else:
            for obstacle_id in self.obstacles:
                item_info = p.getBodyInfo(obstacle_id)
                item_name = item_info[1].decode("utf-8")
                print(f"Obstacle Name: {item_name}, ID: {obstacle_id}")
    
    # ----------------------------------------------------------------
    # functions for plan 
    # ----------------------------------------------------------------

    def plan(self, start, goal):
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
        solved = self.ss.solve(self.planning_time)
        if solved:
            sol_path_geometric = self.ss.getSolutionPath()
            sol_path_geometric.interpolate(self.interpolate_num)     # Linear interpolation, Generate more intermediate states to make the path smoother and more refined
            sol_path_states = sol_path_geometric.getStates()
            path = [self.state_to_list(state) for state in sol_path_states]
            print("\n" + "-" * 20 + "\n" + "End planning"+ "\n" + "-" * 20 + "\n")
            return path
        else:
            raise RuntimeError("No solution found!")
    
    # ----------------------------------------------------------------
    # Utils
    # ----------------------------------------------------------------
    
    def state_to_list(self, state):
        return [state[i] for i in range(self.DOF)]