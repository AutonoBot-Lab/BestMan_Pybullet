"""
@Description :   A few functions for planning
@Author      :   Yan Ding 
@Time        :   2023/08/30 22:52:39
"""


import pybullet as p
import numpy as np

"""
Get the utils module path
"""
import sys
import os

sys.path.append('refactor')

from itertools import product
from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
from .utils import *

INTERPOLATE_NUM = 500
DEFAULT_PLANNING_TIME = 5.0

# real vector state space
class StateSpace(ob.RealVectorStateSpace):
    def __init__(self, num_dim) -> None:
        super().__init__(num_dim)
        self.num_dim = num_dim
        self.state_sampler = None

    def allocStateSampler(self):
        """
        This will be called by the internal OMPL planner
        """
        # WARN: This will cause problems if the underlying planner is multi-threaded!!!
        if self.state_sampler:
            return self.state_sampler

        # when ompl planner calls this, we will return our sampler
        return self.allocDefaultStateSampler()

    def set_state_sampler(self, state_sampler):
        """
        Optional, Set custom state sampler.
        """
        self.state_sampler = state_sampler
        

class OMPL_Planner:
    def __init__(
        self,
        pb_client,
        arm_id,
        joint_idx,
        tcp_link,
        obstacles=[],
        planner="RRTConnect",
        threshold=0.1,
    ):
        """
        Initialize the OMPL_Planner.

        Args:
           arm_id: ID of the arm to use.
           joint_idx: Index of the joints to use in planning.
           obstacles: List of obstacles to consider in the motion planning.
           planner: The name of the motion planner algorithm to use.
           threshold: The threshold value ONLY for repalnning.
        """
        
        self.pb_client = pb_client
        self.client_id = self.pb_client.get_client()

        # parameters for arm
        self.arm_id = arm_id
        self.joint_idx = joint_idx
        # self.arm = pb_ompl.PbOMPLRobot(arm_id, joint_idx=joint_idx)
        self.tcp_link = tcp_link
        self.max_attempts = 500
        self.threshold = threshold

        # obstacles for planning
        self.obstacles = obstacles if obstacles is not None else []
        # self.pb_ompl_interface = ompl.PbOMPL(self.arm, self.obstacles)
        
        # select planner
        self.set_planner(planner)

        # real vector state space
        self.num_dim = len(joint_idx)
        self.space = StateSpace(self.num_dim)
        
        # Set the boundaries of the state space
        bounds = ob.RealVectorBounds(self.num_dim)
        for i, joint_id in enumerate(self.joint_idx):
            joint_info = p.getJointInfo(self.id, joint_id)
            low = joint_info[8]  # low bounds
            high = joint_info[9]  # high bounds
            if low < high:
                bounds.setLow(i, low)
                bounds.setHigh(i, high)
                print('-' * 20 + '\n' + f'!debug:[{low}, {high}]')
        self.space.setBounds(bounds)

        # Configuring Route Planner Settings
        self.ss = og.SimpleSetup(self.space)
        self.ss.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_state_valid))
        self.si = self.ss.getSpaceInformation()
        
        # output OMPL settings
        item_info = p.getBodyInfo(self.arm_id)
        robot_name = item_info[1].decode("utf-8")
        print("--------------------")
        print(
            f"OMPL Configuration\n"
            f"Robot ID: {arm_id}\n"
            f"Robot Name: {robot_name}\n"
            f"Obstacles: {obstacles}\n"
            f"Planner: {planner}\n"
            f"Threshold: {threshold}"
        )
        print("--------------------")

    def set_planner(self, planner):
        """
        Set Planner to OMPL.

        Args:
                planner: The planner to set to OMPL.
        """
        # try:
        #     self.pb_ompl_interface.set_planner(planner)
        # except Exception as e:
        #     print(f"Error setting planner: {e}")
        
        if planner == "PRM":
            self.planner = og.PRM(self.ss.getSpaceInformation())
        elif planner == "RRT":
            self.planner = og.RRT(self.ss.getSpaceInformation())
        elif planner == "RRTConnect":
            self.planner = og.RRTConnect(self.ss.getSpaceInformation())
        elif planner == "RRTstar":
            self.planner = og.RRTstar(self.ss.getSpaceInformation())
        elif planner == "EST":
            self.planner = og.EST(self.ss.getSpaceInformation())
        elif planner == "FMT":
            self.planner = og.FMT(self.ss.getSpaceInformation())
        elif planner == "BITstar":
            self.planner = og.BITstar(self.ss.getSpaceInformation())
        else:
            print("{} not recognized, please add it first".format(planner))
            return

        self.ss.setPlanner(self.planner)

    def set_target(self, target_id):
        """
        Set the target to be used for the manipulation task.

        Args:
                target_id: id of the target.
        """
        self.target = target_id
        self.target_pos, _ = p.getBasePositionAndOrientation(self.target)
        _, _, min_z, _, _, max_z = self.pb_client.get_bounding_box(self.target)
        
        # consider the object's height
        self.target_pos = (
            self.target_pos[0],
            self.target_pos[1],
            self.target_pos[2] + max_z - min_z + 0.01,
        )
        # print("debug! target position:{}".format(self.target_pos))

    def set_target_pos(self, target_pos):
        """
        Set the position of the target.

        Args:
                target_pos: The position of the target.
        """
        self.target_pos = target_pos

    def set_obstacles(self, obstacles):
        """
        Set obstacles to OMPL.

        Args:
                obstacles: List of obstacle.
        """
        self.obstacles = obstacles

    def add_obstacles(self, obstacle_id):
        """
        Add obstacle to list of obstacles.

        Args:
                obstacle_id: id of the item to add.
        """
        self.obstacles.append(obstacle_id)

    def remove_obstacles(self, obstacle_id):
        """
        Remove obstacle from the list of obstacles.
        This is useful for removing a specific obstacle from the list.

        Args:
                obstacle_id: id of the obstacle to remove.
        """
        self.obstacles.remove(obstacle_id)

    # def store_obstacles(self):
    #     """
    #     Store obstacles in the OMPL interface.
    #     This is called after the user finished obstacles setting.
    #     """
    #     self.pb_ompl_interface.set_obstacles(self.obstacles)
    
    def check_obstacles(self):
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
                print(f"\t Obstacle Name: {item_name}, ID: {obstacle_id}")

    def get_scene_items(self, display=True):
        """
        Get IDs of all items in the scene.
        This is a debugging function.

        Args:
                display: If True ( default ) the ID will be displayed on the screen.

        Returns:
                A list of IDs of all items in the scene.
        """
        # Get the total number of items in the scene
        num_items = p.getNumBodies()
        # Initialize an empty list to store the IDs of all items in the scene
        all_item_ids = []

        # This function will return the list of all the items in the list
        for item_id in range(num_items):
            current_id = p.getBodyUniqueId(item_id)
            all_item_ids.append(current_id)
            if display:
                item_info = p.getBodyInfo(current_id)
                item_name = item_info[1].decode("utf-8")
                print(f"Item Name: {item_name}, ID: {current_id}")
        return all_item_ids

    def add_scene_obstacles(self, display=False):
        """
        Add obstacles to the scene.
        This is done by iterating over all items in the scene and adding them to the list of obstacles.

        Args:
            display: If True the name and ID of the item will be displayed on the screen.
            Default is False. If False it will not be displayed.

        Returns:
                A list of IDs of all items in the obstacle list.
        """
        all_item_ids = self.get_scene_items(display=False)
        self.obstacles = []
        # Add the current item ID to the list of obstacles
        for item_id in all_item_ids:
            # Skip the robot ID
            if item_id == self.arm_id:
                continue
            self.obstacles.append(item_id)
            if display:
                item_info = p.getBodyInfo(item_id)
                item_name = item_info[1].decode("utf-8")
                print(f"Item Name: {item_name}, ID: {item_id}")

        # update collision detection
        self.setup_collision_detection()
    
    def setup_collision_detection(
        self, self_collisions=True, allow_collision_links=[]
    ):
        self.check_link_pairs = (
            utils.get_self_link_pairs(self.arm_id, self.joint_idx)
            if self_collisions
            else []
        )
        moving_links = frozenset(
            [
                item
                for item in utils.get_moving_links(self.arm_id, self.joint_idx)
                if not item in allow_collision_links
            ]
        )
        moving_bodies = [(self.arm_id, moving_links)]
        self.check_body_pairs = list(product(moving_bodies, self.obstacles))
        
    def is_state_valid(self, state):
        # satisfy bounds TODO
        # Should be unecessary if joint bounds is properly set

        # check self-collision
        self.robot.set_state(utils.state_to_list(state))
        for link1, link2 in self.check_link_pairs:
            if utils.pairwise_link_collision(
                self.robot_id, link1, self.robot_id, link2
            ):
                # print(get_body_name(body), get_link_name(body, link1), get_link_name(body, link2))
                return False

        # check collision against environment
        for body1, body2 in self.check_body_pairs:
            if utils.pairwise_collision(body1, body2):
                # print('body collision', body1, body2)
                # print(get_body_name(body1), get_body_name(body2))
                return False
        return True
    
    def set_state_sampler(self, state_sampler):
        self.space.set_state_sampler(state_sampler)

    # def compute_distance(self, end_effector_link_index):
    #     """
    #     Compute the distance between the end effector and the object.

    #     Args:
    #             end_effector_link_index: index of the end effector link.

    #     Returns:
    #             distance: distance between the end - effector and the object.
    #     """
    #     end_effector_pose = p.getLinkState(self.arm_id, end_effector_link_index)

    #     # Compute the distance between the end-effector and the object
    #     distance = np.linalg.norm(
    #         np.array(end_effector_pose[0]) - np.array(self.target_pos)
    #     )
    #     # print('debug! end_effector_pose:{} target_pos:{}'.format(end_effector_pose[0], self.target_pos))
    #     return distance

    def plan_grasp(self, start, goal, allowed_time=DEFAULT_PLANNING_TIME):
        """
        Plan grasp from start to goal.
        This is a wrapper around OMPL grasp planning algorithm.

        Args:
                start: state to start planning from.
                goal: state to go to after planning has been completed.

        Returns:
            res: response from ompl interface.
            path: a list of robot state.
        """
        
        # set arm state
        p.setJointMotorControlArray(
            bodyUniqueId=self.arm_id,
            jointIndices=self.joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPositions=start, 
        )
        # for joint, value in zip(self.joint_idx, start):
        #     p.resetJointState(self.id, joint, value, targetVelocity=0)
        
        
        
        print("start_planning")
        print(self.planner.params())

        # set the start and goal states;
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
            path = [self.state_to_list(state) for state in sol_path_states]
            for sol_path in path:
                self.is_state_valid(sol_path)
        else:
            print("No solution found")

        # reset robot state
        p.setJointMotorControlArray(
            bodyUniqueId=self.arm_id,
            jointIndices=self.joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPositions=start, 
        )
            
        return path

    # def move_end_effector_to_goal_position(
    #     self, start, goal, end_effector_link_index
    # ):  # TODO refactor
    #     """
    #     Move end effector to goal position in OMPL planned path.

    #     Args:
    #             start: effective position of end effector.
    #             goal: effective position of goal.
    #             end_effector_link_index: index of end effector.
    #     """
    #     grasp_successful = False

    #     trial = 0
    #     while not grasp_successful:
    #         trial += 1
    #         res, path = self.plan_grasp(start, goal)

    #         if res:
    #             self.pb_ompl_interface.execute(path, dynamics=True)
    #             grasp_successful = True
    #             print("After {} trials, finished.".format(trial))
    #             for _ in range(100):
    #                 p.stepSimulation()

    def plan(self, start, goal):  # TODO refactor
        """
        Reach an object from start to goal

        Args:
                start: effective position of end effector.
                goal: effective position of goal.
                end_effector_link_index: index of end effector.
        """
        attempts = 0
        while attempts < self.max_attempts:
            attempts += 1
            path = self.plan_grasp(start, goal)
            if path:    # Non-empty, there is a feasible solution
                return path
            
            # # Check if the robot is close to the object
            # if self.tcp_link != -1:
            #     distance = self.compute_distance(self.tcp_link)
            # else:
            #     distance = self.compute_distance(end_effector_link_index)
            #     print("Attention, the distance is computed without tcp link")
            # print("debug! distance:{}".format(distance))
            # # This method grasses the robot if the distance is below threshold.
            # if distance <= self.threshold:
            #     print(
            #         "After {} trials, successfully grasped (error:{}).".format(
            #             attempts, distance
            #         )
            #     )
                    
        if attempts >= self.max_attempts:
            print(
                "Could not plan target position without collision after {} attempts".format(
                    self.max_attempts
                )
            )
        return None
    
    # def execute(self, path, dynamics=False):
    #     """
    #     Execute a given path using the OMPL interface and handle errors.

    #     Args:
    #         path: The path to be executed. This should be absolute path.
    #     """
    #     for q in path:
    #         if dynamics:
    #             for i in range(self.robot.num_dim):
    #                 p.setJointMotorControl2(
    #                     self.robot.id, i, p.POSITION_CONTROL, q[i], force=5 * 240.0
    #                 )
    #         else:
    #             self.robot.set_state(q)
    #         p.stepSimulation()
