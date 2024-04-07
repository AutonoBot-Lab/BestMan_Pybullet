"""
@Description :   A few functions for planning
@Author      :   Yan Ding 
@Time        :   2023/08/30 22:52:39
"""


import pybullet as p
import math
import numpy as np
from .pb_ompl import *


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
        
        self.robot = robot
        self.client = robot.client
        self.client_id = self.client.get_client_id()
        
        # parameters for arm
        self.arm_id = robot.get_arm_id()
        self.joint_idx = robot.get_joint_idx()
        self.tcp_link = robot.get_tcp_link()
        # self.arm = PbOMPLRobot(self.arm_id, joint_idx=self.joint_idx)
        self.max_attempts = 500
        self.threshold = Planner_cfg.threshold

        # obstacles
        self.obstacles = []
        self.pb_ompl_interface = pb_ompl(self.robot, self.obstacles)

        # select planner
        self.set_planner(Planner_cfg.planner)

        # output OMPL settings
        item_info = p.getBodyInfo(self.arm_id)
        robot_name = item_info[1].decode("utf-8")
        print("--------------------")
        print(
            f"OMPL Configuration\n"
            f"Robot ID: {self.arm_id}\n"
            f"Robot Name: {robot_name}\n"
            f"Obstacles: {self.obstacles}\n"
            f"Planner: {Planner_cfg.planner}\n"
            f"Threshold: {self.threshold}"
        )
        print("--------------------")

    # ----------------------------------------------------------------
    # set planner / goal
    # ----------------------------------------------------------------
    
    def set_planner(self, planner):
        """
        Set Planner to OMPL.

        Args:
                planner: The planner to set to OMPL.
        """
        try:
            self.pb_ompl_interface.set_planner(planner)
        except Exception as e:
            print(f"Error setting planner: {e}")

    def set_target(self, target_id):
        """
        Set the target to be used for the manipulation task.

        Args:
                target_id: id of the target.
        """
              
        try:
            _ = p.getBodyInfo(target_id)
        except Exception as e:
            print(f"Target object not set: {e}")
        
        # self.target_pos, _ = p.getBasePositionAndOrientation(target_id)
        # _, _, min_z, _, _, max_z = self.pb_client.get_bounding_box(target_id)
        min_x, min_y, _, max_x, max_y, max_z = self.client.get_bounding_box(target_id)
        
        # set target object postion
        self.target_pos = (
            (min_x + max_x) / 2,
            (min_y + max_y) / 2,
            max_z
        )
        
        target_orientation = [0.0, math.pi / 2.0, 0.0]  # vertical
        self.goal = self.robot.cartesian_to_joints(position=[self.target_pos[0], self.target_pos[1], max_z + self.robot.tcp_height + 0.01], 
                                                   orientation=target_orientation)
    
    # ----------------------------------------------------------------
    # obstacles
    # ----------------------------------------------------------------
    
    def add_scene_obstacles(self):
        """
        Add obstacles to the scene.
        This is done by iterating over all items in the scene and adding them to the list of obstacles.

        Args:
            display: If True the name and ID of the item will be displayed on the screen.
            Default is False. If False it will not be displayed.

        Returns:
                A list of IDs of all items in the obstacle list.
        """
        
        self.obstacles = []

        num_items = p.getNumBodies()

        # This function will return the list of all the items in the list
        for item_id in range(num_items):
            if item_id == self.arm_id:
                continue
            self.obstacles.append(item_id)
        
        self.store_obstacles()
        return self.obstacles
    
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

    def store_obstacles(self):
        """
        Store obstacles in the OMPL interface.
        This is called after the user finished obstacles setting.
        """
        
        self.pb_ompl_interface.set_obstacles(self.obstacles)

    # ----------------------------------------------------------------
    # Impact checking functions
    # ----------------------------------------------------------------
    
    def setup_collision_detection(
        self, robot, obstacles, self_collisions=True, allow_collision_links=[]
    ):
        self.check_link_pairs = (
            get_self_link_pairs(robot.id, robot.joint_idx)
            if self_collisions
            else []
        )
        moving_links = frozenset(
            [
                item
                for item in get_moving_links(robot.id, robot.joint_idx)
                if not item in allow_collision_links
            ]
        )
        moving_bodies = [(robot.id, moving_links)]
        self.check_body_pairs = list(product(moving_bodies, obstacles))
    
    def is_state_valid(self, state):
        # satisfy bounds TODO
        # Should be unecessary if joint bounds is properly set

        # check self-collision
        self.robot.set_state(state)
        for link1, link2 in self.check_link_pairs:
            if pairwise_link_collision(
                self.robot_id, link1, self.robot_id, link2
            ):
                return False

        # check collision against environment
        for body1, body2 in self.check_body_pairs:
            if pairwise_collision(body1, body2):
                return False
        return True
    
    
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
        # print('debug! end_effector_pose:{} target_pos:{}'.format(end_effector_pose[0], self.target_pos))
        return distance

    def plan(self, start, goal):
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
        
        # self.arm.set_state(start)
        self.robot.set_arm_to_joint_angles(start)
        res, path = self.pb_ompl_interface.plan_start_goal(start, goal)
        return res, path

    def execute(self, path):
        """
        Execute a given path using the OMPL interface and handle errors.

        Args:
            path: The path to be executed. This should be absolute path.
        """
        try:
            self.pb_ompl_interface.execute(path)
        except Exception as e:
            print(f"Error executing path: {e}")
    
    def plan_execute(self):  # TODO refactor
        """
        Reach an object from start to goal
        """
        
        # current arm joint angles
        start = self.robot.get_arm_joint_angles()
        
        attempts = 0
        while attempts < self.max_attempts:
            attempts += 1
            res, path = self.plan(start, self.goal)

            # Execute the path and attach the object to the robot
            if res:
                self.pb_ompl_interface.execute(path)
                # Check if the robot is close to the object
                if self.tcp_link != -1:
                    distance = self.compute_distance(self.tcp_link)
                else:
                    distance = self.compute_distance(self.robot.end_effector_link_index)
                    print("Attention, the distance is computed without tcp link")
                print("debug! distance:{}".format(distance))
                # This method grasses the robot if the distance is below threshold.
                if distance <= self.threshold:
                    print(
                        "After {} trials, successfully grasped (error:{}).".format(
                            attempts, distance
                        )
                    )
                    return True, path

        if attempts >= self.max_attempts:
            print(
                "Could not reach target position without collision after {} attempts".format(
                    self.max_attempts
                )
            )
        return False, None