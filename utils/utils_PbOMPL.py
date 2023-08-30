import cv2
import pybullet as p
import pybullet_data
import math
import time
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import random
import sys
import os
from matplotlib.colors import LinearSegmentedColormap
import utils.pb_ompl

class PbOMPL:
    def __init__(self, pb_client, arm_id, joint_idx, tcp_link, obstacles=[], planner="RRTConnect", threshold=0.1):
        """
        Initialize the OMPL.

        Args:
           arm_id: ID of the robot to use.
           joint_idx: Index of the joints to use in planning.
           obstacles: List of obstacles to consider in the motion planning.
           planner: The name of the motion planner algorithm to use.
           threshold: The threshold value ONLY for repalnning.
        """
        self.pb_client = pb_client
        self.client_id = self.pb_client.get_client()

        # parameters for arm
        self.arm_id = arm_id
        self.arm = utils.pb_ompl.PbOMPLRobot(arm_id, joint_idx=joint_idx)
        self.tcp_link = tcp_link
        self.max_attempts = 500
        self.threshold = threshold

        # obstacles for planning
        self.obstacles = obstacles if obstacles is not None else []
        self.pb_ompl_interface = utils.pb_ompl.PbOMPL(self.arm, self.obstacles)
        
        # select planner
        self.set_planner(planner)

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
            item_info = p.getBodyInfo(target_id)
        except Exception as e:
            print(f"Error setting target: {e}")
        self.target = target_id
        self.target_pos, _ = p.getBasePositionAndOrientation(self.target)
        # consider the object's height
        _, _, min_z, _, _, max_z = self.pb_client.get_bounding_box(self.target)
        self.target_pos = (self.target_pos[0], self.target_pos[1], self.target_pos[2] + max_z - min_z + 0.01)
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
        for i in obstacles:
            try:
                item_info = p.getBodyInfo(i)
            except Exception as e:
                print(f"Error adding obstacle: {e}")
        self.obstacles = obstacles

    def add_obstacles(self, item_id):
        """
        Add obstacle to list of obstacles.

        Args:
                item_id: id of the item to add.
        """
        try:
            item_info = p.getBodyInfo(item_id)
        except Exception as e:
            print(f"Error adding obstacle: {e}")
        self.obstacles.append(item_id)

    def remove_obstacles(self, obstacle_id):
        """
        Remove obstacle from the list of obstacles.
        This is useful for removing a specific obstacle from the list.

        Args:
                obstacle_id: id of the obstacle to remove.
        """
        try:
            self.obstacles = [obs for obs in self.obstacles if obs != obstacle_id]
        except Exception as e:
            print(f"Error removing obstacle: {e}")

    def store_obstacles(self):
        """
        Store obstacles in the OMPL interface.
        This is called after the user finished obstacles setting.
        """
        self.pb_ompl_interface.set_obstacles(self.obstacles)

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

        self.store_obstacles()
        return self.obstacles

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

    def plan_grasp(self, start, goal):
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
        self.arm.set_state(start)
        res, path = self.pb_ompl_interface.plan(goal)
        return res, path

    def move_end_effector_to_goal_position(self, start, goal, end_effector_link_index):  # TODO refactor
        """
        Move end effector to goal position in OMPL planned path.

        Args:
                start: effective position of end effector.
                goal: effective position of goal.
                end_effector_link_index: index of end effector.
        """
        grasp_successful = False

        trial = 0
        while not grasp_successful:
            trial += 1
            res, path = self.plan_grasp(start, goal)

            if res:
                self.pb_ompl_interface.execute(path, dynamics=True)
                # Pause briefly to simulate real-time
                time.sleep(0.1)
                grasp_successful = True
                print("After {} trials, finished.".format(trial))
                # step simulation in a loop.
                for _ in range(100):
                    p.stepSimulation()
                    time.sleep(1.0 / 100.0)

    def reach_object(self, start, goal, end_effector_link_index):  # TODO refactor
        """
        Plans an object from start to goal and attaches it to the robot.

        Args:
                start: effective position of end effector.
                goal: effective position of goal.
                end_effector_link_index: index of end effector.
        """
        attempts = 0
        while attempts < self.max_attempts:
            attempts += 1
            res, path = self.plan_grasp(start, goal)

            # Execute the path and attach the object to the robot
            if res:
                self.pb_ompl_interface.execute(path)
                # Pause briefly to simulate real-time
                time.sleep(0.1)
                # Check if the robot is close to the object
                if self.tcp_link != -1:
                    distance = self.compute_distance(self.tcp_link)
                else:
                    distance = self.compute_distance(end_effector_link_index)
                    print('Attention, the distance is computed without tcp link')
                print('debug! distance:{}'.format(distance))
                # This method grasses the robot if the distance is below threshold.
                if distance <= self.threshold:
                    print("After {} trials, successfully grasped (error:{}).".format(attempts, distance))
                    return True
                    break
                    # Attach the object to the robot
                    cube_orn = p.getQuaternionFromEuler([0, math.pi, 0])
                    self.gripper_id = p.createConstraint(
                        self.arm_id,
                        end_effector_link_index,  # TODO refactor
                        self.target,
                        -1,
                        p.JOINT_FIXED,
                        [0, 0, 0],
                        [0, 0, 0.05],
                        [0, 0, 0],
                        childFrameOrientation=cube_orn,
                    )
        if attempts >= self.max_attempts:
            print(
                "Could not reach target position without collision after {} attempts".format(
                    self.max_attempts
                )
            )
        return False