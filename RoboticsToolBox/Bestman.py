"""
@Description :   A few functions used in bestman robot, where the robot has a base and an arm.
@Author      :   Yan Ding 
@Time        :   2023/08/30 20:43:44
"""

import cv2
import pybullet as p
import math
import time
import numpy as np
import matplotlib.pyplot as plt
import random
from .Pose import Pose
from Controller.PIDController import PIDController

class Bestman:
    def __init__(self, client, visualizer,  cfg):
        """
        Initialize a new object.

        Parameters:
            init_pos (list, optional): A list of three floats representing the initial position. Defaults to [0, 0, 0].
            client (object): The pybullet client object.

        Attributes:
            client (object): The pybullet client object.
            client_id (int): The client id returned by the pybullet client.
            
            frequency (int): The frequency of the PID controller.
            timeout (float): The timeout value for the PID controller.
            max_force (int): The maximum force for the PID controller.
            max_iterations (int): The maximum number of iterations for the PID controller.
            threshold (float): The threshold value for the PID controller.
            max_attempts (int): The maximum number of attempts for the PID controller.
            target_distance (float): The target distance for the PID controller.
            controller (object): The PID controller object.
            rotated (bool): A flag indicating whether the object has been rotated.
            
            init_pos (object): The initial position object.
            base_id (int): The base id of the URDF model.
            arm_id (int): The arm id of the URDF model.
            arm_joints_idx (list): A list of joint indexes.
            arm_height (float): The height of the arm.
            end_effector_index (int): The index of the end effector.
            tcp_link (int): The tcp link index.
            tcp_height (float): The height of the tcp link.
            
            visualizer (object): The visualizer object.
            gripper_id (None): The gripper id. Initialized to None.
        """

        # Extract config
        robot_cfg = cfg.Robot
        controller_cfg = cfg.Controller
        
        self.client = client
        self.client_id = self.client.get_client_id()
        
        self.visualizer = visualizer
        
        # Init PID controller
        self.frequency = controller_cfg.frequency
        self.timeout = controller_cfg.timeout
        self.max_force = controller_cfg.max_force
        self.max_iterations = controller_cfg.max_iterations
        self.threshold = controller_cfg.threshold
        self.max_attempts = controller_cfg.max_attempts
        self.target_distance = controller_cfg.target_distance
        self.distance_controller = PIDController(
            Kp=controller_cfg.Kp, Ki=controller_cfg.Ki, Kd=controller_cfg.Kd, setpoint=self.target_distance
        )
        self.rotated = False

        # Init base
        init_pose = Pose(robot_cfg.init_pose[:3], robot_cfg.init_pose[3:])
        self.base_id = p.loadURDF(
            fileName=robot_cfg.base_urdf_path,
            basePosition=init_pose.position,
            baseOrientation=p.getQuaternionFromEuler(init_pose.orientation),
            useFixedBase=True,
            physicsClientId=self.client_id,
        )

        # Init arm
        self.arm_id = p.loadURDF(
            fileName=robot_cfg.arm_urdf_path,
            basePosition=init_pose.position,
            baseOrientation=p.getQuaternionFromEuler(init_pose.orientation),
            useFixedBase=True,
            physicsClientId=self.client_id,
        )
        self.arm_joints_idx = robot_cfg.arm_joints_idx
        self.DOF = len(self.arm_joints_idx)
        self.arm_height = robot_cfg.arm_height

        # Add constraint between base and arm
        p.createConstraint(
            parentBodyUniqueId=self.base_id,
            parentLinkIndex=-1,
            childBodyUniqueId=self.arm_id,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0],
            physicsClientId=self.client_id,
        )
        
        # synchronize base and arm positions
        self.current_yaw = init_pose.yaw
        self.sim_sync_base_arm_pose()
        
        # Init arm joint angle
        self.move_arm_to_joint_values(robot_cfg.init_joint)
        
        # change robot color
        self.visualizer.change_robot_color(self.base_id, self.arm_id, False)
        
        # update image
        self.visualizer.set_camera(self.base_id)
        
        # get tcp link
        # if filename.endswith("ur5e.urdf"):
        #     self.tcp_link = -1
        #     self.tcp_height = 0
        # elif filename.endswith("ur5e_vacuum.urdf"):
        #     self.tcp_link = 8
        #     self.tcp_height = 0.065
        # elif filename.endswith("ur5e_vacuum_long.urdf"):
        #     self.tcp_link = 11
        #     self.tcp_height = 0.11
        # elif filename.endswith("ur5_robotiq_85.urdf"):
        #     self.tcp_link = 18
        #     self.tcp_height = 0  # TODO
        # else:
        #     print("Unknown tcp link: {}")
        #     sys.exit()
        
        self.end_effector_index = robot_cfg.end_effector_index
        self.tcp_link = robot_cfg.tcp_link
        self.tcp_height = robot_cfg.tcp_height

        # # Set arm and base colors
        # self.visualizer = Visualizer(client)
        # self.visualizer.set_robot_visual_color(self.base_id, self.arm_id)

        # global parameters
        self.init_pose = init_pose   # Used when resetting the robot position
        self.gripper_id = None      # Constraints between the end effector and the grasped object, None when not grabbed

    # ----------------------------------------------------------------
    # functions for sim
    # ----------------------------------------------------------------
    def sim_adjust_arm_height(self, height):
        """
        Dynamically adjusts the height of the robot arm.
        
        Args:
            height (float): The new height to set for the robot arm.
        """
        self.arm_height = height
        self.client.run(100)
        print("-" * 20 + "\n" + "Arm height has changed into {}".format(height))

    def sim_sync_base_arm_pose(self):
        """
        Synchronizes the position of the robot arm with the base.
        
        This function ensures that the positions of the robot arm and base are aligned.
        """
        
        position, orientation = p.getBasePositionAndOrientation(
            self.base_id, physicsClientId=self.client_id
        )
        position = [position[0], position[1], self.arm_height]  # fixed height
        p.resetBasePositionAndOrientation(
            self.arm_id, position, orientation, physicsClientId=self.client_id
        )

    def sim_action(self, output):
        """
        Ajust base position using PID controller's output

        Args:
            output (float): The output of the PID controller, which is used to calculate the new position of the robot's base.
        """
        position, orientation = p.getBasePositionAndOrientation(
            self.base_id, physicsClientId=self.client_id
        )
        euler_angles = p.getEulerFromQuaternion(
            orientation, physicsClientId=self.client_id
        )

        delta_x = output * math.cos(euler_angles[2])
        delta_y = output * math.sin(euler_angles[2])

        target_position = [position[0] + delta_x, position[1] + delta_y, position[2]]
        target_orientation = orientation

        result = self.check_collision_navigation()  # check if collision exists
        if result:
            assert "Collision detected during navigation, stopping..."

        p.resetBasePositionAndOrientation(
            self.base_id,
            target_position,
            target_orientation,
            physicsClientId=self.client_id,
        )
        self.sim_sync_base_arm_pose()
        # time.sleep(1.0 / self.frequency)

    def sim_set_arm_to_joint_values(self, joint_values):
        """
        Set arm to move to a specific set of joint angles, witout considering physics

        Args:
            joint_values: A list of desired joint angles (in radians) for each joint of the arm.
        """
        
        for joint, value in zip(self.arm_joints_idx, joint_values):
            p.resetJointState(self.arm_id, joint, value, targetVelocity=0)
            
    def sim_debug_set_joint_values(self):
        """
        Manually set each joint value of the arm for debugging purposes.
        """
        
        joint_values = self.get_current_joint_values()
        print("Current joint angles: {}".format(joint_values))

        for i in self.arm_joints_idx:
            joint_value = input(
                "Enter value for joint {} (current value: {}) or 'skip' to keep current value: ".format(
                    i, joint_values[i]
                )
            )
            if joint_value.lower() == "q":
                print("Skipping joint {}".format(i))
                continue
            try:
                joint_values[i] = float(joint_value)
            except ValueError:
                print("Invalid input. Keeping current value for joint {}.".format(i))

        self.sim_set_arm_to_joint_values(joint_values)
        print("Updated joint angles: {}".format(joint_values))
    
    # ----------------------------------------------------------------
    # functions for arm & base
    # ----------------------------------------------------------------
    def get_robot_max_size(self):
        """
        Retrieves the maximum size of the robot (arm and base) in meters.

        Returns:
            float: The maximum size of the robot in meters.
        """

        (
            min_x_base,
            min_y_base,
            _,
            max_x_base,
            max_y_base,
            _,
        ) = self.client.get_bounding_box(self.base_id)
        
        (
            min_x_arm,
            min_y_arm,
            _,
            max_x_arm,
            max_y_arm,
            _,
        ) = self.client.get_bounding_box(self.arm_id)
        
        robot_size = max(
            max_x_base - min_x_base,
            max_y_base - min_y_base,
            max_x_arm - min_x_arm,
            max_y_arm - min_y_arm,
        )
        
        return robot_size

    # ----------------------------------------------------------------
    # functions for arm
    # ----------------------------------------------------------------
    
    def print_joint_link_info(self, name):
        """
        print base/arm joint and link info of robot
        
        Args:
            name(str): 'base' or 'arm'
        """
        
        if name == "base":
            id = self.base_id
        elif name == "arm":
            id = self.arm_id
        else:
            print(
                "unknown name: {}, please input base or arm!".format(name)
            )

        num_joints = p.getNumJoints(id, physicsClientId=self.client_id)
        print("-" * 20 + "\n" + "Robot {} has {} joints".format(id, num_joints))
        for i in range(num_joints):
            joint_info = p.getJointInfo(id, i, physicsClientId=self.client_id)
            joint_name = joint_info[1]
            joint_state = p.getJointState(id, i, physicsClientId=self.client_id)
            joint_angle = joint_state[0]
            link_name = joint_info[12].decode("UTF-8")
            print(
                "Joint index:{}, name:{}, angle:{}".format(i, joint_name, joint_angle)
            )
            print("Link index: {}, name: {}".format(i, link_name))

    def get_arm_id(self):
        """
        Retrieves the ID of the robot arm.
        
        Returns:
            int: The ID of the robot arm.
        """  
        return self.arm_id
    
    def get_DOF(self):
        """
        Retrieves the degree of freedom (DOF) of the robot arm.
        
        Returns:
            int: The degree of freedom of the robot arm.
        """
        return self.DOF
    
    def get_joint_idx(self):
        """
        Retrieves the indices of the joints in the robot arm.
        
        Returns:
            list: A list of indices for the joints in the robot arm.
        """  
        return  self.arm_joints_idx
    
    def get_tcp_link(self):
        """
        Retrieves the TCP (Tool Center Point) link of the robot arm.
        
        Returns:
            str: The TCP link of the robot arm.
        """
        return self.tcp_link

    def get_joint_bounds(self):
        """
        Retrieves the joint bounds of the robot arm.
        
        By default, this function reads the joint bounds from pybullet.
        
        Returns:
            list: A list of tuples representing the joint bounds, where each tuple contains the minimum and maximum values for a joint.
        """
        
        joint_bounds = [p.getJointInfo(self.arm_id, joint_id)[8:10] for joint_id in self.arm_joints_idx]
        print("Joint bounds: {}".format(joint_bounds))
        return joint_bounds

    def get_current_joint_values(self):
        """
        Retrieve arm's joint angle
        """
            
        current_joint_values = [
            p.getJointState(self.arm_id, i, physicsClientId=self.client_id)[0]
            for i in self.arm_joints_idx
        ]
        
        return current_joint_values

    def move_arm_to_joint_values(self, joint_values):
        """
        Move arm to move to a specific set of joint angles, with considering physics

        Args:
            joint_values: A list of desired joint angles (in radians) for each joint of the arm.
        """
        
        p.setJointMotorControlArray(
            bodyIndex=self.arm_id,
            jointIndices=self.arm_joints_idx,
            controlMode=p.POSITION_CONTROL,
            targetPositions=joint_values,
            physicsClientId=self.client_id
        )

        start_time = time.time()  # avoid time anomaly

        while True:
            p.stepSimulation(physicsClientId=self.client_id)

            # Check if reach the goal
            current_angles = [
                p.getJointState(self.arm_id, i, physicsClientId=self.client_id)[0]
                for i in self.arm_joints_idx
            ]
            diff_angles = [abs(a - b) for a, b in zip(joint_values, current_angles)]
            if all(diff < self.threshold for diff in diff_angles):
                break

            if time.time() - start_time > self.timeout:  # avoid time anomaly
                print(
                    "-" * 20 + "\n" + "Timeout before reaching target joint position."
                )
                break
    
    def get_current_end_effector_pose(self):
        """
        Retrieve arm's end effect information
        """
        joint_info = p.getJointInfo(
            self.arm_id, self.end_effector_index, physicsClientId=self.client_id
        )
        end_effector_name = joint_info[1].decode("utf-8")
        end_effector_info = p.getLinkState(
            bodyUniqueId=self.arm_id,
            linkIndex=self.end_effector_index,
            physicsClientId=self.client_id,
        )
        end_effector_position = end_effector_info[0]
        end_effector_orientation = end_effector_info[1]
        print("-" * 20 + "\n" + "End effector name:{}".format(end_effector_name))
        print(
            "Position:{}; Orientation:{}".format(
                end_effector_position, end_effector_orientation
            )
        )
        return end_effector_position, end_effector_orientation

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
    
    def get_end_effector_link(self):
        # TODO
        return None

    def joints_to_cartesian(self, joint_values):
        """
        Transforms the robot arm's joint angles to its Cartesian coordinates.
        
        Args:
            joint_values (list): A list of joint angles for the robot arm.
        
        Returns:
            tuple: A tuple containing the Cartesian coordinates (position and orientation) of the robot arm.
        """
        # self.sim_set_arm_to_joint_values(joint_values)
        self.move_arm_to_joint_values(joint_values)
        end_effector_info = p.getLinkState(
            bodyUniqueId=self.arm_id,
            linkIndex=self.end_effector_index,
            physicsClientId=self.client_id,
        )
        orientation = p.getEulerFromQuaternion(
            end_effector_info[1], physicsClientId=self.client_id
        )
        position = end_effector_info[0]
        # print("-" * 30 + "\n" + "position:{}".format(position))
        # print("orientation:{}".format(orientation))
        return position

    def cartesian_to_joints(self, position, orientation):
        """
        Transforms the robot arm's Cartesian coordinates to its joint angles.
        
        Args:
            position (list): The Cartesian position of the robot arm.
            orientation (list): The Cartesian orientation of the robot arm.
        
        Returns:
            list: A list of joint angles corresponding to the given Cartesian coordinates.
        """
        joint_values = p.calculateInverseKinematics(
            bodyUniqueId=self.arm_id,
            endEffectorLinkIndex=self.end_effector_index,
            targetPosition=position,
            targetOrientation=p.getQuaternionFromEuler(orientation),
            maxNumIterations=self.max_iterations,
            residualThreshold=self.threshold,
            physicsClientId=self.client_id,
        )
        return joint_values

    def rotate_end_effector(self, angle):
        """
        Rotate the end effector of the robot arm by a specified angle.

        Args:
            angle (float): The desired rotation angle in radians.
        """

        # Change this to your actual joint indices
        joint_states = p.getJointStates(
            self.arm_id, self.arm_joints_idx, physicsClientId=self.client_id
        )

        # Create a new list of target joint angles
        target_joint_values = [joint_state[0] for joint_state in joint_states]

        # Add desired rotation to the last joint's current angle
        target_joint_values[-1] += angle

        # Set the target angles
        # self.sim_set_arm_to_joint_values(target_joint_values)
        self.move_arm_to_joint_values(target_joint_values)

        # Step the simulation until the joints reach their target angles
        while True:
            # Update the current joint states
            joint_states = p.getJointStates(
                self.arm_id, self.arm_joints_idx, physicsClientId=self.client_id
            )
            current_joint_values = [joint_state[0] for joint_state in joint_states]

            # Check if all joints have reached their target angles
            if all(
                abs(current_joint_values[i] - target_joint_values[i]) < 0.01
                for i in range(len(self.arm_joints_idx))
            ):
                break

            p.stepSimulation(physicsClientId=self.client_id)
            # time.sleep(1.0 / self.frequency)

        print("-" * 20 + "\n" + "Rotation completed!")

    def move_end_effector_to_goal_pose(self, end_effector_goal_pose):
        """
        Move arm's end effector to a target position.

        Args:
            end_effector_goal_pose (Pose): The desired pose of the end effector (includes both position and orientation).
        """

        # get current end effector position
        state = p.getLinkState(
            bodyUniqueId=self.arm_id,
            linkIndex=self.end_effector_index,
            physicsClientId=self.client_id,
        )
        current_position = state[0]

        # calculate distance
        distance = np.linalg.norm(
            np.array(end_effector_goal_pose.position) - np.array(current_position)
        )

        # # check if the distance is small enough, if yes, return immediately
        # if distance < self.threshold:  # use an appropriate value here
        #     print("Current position is already close to target position. No need to move.")
        #     return True

        target_position = end_effector_goal_pose.position
        target_orientation = end_effector_goal_pose.orientation
        # target_orientation = [0, math.pi / 2.0, 0]  # vcertical downward grip

        attempts = 0
        while attempts < self.max_attempts:
            joint_values = self.cartesian_to_joints(target_position, target_orientation)

            # If IK solution is invalid, break this loop and start a new attempt
            if joint_values is None or len(joint_values) != 6:
                break

            # Test the calculated angles for collision
            for joint_index, joint_angle in enumerate(joint_values):
                p.resetJointState(
                    self.arm_id,
                    joint_index,
                    joint_angle,
                    physicsClientId=self.client_id,
                )

            p.stepSimulation(physicsClientId=self.client_id)

            # if p.getContactPoints(self.arm_id): # collision with other objects
            if p.getContactPoints(
                self.arm_id, self.base_id, physicsClientId=self.client_id
            ):
                # Collision detected, break and re-calculate the IK
                break
            else:
                # get current end effector position
                state = p.getLinkState(
                    bodyUniqueId=self.arm_id,
                    linkIndex=self.end_effector_index,
                    physicsClientId=self.client_id,
                )
                current_position = state[0]

                # calculate error
                error = np.linalg.norm(
                    np.array(target_position) - np.array(current_position)
                )
                # print('current_position:{}, target_position:{}, error:{}'.format(current_position, target_position, error))

                if error < self.threshold * 2:
                    self.move_arm_to_joint_values(joint_values)
                    # print(
                    #     "Reached target end effector position:{}".format(
                    #         end_effector_goal_pose.position
                    #     )
                    # )
                    return True
            attempts += 1
        print(
            "-" * 20
            + "\n"
            + "Could not reach target position without collision after {} attempts".format(
                self.max_attempts
            )
        )
        return False

    def move_end_effector_to_goal_position(self, end_effector_goal_pose):
        """
        Move arm's end effector to a target position.
        
        Args:
            end_effector_goal_pose (Pose): The desired pose of the end effector (includes both position and orientation).
        """

        # get current end effector position
        state = p.getLinkState(
            bodyUniqueId=self.arm_id,
            linkIndex=self.end_effector_index,
            physicsClientId=self.client_id,
        )
        current_position = state[0]

        # calculate distance
        distance = np.linalg.norm(
            np.array(end_effector_goal_pose.position) - np.array(current_position)
        )

        # # check if the distance is small enough, if yes, return immediately
        # if distance < self.threshold:  # use an appropriate value here
        #     print("Current position is already close to target position. No need to move.")
        #     return True

        target_position = end_effector_goal_pose.position
        target_orientation = end_effector_goal_pose.orientation
        # target_orientation = [0, math.pi / 2.0, 0]  # vcertical downward grip

        attempts = 0
        while attempts < self.max_attempts:
            joint_values = self.cartesian_to_joints(target_position, target_orientation)

            # If IK solution is invalid, break this loop and start a new attempt
            if joint_values is None or len(joint_values) != 6:
                break

            # Test the calculated angles for collision
            for joint_index, joint_angle in enumerate(joint_values):
                p.resetJointState(
                    self.arm_id,
                    joint_index,
                    joint_angle,
                    physicsClientId=self.client_id,
                )

            p.stepSimulation(physicsClientId=self.client_id)

            # if p.getContactPoints(self.arm_id): # collision with other objects
            if p.getContactPoints(
                self.arm_id, self.base_id, physicsClientId=self.client_id
            ):
                # Collision detected, break and re-calculate the IK
                break
            else:
                # get current end effector position
                state = p.getLinkState(
                    bodyUniqueId=self.arm_id,
                    linkIndex=self.end_effector_index,
                    physicsClientId=self.client_id,
                )
                current_position = state[0]

                # calculate error
                error = np.linalg.norm(
                    np.array(target_position) - np.array(current_position)
                )
                # print('current_position:{}, target_position:{}, error:{}'.format(current_position, target_position, error))

                if error < self.threshold * 2:
                    self.move_arm_to_joint_values(joint_values)
                    # print(
                    #     "Reached target end effector position:{}".format(
                    #         end_effector_goal_pose.position
                    #     )
                    # )
                    return True
            attempts += 1
        print(
            "-" * 20
            + "\n"
            + "Could not reach target position without collision after {} attempts".format(
                self.max_attempts
            )
        )
        return False
    
    def calculate_IK_error(self, goal_position, goal_orientation):
        """
        Calculate the inverse kinematics (IK) error for performing pick-and-place manipulation of an object using a robot arm.

        Args:
            goal_position: The desired goal position for the target object.
            goal_orientation: The desired goal orientation for the target object.
        """
        joint_values = p.calculateInverseKinematics(
            bodyUniqueId=self.arm_id,
            endEffectorLinkIndex=self.end_effector_index,
            targetPosition=goal_position,
            targetOrientation=p.getQuaternionFromEuler(goal_orientation),
            maxNumIterations=self.max_iterations,
            residualThreshold=self.threshold,
            physicsClientId=self.client_id,
        )
        print("-" * 20 + "\n" + "IK: joint_values is {}".format(joint_values))

        self.move_arm_to_joint_values(joint_values)

        state = p.getLinkState(
            bodyUniqueId=self.arm_id,
            linkIndex=self.end_effector_index,
            physicsClientId=self.client_id,
        )
        actual_position = state[0]
        print("actual position: {}".format(actual_position))

        target_position = np.array(goal_position)
        actual_position = np.array(actual_position)
        difference = np.linalg.norm(target_position - actual_position)
        print("IK error: {}".format(difference))

    def execute_trajectory(self, trajectory):
        """
        Execute the path planned by Planner

        Args:
            trajectory: List, each element is a list of angles, corresponding to a transformation
        """
        for joints_value in trajectory:
            # print("joints_value:{}".format(joints_value))
            p.setJointMotorControlArray(
                bodyUniqueId=self.arm_id,
                jointIndices=[0, 1, 2, 3, 4, 5],
                controlMode=p.POSITION_CONTROL,
                targetPositions=joints_value,
            )
            p.stepSimulation(physicsClientId=self.client_id)
        print("-" * 20 + "\n" + "Excite trajectory finished!")

    def pick_place_v1(self, object_id, object_goal_pose):
        """
        Perform pick-and-place manipulation of an object using the robot arm.

        Args:
            object_id: The ID of the target object to be manipulated.
            object_goal_position: The goal position for the target object.
            object_goal_orientation: The goal orientation for the target object.
        """

        object_goal_position, object_goal_orientation = object_goal_pose.position, object_goal_pose.orientation
        
        # get target object position
        object_position_init, orientation = p.getBasePositionAndOrientation(
            object_id, physicsClientId=self.client_id
        )
        # consider the object's height
        _, _, min_z, _, _, max_z = self.client.get_bounding_box(object_id)

        temp_height1, temp_height2 = 0.05, 0.01
        gripper_status = {"ungrasp": 0, "grasp": 1}
        for step in range(100):
            if step < 20:  # phase 1: move 20cm over object
                gripper_value = gripper_status["ungrasp"]
                target_position = [
                    object_position_init[0],
                    object_position_init[1],
                    object_position_init[2]
                    + (max_z - min_z)
                    + self.tcp_height
                    + temp_height1,
                ]
            elif step >= 20 and step < 40:  # phase 2: grasp object
                gripper_value = gripper_status["grasp"]
                target_position = [
                    object_position_init[0],
                    object_position_init[1],
                    object_position_init[2]
                    + (max_z - min_z)
                    + self.tcp_height
                    + temp_height2,
                ]
            elif step >= 40 and step < 60:  # phase 3: move 20cm over goal position
                gripper_value = gripper_status["grasp"]
                target_position = [
                    object_goal_position[0],
                    object_goal_position[1],
                    object_goal_position[2]
                    + (max_z - min_z)
                    + self.tcp_height
                    + temp_height1,
                ]
            elif step >= 60 and step < 80:  # phase 4: move 5cm over goal position
                gripper_value = gripper_status["grasp"]
                target_position = [
                    object_goal_position[0],
                    object_goal_position[1],
                    object_goal_position[2]
                    + (max_z - min_z)
                    + self.tcp_height
                    + temp_height2,
                ]
            else:  # phase 5: drop object
                gripper_value = gripper_status["ungrasp"]

            if step < 80:
                self.move_end_effector_to_goal_position(
                    Pose(target_position, object_goal_orientation)
                )

            if gripper_value == 0 and self.gripper_id != None:
                p.removeConstraint(self.gripper_id, physicsClientId=self.client_id)
                self.gripper_id = None
                for _ in range(self.frequency):
                    p.stepSimulation(physicsClientId=self.client_id)

            if gripper_value == 1 and self.gripper_id == None:
                cube_orn = p.getQuaternionFromEuler([0, math.pi, 0])  # control rotation
                if self.tcp_link != -1:
                    self.gripper_id = p.createConstraint(
                        self.arm_id,
                        self.tcp_link,
                        object_id,
                        -1,
                        p.JOINT_FIXED,
                        [0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0],
                        childFrameOrientation=cube_orn,
                        physicsClientId=self.client_id,
                    )
                else:
                    self.gripper_id = p.createConstraint(
                        self.arm_id,
                        self.end_effector_index,
                        object_id,
                        -1,
                        p.JOINT_FIXED,
                        [0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0],
                        childFrameOrientation=cube_orn,
                        physicsClientId=self.client_id,
                    )

            p.stepSimulation(physicsClientId=self.client_id)

        print("-" * 20 + "\n" + "manipulation is done!")

    def pick_place_v2(self, object_init_pose, object_goal_pose):
        # TODO
        return None

    def pick_v1(self, object_id):
        # TODO
        return None

    def pick_v2(self, object_init_pose):
        # TODO
        return None

    def place(self, object_init_pose):
        # TODO
        return None
    
    # ----------------------------------------------------------------
    # functions for base
    # ----------------------------------------------------------------
    
    def get_base_id(self):
        """
        Retrieves the ID of the robot base.
        
        Returns:
            int: The ID of the robot base.
        """
        return self.base_id

    def get_base_pose(self):
        """
        Retrieves the current position and orientation of the robot base.

        Returns:
            Pose: A Pose object representing the current pose of the base.
                  The pose contains the position as a list [x, y, z] and the orientation as a list [roll, pitch, yaw].
        """
        base_position, base_orientation = p.getBasePositionAndOrientation(
            self.base_id, physicsClientId=self.client_id
        )
        base_orientation = p.getEulerFromQuaternion(
            base_orientation, physicsClientId=self.client_id
        )
        return Pose(base_position, base_orientation)

    def rotate_base(self, target_yaw, gradual=False, step_size=0.05, delay_time=0.05):
        """
        Rotate base to a specified yaw angle. Can be done gradually or at once.

        Args:
            target_yaw (float): The target yaw angle (in radians) for the base.
            gradual (bool): If True, the rotation is done gradually. Otherwise, it's instant.
            step_size (float, optional): Angle increment for each step in radians. Only used if gradual=True.
            delay_time (float, optional): Delay in seconds after each step. Only used if gradual=True.
        """

        def angle_to_quaternion(yaw):
            return [0, 0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]
        
        if gradual:
            while abs(self.current_yaw - target_yaw) > step_size:
                if target_yaw > self.current_yaw:
                    self.current_yaw = min(self.current_yaw + step_size, target_yaw)
                else:
                    self.current_yaw = max(self.current_yaw - step_size, target_yaw)

                orientation = angle_to_quaternion(self.current_yaw)
                position, _ = p.getBasePositionAndOrientation(
                    self.base_id, physicsClientId=self.client_id
                )
                p.resetBasePositionAndOrientation(
                    self.base_id, position, orientation, physicsClientId=self.client_id
                )

                if self.arm_id is not None:
                    self.sim_sync_base_arm_pose()

                p.stepSimulation(physicsClientId=self.client_id)
                # time.sleep(1.0 / self.frequency)

            # Ensure final orientation is set accurately
            orientation = angle_to_quaternion(target_yaw)
            position, _ = p.getBasePositionAndOrientation(
                self.base_id, physicsClientId=self.client_id
            )
            p.resetBasePositionAndOrientation(
                self.base_id, position, orientation, physicsClientId=self.client_id
            )
            p.stepSimulation(physicsClientId=self.client_id)
            # time.sleep(1.0 / self.frequency)

        else:
            orientation = angle_to_quaternion(target_yaw)
            position, _ = p.getBasePositionAndOrientation(
                self.base_id, physicsClientId=self.client_id
            )
            p.resetBasePositionAndOrientation(
                self.base_id, position, orientation, physicsClientId=self.client_id
            )

            if self.arm_id is not None:
                self.sim_sync_base_arm_pose()

            p.stepSimulation(physicsClientId=self.client_id)
    
    def check_collision_navigation(self):
        """
        Check if collision exists during navigation
        """
        aabb_base = self.client.get_bounding_box(self.base_id)
        for obstacle_id in self.client.obstacle_navigation_ids:
            aabb_obstacle = self.client.get_bounding_box(obstacle_id)
            if self.client.check_collision_xy(
                aabb_base, aabb_obstacle
            ):  # print("Collision detected during navigation, stopping...")
                return True
        return False
    
    def move_base_to_waypoint(self, waypoint):
        """
        Move base to waypoint
        The robot first rotates towards the target, and then moves towards it in a straight line.
        The movement is controlled by a controller (assumed to be a PID controller) that adjusts the velocity of the robot based on the distance to the target.

        Args:
            waypoint (Pose): The target pose (position and orientation) for the robot. This should be an instance of a Pose class, which is assumed to have 'x' and 'y' properties.
        """
        self.next_waypoint = waypoint
        self.target_distance = 0.0
        self.rotated = False

        while True:
            pose = self.get_base_pose()
            target = self.next_waypoint
            x, y = pose.x, pose.y

            distance = math.sqrt((target.y - y) ** 2 + (target.x - x) ** 2)
            yaw = math.atan2(target.y - y, target.x - x)

            self.distance_controller.set_goal(self.target_distance)
            output = self.distance_controller.calculate(distance)

            if not self.rotated:
                self.rotate_base(yaw)
                self.rotated = True

            THRESHOLD = 0.01
            if distance < THRESHOLD:
                output = 0.0
                pose = self.get_base_pose()
                break
            
            self.sim_action(-output)     
       
    def navigate_base(self, goal_base_pose, path, visualize = False):
        """
        Navigate a robot from its current position to a specified goal position

        Args:
            goal_base_pose (Pose): The target pose (poTruesition and orientation) for the robot.
        """
        
        # for i, waypoint in enumerate(path, start=1):
        for i in range(len(path)):
            
            next_point = [path[i][0], path[i][1], 0]
            # move to each waypoint
            self.move_base_to_waypoint(
                Pose([path[i][0], path[i][1], 0], goal_base_pose.orientation)
            )
            
            # draw the trajectory
            if i != 0 and visualize:
                front_point = [path[i-1][0], path[i-1][1], 0]
                p.addUserDebugLine(front_point, next_point, lineColorRGB=[1, 0, 0], lineWidth=3, physicsClientId=self.client_id)
            
            # self.visualizer.set_camera(self.base_id)
        
        self.rotate_base(goal_base_pose.yaw)
        print("-" * 20 + "\n" + "Navigation is done!")

    def get_standing_map(
        self,
        object_position,
        radius=1.0,
        resolution=0.03,
        x_max=10,
        y_max=10,
        enable_accurate_occupancy_map=True,
        enable_plot=True,
    ):
        """
        Find the grid cells that correspond to a circle centered around a given position.

        Args:
            object_position (list): The coordinates of the center of the circle [x, y].
            radius (float): The radius of the circle.
            resolution (float): The size of each cell in the grid.
            x_max (float): The maximum value for x coordinate in the grid.
            y_max (float): The maximum value for y coordinate in the grid.

        Returns:
            A list of grid cells that are inside the circle.
        """
        print("object_position:{}".format(object_position))
        # object_position = object_position[0:2]  # only care about x, y

        def to_grid_coordinates(point, resolution):
            return [
                int(round((coordinate + max_val) / resolution))
                for coordinate, max_val in zip(point, [x_max, y_max])
            ]

        # Defining the environment size (in meters) and resolution
        size_x = 2 * x_max
        size_y = 2 * y_max
        n_points_x = int(size_x / resolution)
        n_points_y = int(size_y / resolution)

        # Create a 2D grid representing the environment
        affordance_map = np.zeros((n_points_x, n_points_y))
        static_map = np.zeros((n_points_x, n_points_y))

        # get arm and base size in meters
        (
            min_x_base,
            min_y_base,
            _,
            max_x_base,
            max_y_base,
            _,
        ) = self.client.get_bounding_box(self.base_id)
        (
            min_x_arm,
            min_y_arm,
            _,
            max_x_arm,
            max_y_arm,
            _,
        ) = self.client.get_bounding_box(self.arm_id)
        robot_size = max(
            max_x_base - min_x_base,
            max_y_base - min_y_base,
            max_x_arm - min_x_arm,
            max_y_arm - min_y_arm,
        )
        # print("robot size:{}".format(robot_size))

        # Compute the number of grid cells to inflate around each obstacle
        inflate_cells = int(round(robot_size / 2 / resolution))

        center_grid = to_grid_coordinates(object_position, resolution)
        radius_grid = int(round(radius / resolution))

        # Mark the cells inside the circle as 1
        for i in range(
            max(0, center_grid[0] - radius_grid),
            min(n_points_x, center_grid[0] + radius_grid + 1),
        ):
            for j in range(
                max(0, center_grid[1] - radius_grid),
                min(n_points_y, center_grid[1] + radius_grid + 1),
            ):
                if (i - center_grid[0]) ** 2 + (
                    j - center_grid[1]
                ) ** 2 <= radius_grid**2:
                    affordance_map[i][j] = 1

        # Mark the cells occupied by objects as 1
        if not enable_accurate_occupancy_map:
            # get occupancy map
            for obstacle_id in self.client.obstacle_navigation_ids:
                aabb = self.client.get_bounding_box(obstacle_id)
                for i in range(
                    int((aabb[0] + x_max) / resolution),
                    int((aabb[3] + x_max) / resolution),
                ):
                    for j in range(
                        int((aabb[1] + y_max) / resolution),
                        int((aabb[4] + y_max) / resolution),
                    ):
                        static_map[i][j] = 1
        else:
            # get accurate occupancy map
            for obstacle_id in self.client.obstacle_navigation_ids:
                link_ids = [
                    i
                    for i in range(
                        -1, p.getNumJoints(obstacle_id, physicsClientId=self.client_id)
                    )
                ]
                for link_id in link_ids:
                    aabb_link = p.getAABB(
                        obstacle_id, link_id, physicsClientId=self.client_id
                    )
                    aabb_link = list(aabb_link[0] + aabb_link[1])
                    print(
                        "-" * 20
                        + "\n"
                        + "obstacle_id:{} link_id:{} aabb_link:{}".format(
                            obstacle_id, link_id, aabb_link
                        )
                    )
                    for i in range(
                        max(
                            0, int((aabb_link[0] + x_max) / resolution) - inflate_cells
                        ),
                        min(
                            n_points_x,
                            int((aabb_link[3] + x_max) / resolution) + inflate_cells,
                        ),
                    ):
                        for j in range(
                            max(
                                0,
                                int((aabb_link[1] + y_max) / resolution)
                                - inflate_cells,
                            ),
                            min(
                                n_points_y,
                                int((aabb_link[4] + y_max) / resolution)
                                + inflate_cells,
                            ),
                        ):
                            static_map[i][j] = 1

        # 1 in affordance_map, 0 in static_map
        standing_map = np.logical_and(affordance_map, np.logical_not(static_map))
        if enable_plot:
            plt.figure(figsize=(10, 5))

            plt.subplot(1, 2, 1)
            plt.imshow(affordance_map + 2 * static_map, cmap="gray", vmin=0, vmax=3)
            plt.title("Affordance Map + Static Map")

            plt.subplot(1, 2, 2)
            plt.imshow(standing_map, cmap="gray")
            plt.title("Standing Map")

            plt.show()

        return standing_map

    def compute_standing_position(
        self, object_position, standing_map, x_max=10, y_max=10, resolution=0.03
    ):
        """
        Compute the most suitable standing position from the standing map.

        The strategy is to select the standing position that minimizes the sum of the Euclidean distances to the object and the robot.

        Args:
            object_position (list): The coordinates of the center of the object [x, y].
            standing_map (numpy array): The computed standing map.

        Returns:
            The most suitable standing position [x, y].
        """
        # convert standing_map to a list of coordinates
        standing_positions = np.argwhere(standing_map)

        # if no standing positions available, return None
        if len(standing_positions) == 0:
            return None

        # convert grid coordinates to world coordinates
        def to_world_coordinates(point, resolution):
            return [
                coordinate * resolution - max_val
                for coordinate, max_val in zip(point, [x_max, y_max])
            ]

        # get robot position
        pose = self.get_base_pose()
        robot_position = [pose.x, pose.y]

        # compute the Euclidean distance between object_position and each standing position
        # and the distance between robot_position and each standing position
        object_distances = np.linalg.norm(
            standing_positions - object_position[:2], axis=1
        )
        robot_distances = np.linalg.norm(standing_positions - robot_position, axis=1)
        
        # find the standing position with the minimum sum of distances to the object and the robot
        total_distances = object_distances + robot_distances
        best_position = standing_positions[np.argmin(total_distances)]
        best_position_world = to_world_coordinates(best_position, resolution)

        # add z coordinate
        z = 0  # set to the height of the robot's base above the ground
        best_position_world_3d = [best_position_world[0], best_position_world[1], z]

        return best_position_world_3d


    def get_current_base_pose(self):
        """
        Retrieves the current pose (position and orientation) of the robot base.

        Returns:
            Pose: A Pose object representing the current pose of the base.
                The pose contains the position as a list [x, y, z] and the orientation as a list [roll, pitch, yaw].
        """
        base_position, base_orientation = p.getBasePositionAndOrientation(
            self.base_id, physicsClientId=self.client_id
        )
        base_orientation = p.getEulerFromQuaternion(base_orientation)
        return Pose(base_position, base_orientation)

    def base_stop(self):
        """
        Stops the movement of the robot base by setting its velocity to zero.

        Returns:
            bool: True if the base has been successfully stopped.
        """
        zero_velocity = [0, 0, 0]
        p.resetBaseVelocity(
            self.base_id, zero_velocity, zero_velocity, physicsClientId=self.client_id
        )
        return True
    
    # ----------------------------------------------------------------
    # functions for gripper
    # ----------------------------------------------------------------

    def sim_active_gripper(self, object_id, value):
        """
        Activate or deactivate the gripper.

        Args:
            object_id (init): ID of the object related to gripper action.
            value (int): 0 or 1, where 0 means deactivate (ungrasp) and 1 means activate (grasp).
        """

        gripper_status = {"ungrasp": 0, "grasp": 1}
        gripper_value = (
            gripper_status["grasp"] if value == 1 else gripper_status["ungrasp"]
        )

        if gripper_value == 0 and self.gripper_id != None:
            p.removeConstraint(self.gripper_id, physicsClientId=self.client_id)
            self.gripper_id = None
            for _ in range(self.frequency):
                p.stepSimulation(physicsClientId=self.client_id)
            print("-" * 20 + "\n" + "Gripper has been deactivated!")

        if gripper_value == 1 and self.gripper_id == None:
            cube_orn = p.getQuaternionFromEuler([0, math.pi, 0])  # control rotation
            if self.tcp_link != -1:
                self.gripper_id = p.createConstraint(
                    self.arm_id,
                    self.tcp_link,
                    object_id,
                    -1,
                    p.JOINT_FIXED,
                    [0, 0, 0],
                    [0, 0, 0],
                    [0, 0, 0],
                    childFrameOrientation=cube_orn,
                    physicsClientId=self.client_id,
                )
                print("-" * 20 + "\n" + "Gripper has been activated!")
            else:
                self.gripper_id = p.createConstraint(
                    self.arm_id,
                    self.end_effector_index,
                    object_id,
                    -1,
                    p.JOINT_FIXED,
                    [0, 0, 0],
                    [0, 0, 0],
                    [0, 0, 0],
                    childFrameOrientation=cube_orn,
                    physicsClientId=self.client_id,
                )
                print("-" * 20 + "\n" + "Gripper has been activated!")

        for _ in range(10):
            p.stepSimulation(physicsClientId=self.client_id)
            # time.sleep(1.0 / self.frequency)

    # ----------------------------------------------------------------
    # functions for camera
    # ----------------------------------------------------------------

    def set_camera(self, robot_id: int, width: int = 224, height: int = 224):
        # get current position and orientation for robot
        base_position, orientation = p.getBasePositionAndOrientation(
            robot_id, physicsClientId=self.client_id
        )

        # set camera position and target position
        base_position, orientation_quat = p.getBasePositionAndOrientation(
            self.base_id, physicsClientId=self.client_id
        )
        camera_position = np.array(base_position) + np.array([0, 0, 2.5])
        tilt = 3
        rot_mat = np.array(p.getMatrixFromQuaternion(orientation_quat)).reshape(3, 3)
        forward_vec = rot_mat @ np.array(
            [tilt, 0, 0]
        )  # Use the rotation matrix to rotate the vector
        target_position = np.array(base_position) + forward_vec
        view_mat = p.computeViewMatrix(
            cameraEyePosition=camera_position,
            cameraTargetPosition=target_position,
            cameraUpVector=[0, 1, 0],
            physicsClientId=self.client_id,
        )
        proj_mat = p.computeProjectionMatrixFOV(
            fov=60.0,  # field of view
            aspect=0.8,  # scale, default=1
            nearVal=0.01,  # view distance min
            farVal=10,
            physicsClientId=self.client_id,
        )
        # get image from the camera
        w, h, rgb, depth, seg = p.getCameraImage(
            width=width,
            height=height,
            viewMatrix=view_mat,
            projectionMatrix=proj_mat,
            physicsClientId=self.client_id,
        )
        return w, h, rgb, depth, seg

    def get_image(self, width, height):
        """
        Captures an image using the set camera settings.

        Args:
            width (int): The width of the image to capture.
            height (int): The height of the image to capture.

        Returns:
            tuple: A tuple containing the width, height, RGB image, depth image, and segmentation image.
        """
        return self.set_camera(self.base_id, width, height)

    def label_target_on_rgb_by_segmentation(
        self,
        rgb_img,
        seg_img,
        height=224,
        width=224,
        target_object_id=4,
        file_output_path="./outputs/target_rectangle.png",
    ):
        """
        Draws a rectangular box around the target object in the RGB image based on segmentation.

        Args:
            rgb_img (np.ndarray): The RGB image.
            seg_img (np.ndarray): The segmentation image.
            target_object_id (int): The ID of the target object in the segmentation image.
            file_output_path (str, optional): The path to save the output image. Default is './outputs/target_rectangle.png'.

        Returns:
            np.ndarray: The RGB image with a rectangular box drawn around the target object.
        """

        # get the target object's bounding box
        target_object_mask = np.where(seg_img == target_object_id, 255, 0).astype(
            np.uint8
        )
        contours, _ = cv2.findContours(
            target_object_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )
        # if the target object is found, draw a rectangular box around it
        if len(contours) != 0:
            x, y, w, h = cv2.boundingRect(contours[0])
            # draw a rectangular box around the target object
            cv2.rectangle(rgb_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # transfer the image to RGB from BGR
        rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2RGB)
        # save the image
        cv2.imwrite(file_output_path, rgb_img)
        return rgb_img