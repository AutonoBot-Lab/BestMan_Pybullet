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
import sys

# customized package
# current_path = os.path.abspath(__file__)
# utils_path = os.path.dirname(os.path.dirname(os.path.dirname(current_path)))
# if os.path.basename(utils_path) != "refactor":
#     raise ValueError('Not add the path of folder "utils", please check again!')
sys.path.append('/BestMan_Pybullet/refactor')

from .Pose import Pose
from Env.PbClient import PbClient
from Visualization.PbVisualizer import PbVisualizer
from Motion_Planning.Controller.PIDController import PIDController

class Bestman:
    def __init__(self, pb_client, robot_cfg):
        """
        Initialize a new object.

        Parameters:
            init_pos (list, optional): A list of three floats representing the initial position. Defaults to [0, 0, 0].
            pb_client (object): The pybullet client object.

        Attributes:
            pb_client (object): The pybullet client object.
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
            arm_joint_indexs (list): A list of joint indexes.
            arm_height (float): The height of the arm.
            end_effector_index (int): The index of the end effector.
            tcp_link (int): The tcp link index.
            tcp_height (float): The height of the tcp link.
            
            visualizer (object): The visualizer object.
            gripper_id (None): The gripper id. Initialized to None.
        """

        self.pb_client = pb_client
        self.client_id = self.pb_client.get_client()
        
        # Initialize PID controller
        controller_cfg = robot_cfg.Controller
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

        # Initialize base
        init_pose = Pose(robot_cfg.init_pose[:3], robot_cfg.init_pose[3:])
        self.base_id = p.loadURDF(
            fileName=robot_cfg.base_urdf_path,
            basePosition=init_pose.position,
            baseOrientation=p.getQuaternionFromEuler(init_pose.orientation),
            useFixedBase=True,
            physicsClientId=self.client_id,
        )

        # Initialize arm
        # filenames = {
        #     "ur5e": "/BestMan_Pybullet/refactor/Asset/mobile_manipulator/arm/ur5e/ur5e.urdf",
        #     "ur5e_vacuum": "/BestMan_Pybullet/refactor/Asset/mobile_manipulator/arm/ur5e/ur5e_vacuum.urdf",
        #     "ur5e_vacuum_long": "/BestMan_Pybullet/refactor/Asset/mobile_manipulator/arm/ur5e/ur5e_vacuum_long.urdf",
        #     "ur5_robotiq_85": "/BestMan_Pybullet/URDF_robot/model_elephant/urdf/ur5_robotiq_85.urdf",
        # }
        # filename = filenames["ur5e_vacuum_long"]

        self.arm_id = p.loadURDF(
            fileName=robot_cfg.arm_urdf_path,
            basePosition=init_pose.position,
            baseOrientation=p.getQuaternionFromEuler(init_pose.orientation),
            useFixedBase=True,
            physicsClientId=self.client_id,
        )
        self.arm_joint_indexs = robot_cfg.arm_joint_indexs
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
        self.sync_base_arm_pose()
        
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

        # Set arm and base colors
        self.visualizer = PbVisualizer(pb_client)
        self.visualizer.set_robot_visual_color(self.base_id, self.arm_id)

        # global parameters
        self.init_pos = init_pose   # Used when resetting the robot position
        self.gripper_id = None      # Constraints between the end effector and the grasped object, None when not grabbed

    # ----------------------------------------------------------------
    # functions for base and arm
    # ----------------------------------------------------------------
    
    def get_joint_link_info(self, robot_name):  # Print base or arm joint information
        if robot_name == "base":
            robot_id = self.base_id
        elif robot_name == "arm":
            robot_id = self.arm_id
        else:
            print(
                "unknown robot_name: {}, please input base or arm!".format(robot_name)
            )

        num_joints = p.getNumJoints(robot_id, physicsClientId=self.client_id)
        print("-" * 20 + "\n" + "Robot {} has {} joints".format(robot_id, num_joints))
        for i in range(num_joints):
            joint_info = p.getJointInfo(robot_id, i, physicsClientId=self.client_id)
            joint_name = joint_info[1]
            joint_state = p.getJointState(robot_id, i, physicsClientId=self.client_id)
            joint_angle = joint_state[0]
            link_name = joint_info[12].decode("UTF-8")
            print(
                "Joint index:{}, name:{}, angle:{}".format(i, joint_name, joint_angle)
            )
            print("Link index: {}, name: {}".format(i, link_name))

    def sync_base_arm_pose(self):
        """
        Synchronize the arm and base position
        """
        position, orientation = p.getBasePositionAndOrientation(
            self.base_id, physicsClientId=self.client_id
        )
        position = [position[0], position[1], self.arm_height]  # fixed height
        p.resetBasePositionAndOrientation(
            self.arm_id, position, orientation, physicsClientId=self.client_id
        )

    def action(self, output):
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
        self.sync_base_arm_pose()
        # time.sleep(1.0 / self.frequency)

    # ----------------------------------------------------------------
    # functions for base
    # ----------------------------------------------------------------
    
    def get_base_pose(self):
        """
        Retrieve current position and orientation of a specific object

        Returns:
            The function returns a Pose object representing the current pose of the base.
            The pose contains the position as a list [x, y, z] and the orientation as a list [roll, pitch, yaw].
        """
        base_position, base_orientation = p.getBasePositionAndOrientation(
            self.base_id, physicsClientId=self.client_id
        )
        base_orientation = p.getEulerFromQuaternion(
            base_orientation, physicsClientId=self.client_id
        )
        return Pose(base_position, base_orientation)

    def rotate_base(self, target_yaw, gradual=True, step_size=0.05, delay_time=0.05):
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
                    self.sync_base_arm_pose()

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
                self.sync_base_arm_pose()

            p.stepSimulation(physicsClientId=self.client_id)
    
    def check_collision_navigation(self):
        """
        Check if collision exists during navigation
        """
        aabb_base = self.pb_client.get_bounding_box(self.base_id)
        for obstacle_id in self.pb_client.obstacle_navigation_ids:
            aabb_obstacle = self.pb_client.get_bounding_box(obstacle_id)
            if self.pb_client.check_collision_xy(
                aabb_base, aabb_obstacle
            ):  # print("Collision detected during navigation, stopping...")
                return True
        return False
    
    def move_base_to_next_waypoint(self, next_waypoint):
        """
        Move base to next_waypoint
        The robot first rotates towards the target, and then moves towards it in a straight line.
        The movement is controlled by a controller (assumed to be a PID controller) that adjusts the velocity of the robot based on the distance to the target.

        Args:
            next_waypoint (Pose): The target pose (position and orientation) for the robot. This should be an instance of a Pose class, which is assumed to have 'x' and 'y' properties.
        """
        self.next_waypoint = next_waypoint
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

            self.action(-output)     
       
    def navigate_base(self, goal_base_pose, path):
        """
        Navigate a robot from its current position to a specified goal position

        Args:
            goal_base_pose (Pose): The target pose (poTruesition and orientation) for the robot.
        """
        # init_base_pose = self.get_base_pose()  # get current base pose
        # path = self.find_base_path(
        #     init_base_pose, goal_base_pose
        # )  # get a set of waypoints
        for waypoint in path:
            self.move_base_to_next_waypoint(
                Pose([waypoint[0], waypoint[1], 0], goal_base_pose.orientation)
            )  # move to each waypoint
        self.rotate_base(goal_base_pose.yaw)
        print("-" * 20 + "\n" + "Navigation is done!")

    # ----------------------------------------------------------------
    # functions for arm
    # ----------------------------------------------------------------
    
    def adjust_arm_height(self, height):
        # dynmaically adjust arm height
        self.arm_height = height
        self.pb_client.run(100)
        print("-" * 20 + "\n" + "Arm height has changed into {}".format(height))

    def get_arm_joint_angle(self):
        """
        Retrieve arm's joint angle
        """
        joint_angles = []
        for i in range(6):
            joint_state = p.getJointState(
                self.arm_id, i, physicsClientId=self.client_id
            )
            joint_angle = joint_state[0]
            joint_angles.append(joint_angle)  # Add the current joint angle to the list
        print(
            "Joint angles (only arm and not include ee_fixed_joint):{}".format(
                joint_angles
            )
        )
        return joint_angles  # Return the list of joint angles
    
    def set_arm_to_joint_angles(self, joint_angles):
        """
        Set arm to move to a specific set of joint angles, witout considering physics

        Args:
            joint_angles: A list of desired joint angles (in radians) for each joint of the arm.
        """
        for joint_index in range(6):
            p.resetJointState(
                bodyUniqueId=self.arm_id,
                jointIndex=joint_index,
                targetValue=joint_angles[joint_index],
                physicsClientId=self.client_id,
            )
        p.stepSimulation(physicsClientId=self.client_id)
        # time.sleep(1.0 / self.frequency)
            
    def debug_set_joint_values(self):
        """
        Manually set each joint value of the arm for debugging purposes.
        """
        joint_angles = self.get_arm_joint_angle()
        print("Current joint angles: {}".format(joint_angles))

        for i in range(6):
            joint_value = input(
                "Enter value for joint {} (current value: {}) or 'skip' to keep current value: ".format(
                    i, joint_angles[i]
                )
            )
            if joint_value.lower() == "q":
                print("Skipping joint {}".format(i))
                continue
            try:
                joint_angles[i] = float(joint_value)
            except ValueError:
                print("Invalid input. Keeping current value for joint {}.".format(i))

        self.set_arm_to_joint_angles(joint_angles)
        print("Updated joint angles: {}".format(joint_angles))

    def move_arm_to_joint_angles(self, joint_angles):
        """
        Move arm to move to a specific set of joint angles, with considering physics

        Args:
            joint_angles: A list of desired joint angles (in radians) for each joint of the arm.
        """
        for joint_index in range(6):
            p.setJointMotorControl2(
                bodyUniqueId=self.arm_id,
                jointIndex=joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_angles[joint_index],
                force=self.max_force,
                physicsClientId=self.client_id,
            )

        start_time = time.time()  # avoid time anomaly

        while True:
            p.stepSimulation(physicsClientId=self.client_id)

            # Check if reach the goal
            current_angles = [
                p.getJointState(self.arm_id, i, physicsClientId=self.client_id)[0]
                for i in range(6)
            ]
            diff_angles = [abs(a - b) for a, b in zip(joint_angles, current_angles)]
            if all(diff < self.threshold for diff in diff_angles):
                # print(
                #     "-" * 20
                #     + "\n"
                #     + "Reached target joint position:{}".format(joint_angles)
                # )
                break

            if time.time() - start_time > self.timeout:  # avoid time anomaly
                print(
                    "-" * 20 + "\n" + "Timeout before reaching target joint position."
                )
                break
    
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

    def get_end_effector_info(self):
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
    
    def joints_to_cartesian(self, joint_angles):
        """
        Transform from arm's joint angles to its Cartesian coordinates
        """
        self.set_arm_to_joint_angles(joint_angles)
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
        Transform from arm's Cartesian coordinates to its joint angles
        """
        joint_angles = p.calculateInverseKinematics(
            bodyUniqueId=self.arm_id,
            endEffectorLinkIndex=self.end_effector_index,
            targetPosition=position,
            targetOrientation=p.getQuaternionFromEuler(orientation),
            maxNumIterations=self.max_iterations,
            residualThreshold=self.threshold,
            physicsClientId=self.client_id,
        )
        return joint_angles

    def rotate_end_effector(self, angle):
        """
        Rotate the end effector of the robot arm by a specified angle.

        Args:
            angle (float): The desired rotation angle in radians.
        """
        # Get the current joint states
        joint_indices = [i for i in range(6)]

        # Change this to your actual joint indices
        joint_states = p.getJointStates(
            self.arm_id, joint_indices, physicsClientId=self.client_id
        )

        # Create a new list of target joint angles
        target_joint_angles = [joint_state[0] for joint_state in joint_states]

        # Add desired rotation to the last joint's current angle
        target_joint_angles[-1] += angle

        # Set the target angles
        self.set_arm_to_joint_angles(target_joint_angles)

        # Step the simulation until the joints reach their target angles
        while True:
            # Update the current joint states
            joint_states = p.getJointStates(
                self.arm_id, joint_indices, physicsClientId=self.client_id
            )
            current_joint_angles = [joint_state[0] for joint_state in joint_states]

            # Check if all joints have reached their target angles
            if all(
                abs(current_joint_angles[i] - target_joint_angles[i]) < 0.01
                for i in range(6)
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
            joint_angles = self.cartesian_to_joints(target_position, target_orientation)

            # If IK solution is invalid, break this loop and start a new attempt
            if joint_angles is None or len(joint_angles) != 6:
                break

            # Test the calculated angles for collision
            for joint_index, joint_angle in enumerate(joint_angles):
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
                    self.move_arm_to_joint_angles(joint_angles)
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
    
    
    
    # use a simple method to compute standing map
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
        ) = self.pb_client.get_bounding_box(self.base_id)
        (
            min_x_arm,
            min_y_arm,
            _,
            max_x_arm,
            max_y_arm,
            _,
        ) = self.pb_client.get_bounding_box(self.arm_id)
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
            for obstacle_id in self.pb_client.obstacle_navigation_ids:
                aabb = self.pb_client.get_bounding_box(obstacle_id)
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
            for obstacle_id in self.pb_client.obstacle_navigation_ids:
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

    # use a simple method to compute standing map
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

    def pick_place(self, object_id, object_goal_position, object_goal_orientation):
        """
        Perform pick-and-place manipulation of an object using the robot arm.

        Args:
            object_id: The ID of the target object to be manipulated.
            object_goal_position: The goal position for the target object.
            object_goal_orientation: The goal orientation for the target object.
        """

        # get target object position
        object_position_init, orientation = p.getBasePositionAndOrientation(
            object_id, physicsClientId=self.client_id
        )
        # consider the object's height
        _, _, min_z, _, _, max_z = self.pb_client.get_bounding_box(object_id)

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

    def active_gripper(self, object_id, value):
        """
        Activate or deactivate the gripper.

        Args:
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

    def calculate_IK_error(self, goal_position, goal_orientation):
        """
        Calculate the inverse kinematics (IK) error for performing pick-and-place manipulation of an object using a robot arm.

        Args:
            goal_position: The desired goal position for the target object.
            goal_orientation: The desired goal orientation for the target object.
        """
        joint_angles = p.calculateInverseKinematics(
            bodyUniqueId=self.arm_id,
            endEffectorLinkIndex=self.end_effector_index,
            targetPosition=goal_position,
            targetOrientation=p.getQuaternionFromEuler(goal_orientation),
            maxNumIterations=self.max_iterations,
            residualThreshold=self.threshold,
            physicsClientId=self.client_id,
        )
        print("-" * 20 + "\n" + "IK: joint_angles is {}".format(joint_angles))

        self.move_arm_to_joint_angles(joint_angles)

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

    # ----------------------------------------------------------------
    # For RL algorithm
    # ----------------------------------------------------------------

    def reset_pos(self, seed=None, pos=None):
        """
        Reset the robot to initial state when the pose is not given
        """
        if pos is None:
            # reset the base and arm to initial state when the pose is not given
            p.resetBasePositionAndOrientation(
                self.base_id,
                self.init_pos.position,
                p.getQuaternionFromEuler([0, 0, self.init_pos.yaw]),
                physicsClientId=self.client_id,
            ),
            # BUG: need to reset the arm separately or just call the sync_base_arm_pose?
            self.sync_base_arm_pose()
        else:
            p.resetBasePositionAndOrientation(
                self.base_id,
                pos.position,
                p.getQuaternionFromEuler([0, 0, pos.yaw]),
                physicsClientId=self.client_id,
            )
            self.sync_base_arm_pose()

    def get_current_pos(self):
        """
        Get the current position of the robot
        """
        base_position, base_orientation = p.getBasePositionAndOrientation(
            self.base_id, physicsClientId=self.client_id
        )
        base_orientation = p.getEulerFromQuaternion(base_orientation)
        return Pose(base_position, base_orientation)

    def set_camera(
        self,
        robot_id: int,
        width: int = 224,
        height: int = 224,
    ):
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
        This function draws a rectangular box around the target object in the RGB image.

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

    def forward(self, distance=0.25):  # distance is given in meters
        # get the current pose
        current_pose = self.get_base_pose()
        current_position = current_pose.position
        current_orientation = current_pose.orientation

        # get the current yaw angle
        _, _, yaw = current_orientation

        # calculate the new position
        new_position = [
            current_position[0] + distance * math.cos(yaw),
            current_position[1] + distance * math.sin(yaw),
            current_position[2],
        ]

        # set the new position as a goal
        goal_pose = Pose(new_position, current_orientation)

        # move to the new position
        self.move_base_to_next_waypoint(goal_pose)

        result = self.check_collision_navigation()  # check if collision exists
        if result:
            print("Fail to move forward without collision.")
            return False
        print("Successfully moved forward without collision.")
        return True

    def rotate_left(self, degree=30):  # degree is given in degrees
        # Convert the degree to radians
        radian = math.radians(degree)

        # Get the current pose
        current_pose = self.get_base_pose()
        current_orientation = current_pose.orientation

        # Get the current yaw angle
        _, _, yaw = current_orientation

        # Calculate the new yaw angle
        new_yaw = yaw + radian  # Add radian to current yaw for a left turn

        # Ensure the yaw is within -pi to pi
        new_yaw = (new_yaw + math.pi) % (2 * math.pi) - math.pi

        # Rotate to the new yaw angle
        self.rotate_base(new_yaw)

        result = self.check_collision_navigation()  # check if collision exists
        if result:
            print("Fail to rotate left without collision.")
            return False
        print("Successfully rotate left without collision.")
        return True

    def rotate_right(self, degree=30):  # degree is given in degrees
        # Convert the degree to radians
        radian = math.radians(degree)

        # Get the current pose
        current_pose = self.get_base_pose()
        current_orientation = current_pose.orientation

        # Get the current yaw angle
        _, _, yaw = current_orientation

        # Calculate the new yaw angle
        new_yaw = yaw - radian  # Subtract radian from current yaw for a right turn

        # Ensure the yaw is within -pi to pi
        new_yaw = (new_yaw + math.pi) % (2 * math.pi) - math.pi

        # Rotate to the new yaw angle
        self.rotate_base(new_yaw)

        result = self.check_collision_navigation()  # check if collision exists
        if result:
            print("Fail to rotate right without collision.")
            return False
        print("Successfully rotate right without collision.")
        return True

    def stop(self):
        zero_velocity = [0, 0, 0]
        p.resetBaseVelocity(
            self.base_id, zero_velocity, zero_velocity, physicsClientId=self.client_id
        )
        return True

    def grasp(self, object_id):
        # get target object position
        object_position_init, orientation = p.getBasePositionAndOrientation(
            object_id, physicsClientId=self.client_id
        )

        # orientation = [0.0, math.pi / 2.0, 0.0]  # vertial downward grip
        orientation = [0.0, math.pi, 0.0]  # vertial downward grip

        target_position = [
            object_position_init[0],
            object_position_init[1],
            object_position_init[2] + 0.1,
        ]

        gripper_status = {"ungrasp": 0, "grasp": 1}
        gripper_value = gripper_status["ungrasp"]
        for step in range(100):
            if step >= 20:
                target_position = [
                    object_position_init[0],
                    object_position_init[1],
                    object_position_init[2] + 0.1,
                ]  # grab object
                gripper_value = gripper_status["grasp"]

            result = self.move_end_effector_to_goal_position(
                Pose(target_position, orientation)
            )
            if not result:
                print("-" * 20 + "\n" + "grasping is unsuccessful!")
                return False

            if gripper_value == 0 and self.gripper_id != None:
                p.removeConstraint(self.gripper_id, physicsClientId=self.client_id)
                self.gripper_id = None
            if gripper_value == 1 and self.gripper_id == None:
                cube_orn = p.getQuaternionFromEuler([0, math.pi, 0])
                self.gripper_id = p.createConstraint(
                    self.arm_id,
                    self.end_effector_index,
                    object_id,
                    -1,
                    p.JOINT_FIXED,
                    [0, 0, 0],
                    [0, 0, 0.05],
                    [0, 0, 0],
                    childFrameOrientation=cube_orn,
                    physicsClientId=self.client_id,
                )

            p.stepSimulation(physicsClientId=self.client_id)

        # check if the object has been moved
        object_position_after, _ = p.getBasePositionAndOrientation(object_id)
        if object_position_init != object_position_after:
            print("-" * 20 + "\n" + "grasping is successful!")
            return True
        else:
            print("-" * 20 + "\n" + "grasping is unsuccessful!")
            return False

    def set_envs(self):
        # load table
        table_id = self.pb_client.load_object(
            "./URDF_models/furniture_table_rectangle_high/table.urdf",
            [1.0, 1.0, 0.0],
            [0.0, 0.0, 0.0],
            1.0,
            "table",
        )

        base_boundary = self.pb_client.get_bounding_box(self.base_id)
        arm_boundary = self.pb_client.get_bounding_box(self.arm_id)
        table_boundary = self.pb_client.get_bounding_box(table_id)

        obstacle_boundary_list = []  # store obstacles
        obstacle_boundary_list.append(base_boundary)
        obstacle_boundary_list.append(arm_boundary)
        obstacle_boundary_list.append(table_boundary)

        # generate random position for bowl within the table boundary
        table_min_x, table_min_y, table_min_z, table_max_x, table_max_y, table_max_z = (
            table_boundary[0],
            table_boundary[1],
            table_boundary[2],
            table_boundary[3],
            table_boundary[4],
            table_boundary[5],
        )
        # print('debug: table_boundary:{}'.format(table_boundary))

        bowl_x = random.uniform(table_min_x, table_max_x)
        bowl_y = random.uniform(table_min_y, table_max_y)
        bowl_id = self.pb_client.load_object(
            "./URDF_models/utensil_bowl_blue/model.urdf",
            [bowl_x, bowl_y, table_max_z + 0.05],  # bowl is on the table
            [0.0, 0.0, 0.0],
            1.0,
            "bowl",
        )

        # initialize chair_id list
        chair_ids = []

        # number of chairs to be loaded, random between 1 and 3
        num_chairs = random.randint(3, 5)

        for _ in range(num_chairs):
            collision_flag = True
            max_attempts = 200

            for attempt in range(max_attempts):
                chair_x, chair_y = self.pb_client.generate_point_outside_area(
                    table_min_x, table_min_y, table_max_x, table_max_y
                )
                # print('chair_x:{}, chair_y:{}'.format(chair_x, chair_y))
                chair_size = 0.3
                chair_boundary = [
                    chair_x - chair_size,
                    chair_y - chair_size,
                    0.1,
                    chair_x + chair_size,
                    chair_y + chair_size,
                    1.0,
                ]

                # if the chair position is collision with other obstacle, break the loop
                collision_flag = any(
                    self.pb_client.check_collision_xy(chair_boundary, obstacle)
                    for obstacle in obstacle_boundary_list
                )

                if not collision_flag:
                    chair_angle = random.uniform(0, 2 * math.pi)
                    chair_z = 0.1  # assuming the chair is on the floor
                    chair_id = self.pb_client.load_object(
                        "./URDF_models/furniture_chair/model.urdf",
                        [chair_x, chair_y, chair_z],
                        [math.pi / 2.0 * 3, 0.0, chair_angle],
                        1.5,
                        "chair",
                    )
                    obstacle_boundary_list.append(chair_boundary)
                    chair_ids.append(chair_id)
                    break

            if collision_flag and attempt == max_attempts - 1:
                print("-" * 20 + "\n" + "Failed to place chair after maximum attempts!")

        # return IDs of all loaded objects
        self.pb_client.run(100)
        return {"table": table_id, "bowl": bowl_id, "chairs": chair_ids}
