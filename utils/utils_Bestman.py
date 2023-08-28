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

from utils.utils_PbVisualizer import PbVisualizer
from utils.utils_PbClient import PbClient
from utils.utils_PIDController import PIDController

"""
Pose definition
"""

class Pose:
    # [x, y, z]: a point in a Cartesian coordinate system
    # [roll, pitch, yaw]: the orientation or heading of the robot
    def __init__(self, position_array, orientation_array):
        self.position = position_array
        self.x = position_array[0]
        self.y = position_array[1]
        self.z = position_array[2]

        self.orientation = orientation_array
        self.roll = orientation_array[0]
        self.pitch = orientation_array[1]
        self.yaw = orientation_array[2]


"""
Bestman API:
Segbot has 7 joints
Joint:0 and its name is: 'base_footprint_joint'
Joint:1 and its name is: 'base_link_to_cylinder_joint'
Joint:2 and its name is: 'cylinder_to_plate_joint'
Joint:3 and its name is: 'base_caster_support_joint'
Joint:4 and its name is: 'caster_wheel_joint'
Joint:5 and its name is: 'left_wheel_joint'
Joint:6 and its name is: 'right_wheel_joint'

UR5e has 7 joints
Joint:0 and its name is: 'shoulder_pan_joint'
Joint:1 and its name is: 'shoulder_lift_joint'
Joint:2 and its name is: 'elbow_joint'
Joint:3 and its name is: 'wrist_1_joint'
Joint:4 and its name is: 'wrist_2_joint'
Joint:5 and its name is: 'wrist_3_joint'
Joint:6 and its name is: 'ee_fixed_joint'
"""


class Bestman:
    def __init__(self, init_pos, pb_client):
        self.pb_client = pb_client
        self.client_id = self.pb_client.get_client()
        # parameters for base
        self.frequency = 240  # simulation step for base and arm
        self.timeout = 100.0  # maximum time for planning
        # parameters for arm
        self.max_force = 500
        self.max_iterations = 10000
        self.threshold = 0.01
        self.max_attempts = 500

        # Initialize PID controller
        self.target_distance = 0.0
        self.controller = PIDController(Kp=0.01, Ki=0.0, Kd=0.0, setpoint=self.target_distance)
        self.rotated = False

        # Initialize base
        self.base_id = p.loadURDF(
            fileName="./URDF_robot/segbot.urdf",
            basePosition=init_pos.position,
            baseOrientation=p.getQuaternionFromEuler([0, 0, init_pos.yaw]),
            physicsClientId=self.client_id
        )

        # Initialize arm

        """
        ur5 arm
        """
        # Joint index:0, name:b'shoulder_pan_joint', angle:0.0
        # Joint index:1, name:b'shoulder_lift_joint', angle:0.0
        # Joint index:2, name:b'elbow_joint', angle:0.0
        # Joint index:3, name:b'wrist_1_joint', angle:0.0
        # Joint index:4, name:b'wrist_2_joint', angle:0.0
        # Joint index:5, name:b'wrist_3_joint', angle:0.0
        
        """
        ur5 arm's virutal joint
        """
        # Joint index:6, name:b'ee_fixed_joint', angle:0.0

        """
        tcp: 2f-85's joint
        """
        # Joint index:7, name:b'ee_link_gripper_base_joint', angle:0.0
        # Joint index:8, name:b'gripper_right_spring_link_joint', angle:0.0
        # Joint index:9, name:b'gripper_left_spring_link_joint', angle:0.0
        # Joint index:10, name:b'gripper_right_driver_joint', angle:0.0
        # Joint index:11, name:b'gripper_right_coupler_joint', angle:0.0
        # Joint index:12, name:b'gripper_right_follower_joint', angle:0.0
        # Joint index:13, name:b'gripper_right_pad_joint', angle:0.0
        # Joint index:14, name:b'gripper_left_driver_joint', angle:0.0
        # Joint index:15, name:b'gripper_left_coupler_joint', angle:0.0
        # Joint index:16, name:b'gripper_left_follower_joint', angle:0.0
        # Joint index:17, name:b'gripper_left_pad_joint', angle:0.0
        # Joint index:18, name:b'tool_center_point', angle:0.0

        """
        tcp: vacuum's joint
        """
        # Joint index:7, name:b'ee_link_gripper_base_joint', angle:0.0
        # Joint index:8, name:b'tool_center_point', angle:0.0

        # differenciate arm and tcp
        self.joint_indexs = [0, 1, 2, 3, 4, 5]
        self.end_effector_index = 6

        filenames = {
            "ur5e":"./URDF_robot/ur5e.urdf",
            "ur5e_2f85":"./URDF_robot/ur5e_2f85.urdf",
            "ur5e_vacuum":"./URDF_robot/ur5e_vacuum.urdf",
        }
        filename = filenames["ur5e_vacuum"]
        print("-" * 20 + "\n" + "Arm model: {}".format(filename))
        self.arm_id = p.loadURDF(
            fileName=filename,
            basePosition=init_pos.position,
            baseOrientation=p.getQuaternionFromEuler([0.0, 0.0, math.pi / 2.0]),
            useFixedBase=True,
            physicsClientId=self.client_id
        )

        # get tcp link
        if filename.endswith('ur5e.urdf'):
            self.tcp_link = -1
            self.tcp_height = 0
        elif filename.endswith('ur5e_vacuum.urdf'):
            self.tcp_link = 8
            self.tcp_height = 0.065 # real value is 0.061
        elif filename.endswith('ur5e_2f85.urdf'):
            self.tcp_link = 18
            self.tcp_height = 0 # TODO
        else:
            print("Unknown tcp link: {}")
            sys.exit()

        # Add constraint between base and arm
        robot2_start_pos = [0, 0, 0]
        fixed_joint = p.createConstraint(
            self.base_id,
            -1,
            self.arm_id,
            -1,
            p.JOINT_FIXED,
            [0, 0, 0],
            [0, 0, 1.3],
            robot2_start_pos,
            physicsClientId=self.client_id
        )

        self.init_pos = init_pos
        self.gripper_id = None
        self.sync_base_arm_pose()

        # Set arm and base colors
        self.visualizer = PbVisualizer(pb_client)
        self.visualizer.set_arm_visual_color(self.base_id, self.arm_id)

    # ----------------------------------------------------------------
    # Segbot Navigation
    # ----------------------------------------------------------------

    """
    This function retrieves and prints the information about all joints of a specific object in the PyBullet simulation environment.

    Prints:
        The function first prints the total number of joints in the base model, and then it prints the index and name of each joint.
    """

    def get_base_joint_info(self):
        num_joints = p.getNumJoints(self.base_id, physicsClientId=self.client_id)
        print("-" * 20 + "\n" + "Segbot has {} joints".format(num_joints))
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.base_id, i, physicsClientId=self.client_id)
            joint_name = joint_info[1]
            joint_state = p.getJointState(self.base_id, i, physicsClientId=self.client_id)
            joint_angle = joint_state[0]
            print(
                "Joint index:{}, name:{}, angle:{}".format(i, joint_name, joint_angle)
            )

    def get_link_names(self):
        num_joints = p.getNumJoints(self.base_id, physicsClientId=self.client_id)
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.base_id, i, physicsClientId=self.client_id)
            link_name = joint_info[12].decode("UTF-8")
            print("Link index: {}, name: {}".format(i, link_name))

    """
    This function navigates a robot from its current position to a specified goal position. 
    The navigation is performed by finding a path from the current position to the goal.
    And then move along this path. 
    The robot's orientation is adjusted at each waypoint and finally set to the desired yaw angle at the goal.

    Args:
        goal_base_pose (Pose): The target pose (position and orientation) for the robot. 
    """

    def navigate_base(self, goal_base_pose):
        init_base_pose = self.get_base_pose()  # get current base pose
        path = self.find_base_path(
            init_base_pose, goal_base_pose
        )  # get a set of waypoints
        for waypoint in path:
            self.move_base_to_next_waypoint(
                Pose([waypoint[0], waypoint[1], 0], goal_base_pose.orientation)
            )  # move to each waypoint
        self.rotate_base(goal_base_pose.yaw)
        print("-" * 20 + "\n" + "navigation is done!")

    """
    This function retrieves the current position and orientation of a specific object

    Returns:
        The function returns a Pose object representing the current pose of the base. 
        The pose contains the position as a list [x, y, z] and the orientation as a list [roll, pitch, yaw].
    """

    def get_base_pose(self):
        base_position, base_orientation = p.getBasePositionAndOrientation(self.base_id, physicsClientId=self.client_id)
        base_orientation = p.getEulerFromQuaternion(base_orientation, physicsClientId=self.client_id)
        return Pose(base_position, base_orientation)

    """
    This function finds a path from a specified initial position to a goal position in a 2D grid representation.

    Args:
        init_base_position (Pose): The initial position of the robot.
        goal_base_position (Pose): The goal position of the robot.

    Returns:
        The function returns a list of waypoints that form a path from the initial position to the goal position. 
        Each waypoint is a list [x, y] representing a position in the world coordinates.
    
    Note:
        A* for path searching in map: 10 meter * 10 meter
        resolution is 0.1 meter
        a 2D grid: 1 for obstacles
    """

    def find_base_path(self, init_base_position, goal_base_position):
        init_base_position = init_base_position.position[0:2]  # only care about x, y
        goal_base_position = goal_base_position.position[0:2]

        def to_grid_coordinates(point, resolution):
            return [int(round(coordinate / resolution)) for coordinate in point]

        def to_world_coordinates(point, resolution):
            return [coordinate * resolution for coordinate in point]

        # Defining the environment size (in meters) and resolution
        size = 10
        resolution = 0.1
        n_points = int(size / resolution)

        # Create a 2D grid representing the environment
        static_map = np.zeros((n_points, n_points))

        # Add obstacles to the map
        for obstacle_id in self.pb_client.obstacle_navigation_ids:
            aabb = self.pb_client.get_bounding_box(obstacle_id)
            for i in range(int(aabb[0] / resolution), int(aabb[3] / resolution)):
                for j in range(int(aabb[1] / resolution), int(aabb[4] / resolution)):
                    static_map[i][j] = 1

        # plt.imshow(static_map, cmap='gray_r')  # 'gray_r' means reversed grayscale
        # plt.show()

        init_base_position = to_grid_coordinates(
            init_base_position, resolution
        )  # In grid coordinates
        goal_base_position = to_grid_coordinates(
            goal_base_position, resolution
        )  # In grid coordinates

        # Make sure the positions are within the environment and not on the table
        assert (
            0 <= init_base_position[0] < n_points
            and 0 <= init_base_position[1] < n_points
        ), "Initial base position is out of boundary!"
        assert (
            0 <= goal_base_position[0] < n_points
            and 0 <= goal_base_position[1] < n_points
        ), "Goal base position is out of boundary!"
        assert (
            static_map[init_base_position[0], init_base_position[1]] != 1
        ), "Initial base position is in an obstacle!"
        assert (
            static_map[goal_base_position[0], goal_base_position[1]] != 1
        ), "Goal base position is in an obstacle!"

        # Convert the numpy grid map to NetworkX graph
        graph = nx.grid_2d_graph(*static_map.shape)
        for i in range(n_points):
            for j in range(n_points):
                if static_map[i, j] == 1:
                    graph.remove_node((i, j))

        path = nx.astar_path(
            graph, tuple(init_base_position), tuple(goal_base_position)
        )

        # Convert grid coordinates back to world coordinates
        path = [to_world_coordinates(point, resolution) for point in path]
        # print('raw path:{}'.format(path))
        return path

    """
    This function moves the base of a robot to the next waypoint (next_waypoint). 
    The robot first rotates towards the target, and then moves towards it in a straight line. 
    The movement is controlled by a controller (assumed to be a PID controller) that adjusts the velocity of the robot based on the distance to the target.

    Args:
        next_waypoint (Pose): The target pose (position and orientation) for the robot. This should be an instance of a Pose class, which is assumed to have 'x' and 'y' properties.
    """

    def move_base_to_next_waypoint(self, next_waypoint):
        self.next_waypoint = next_waypoint
        self.target_distance = 0.0
        self.rotated = False
        while True:
            pose = self.get_base_pose()
            target = self.next_waypoint
            x, y = pose.x, pose.y

            distance = math.sqrt((x - target.x) ** 2 + (y - target.y) ** 2)
            yaw = math.atan2(target.y - y, target.x - x)

            self.controller.set_goal(self.target_distance)
            output = self.controller.calculate(distance)

            if not self.rotated:
                # print('Step into orientation changing')
                self.rotate_base(yaw)
                # print('Reorienting finished!')
                self.rotated = True

            THRESHOLD = 0.01
            if distance < THRESHOLD:
                output = 0.0
                # print('Reach goal: {}!'.format(self.next_waypoint.position))
                pose = self.get_base_pose()
                break

            self.action(-output)

    """
    This function rotates the base of a robot to a specified yaw angle. 

    Args:
        base_yaw (float): The target yaw angle (in radians) for the base.
    """

    def rotate_base(self, base_yaw):
        # Convert a Euler yaw angle (in radians) to Quaternion.
        def angle_to_quaternion(base_yaw):
            return [0, 0, math.sin(base_yaw / 2.0), math.cos(base_yaw / 2.0)]

        position, orientation = p.getBasePositionAndOrientation(self.base_id, physicsClientId=self.client_id)
        orientation = angle_to_quaternion(base_yaw)
        p.resetBasePositionAndOrientation(self.base_id, position, orientation, physicsClientId=self.client_id)

        if self.arm_id is not None:
            self.sync_base_arm_pose()
        p.stepSimulation(physicsClientId=self.client_id)
        time.sleep(1.0 / self.frequency)

    """
    This function uses the output of the PID controller to adjust the position of the robot's base.

    Args:
        output (float): The output of the PID controller, which is used to calculate the new position of the robot's base.
    """

    def action(self, output):
        position, orientation = p.getBasePositionAndOrientation(self.base_id, physicsClientId=self.client_id)
        euler_angles = p.getEulerFromQuaternion(orientation, physicsClientId=self.client_id)

        delta_x = output * math.cos(euler_angles[2])
        delta_y = output * math.sin(euler_angles[2])

        target_position = [position[0] + delta_x, position[1] + delta_y, position[2]]
        target_orientation = orientation

        result = self.check_collision_navigation()  # check if collision exists
        # if result:
        #     assert "Collision detected during navigation, stopping..."

        p.resetBasePositionAndOrientation(
            self.base_id, target_position, target_orientation, physicsClientId=self.client_id
        )
        self.sync_base_arm_pose()
        # time.sleep(1.0 / self.frequency)

    """
    This function synchronizes the pose (position and orientation) of the robot's arm with its base.
    """

    def sync_base_arm_pose(self):
        position, orientation = p.getBasePositionAndOrientation(self.base_id, physicsClientId=self.client_id)
        position = [position[0], position[1], 1.02]  # fixed height
        p.resetBasePositionAndOrientation(self.arm_id, position, orientation, physicsClientId=self.client_id)

    """
    This function check if collision exists during navigation
    """

    def check_collision_navigation(self):
        for obstacle_id in self.pb_client.obstacle_navigation_ids:
            aabb_base = self.pb_client.get_bounding_box(self.base_id)
            aabb_obstacle = self.pb_client.get_bounding_box(obstacle_id)
            if self.pb_client.check_collision_xy(
                aabb_base, aabb_obstacle
            ):  # print("Collision detected during navigation, stopping...")
                return True
        return False

    # ----------------------------------------------------------------
    # Arm Manipulation
    # ----------------------------------------------------------------

    """
    This function retrieves and prints information about the joints of the robot's arm.
    """

    def get_arm_joint_info(self):
        num_joints = p.getNumJoints(self.arm_id, physicsClientId=self.client_id)
        print("-" * 20 + "\n" + "UR5e has {} joints".format(num_joints))
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.arm_id, i, physicsClientId=self.client_id)
            joint_name = joint_info[1]
            joint_state = p.getJointState(self.arm_id, i, physicsClientId=self.client_id)
            joint_angle = joint_state[0]
            print(
                "Joint index:{}, name:{}, angle:{}".format(i, joint_name, joint_angle)
            )

    """
    This function retrieves and return joints' angle.
    """

    def get_arm_joint_angle(self):
        joint_angles = []  # Initialize an empty list to store the joint angles
        for i in range(6):
            joint_state = p.getJointState(self.arm_id, i, physicsClientId=self.client_id)
            joint_angle = joint_state[0]
            joint_angles.append(joint_angle)  # Add the current joint angle to the list
        print("Joint angles (only arm and not include ee_fixed_joint):{}".format(joint_angles))
        return joint_angles  # Return the list of joint angles

    """
    This function commands the robot's arm to move to a specific set of joint angles.

    Args:
        joint_angles: A list of desired joint angles (in radians) for each joint of the arm.
    """

    def set_arm_to_joint_angles(self, joint_angles):
        for joint_index in range(6):
            p.resetJointState( 
                bodyUniqueId=self.arm_id,
                jointIndex=joint_index,
                targetValue=joint_angles[joint_index],
                physicsClientId=self.client_id
            )
        p.stepSimulation(physicsClientId=self.client_id)
        time.sleep(1.0 / self.frequency)

    """
    This function commands the robot's arm to move to a specific set of joint angles.

    Args:
        joint_angles: A list of desired joint angles (in radians) for each joint of the arm.
    """

    def move_arm_to_joint_angles(self, joint_angles):
        # set arm's target joints
        for joint_index in range(6):
            p.setJointMotorControl2(
                bodyUniqueId=self.arm_id,
                jointIndex=joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_angles[joint_index],
                force=self.max_force,
                physicsClientId=self.client_id
            )

        start_time = time.time()

        while True:
            p.stepSimulation(physicsClientId=self.client_id)
            time.sleep(1.0 / self.frequency)

            # Check if reach the goal
            current_angles = [
                p.getJointState(self.arm_id, i, physicsClientId=self.client_id)[0] for i in range(6)
            ]
            diff_angles = [abs(a - b) for a, b in zip(joint_angles, current_angles)]
            if all(diff < self.threshold for diff in diff_angles):
                # print(
                #     "-" * 20
                #     + "\n"
                #     + "Reached target joint position:{}".format(joint_angles)
                # )
                break

            # Check for timeout
            if time.time() - start_time > self.timeout:
                print(
                    "-" * 20 + "\n" + "Timeout before reaching target joint position."
                )
                break

    """
    This function retrieves the information about the end effector of the robot arm.
    """

    def get_end_effector_info(self):
        joint_info = p.getJointInfo(self.arm_id, self.end_effector_index, physicsClientId=self.client_id)
        end_effector_name = joint_info[1].decode("utf-8")
        end_effector_info = p.getLinkState(
            bodyUniqueId=self.arm_id, linkIndex=self.end_effector_index, physicsClientId=self.client_id
        )
        end_effector_position = end_effector_info[0]
        end_effector_orientation = end_effector_info[1]
        print("-" * 20 + "\n" + "End effector name:{}".format(end_effector_name))
        print("Its position:{}".format(end_effector_position))
        print("Its orientation:{}".format(end_effector_orientation))
        return end_effector_position, end_effector_orientation

    def joints_to_cartesian(self, joint_angles):
        self.set_arm_to_joint_angles(joint_angles)

        end_effector_info = p.getLinkState(
            bodyUniqueId=self.arm_id, linkIndex=self.end_effector_index, physicsClientId=self.client_id
        )
        orientation = p.getEulerFromQuaternion(end_effector_info[1], physicsClientId=self.client_id)
        position = end_effector_info[0]
        # print("-" * 30 + "\n" + "position:{}".format(position))
        # print("orientation:{}".format(orientation))
        return position

    def cartesian_to_joints(self, position, orientation):
        joint_angles = p.calculateInverseKinematics( 
            bodyUniqueId=self.arm_id,
            endEffectorLinkIndex=self.end_effector_index,
            targetPosition=position,
            targetOrientation=p.getQuaternionFromEuler(orientation),
            maxNumIterations=self.max_iterations,
            residualThreshold=self.threshold,
            physicsClientId=self.client_id
        )
        return joint_angles

    """
    This function rotates the end effector of the robot arm by a specified angle.

    Args:
        angle (float): The desired rotation angle in radians.
    """

    def rotate_end_effector(self, angle):
        # Get the current joint states
        joint_indices = [
            i for i in range(6)
        ]  # Change this to your actual joint indices
        joint_states = p.getJointStates(self.arm_id, joint_indices, physicsClientId=self.client_id)

        # Create a new list of target joint angles
        target_joint_angles = [joint_state[0] for joint_state in joint_states]

        # Add desired rotation to the last joint's current angle
        target_joint_angles[-1] += angle

        # Set the target angles
        self.set_arm_to_joint_angles(target_joint_angles)

        # Step the simulation until the joints reach their target angles
        while True:
            # Update the current joint states
            joint_states = p.getJointStates(self.arm_id, joint_indices, physicsClientId=self.client_id)
            current_joint_angles = [joint_state[0] for joint_state in joint_states]

            # Check if all joints have reached their target angles
            if all(
                abs(current_joint_angles[i] - target_joint_angles[i]) < 0.01
                for i in range(6)
            ):
                break

            p.stepSimulation(physicsClientId=self.client_id)
            time.sleep(1.0 / self.frequency)
        print("Rotation completed!")

    """
    This function moves the robot arm's end effector to a target position.

    Args:
        end_effector_goal_pose (Pose): The desired pose of the end effector (includes both position and orientation). 
    """

    def move_end_effector_to_goal_position(self, end_effector_goal_pose):
        # get current end effector position
        state = p.getLinkState(
            bodyUniqueId=self.arm_id, linkIndex=self.end_effector_index, physicsClientId=self.client_id
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
                p.resetJointState(self.arm_id, joint_index, joint_angle, physicsClientId=self.client_id)

            p.stepSimulation(physicsClientId=self.client_id)

            # if p.getContactPoints(self.arm_id): # collision with other objects
            if p.getContactPoints(self.arm_id, self.base_id, physicsClientId=self.client_id):
                # Collision detected, break and re-calculate the IK
                break
            else:
                # get current end effector position
                state = p.getLinkState(
                    bodyUniqueId=self.arm_id, linkIndex=self.end_effector_index, physicsClientId=self.client_id
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
            "Could not reach target position without collision after {} attempts".format(
                self.max_attempts
            )
        )
        return False

    """
    This function performs pick-and-place manipulation of an object using the robot arm.

    Args:
        object_id: The ID of the target object to be manipulated.
        object_goal_position: The goal position for the target object.
        object_goal_orientation: The goal orientation for the target object.
    """
    def pick_place(self, object_id, object_goal_position, object_goal_orientation):
        # get target object position
        object_position_init, orientation = p.getBasePositionAndOrientation(object_id, physicsClientId=self.client_id)
        # consider the object's height
        _, _, min_z, _, _, max_z = self.pb_client.get_bounding_box(object_id)
        
        temp_height1, temp_height2 = 0.05, 0.01
        gripper_status = {"ungrasp": 0, "grasp": 1}
        for step in range(100): 
            if step < 20: # phase 1: move 20cm over object
                gripper_value = gripper_status["ungrasp"]
                target_position = [
                    object_position_init[0],
                    object_position_init[1],
                    object_position_init[2] + (max_z - min_z) + self.tcp_height + temp_height1
                ]
            elif step >= 20 and step < 40: # phase 2: grasp object
                gripper_value = gripper_status["grasp"]
                target_position = [
                    object_position_init[0],
                    object_position_init[1],
                    object_position_init[2] + (max_z - min_z) + self.tcp_height + temp_height2
                ]
            elif step >= 40 and step < 60: # phase 3: move 20cm over goal position
                gripper_value = gripper_status["grasp"]
                target_position = [
                    object_goal_position[0],
                    object_goal_position[1],
                    object_goal_position[2] + (max_z - min_z) + self.tcp_height + temp_height1,
                ]
            elif step >= 60 and step < 80: # phase 4: move 5cm over goal position
                gripper_value = gripper_status["grasp"]
                target_position = [
                    object_goal_position[0],
                    object_goal_position[1],
                    object_goal_position[2] + (max_z - min_z) + self.tcp_height + temp_height2,
                ]
            else: # phase 5: drop object
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
                cube_orn = p.getQuaternionFromEuler([0, math.pi, 0]) # control rotation
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
                        physicsClientId=self.client_id
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
                        physicsClientId=self.client_id
                    )

            p.stepSimulation(physicsClientId=self.client_id)

        print("-" * 20 + "\n" + "manipulation is done!")

    """
    This function calculates the inverse kinematics (IK) error for performing pick-and-place manipulation of an object using a robot arm.

    Args:
        goal_position: The desired goal position for the target object.
        goal_orientation: The desired goal orientation for the target object.
    """

    def calculate_IK_error(self, goal_position, goal_orientation):
        joint_angles = p.calculateInverseKinematics(
            bodyUniqueId=self.arm_id,
            endEffectorLinkIndex=self.end_effector_index,
            targetPosition=goal_position,
            targetOrientation=p.getQuaternionFromEuler(goal_orientation),
            maxNumIterations=self.max_iterations,
            residualThreshold=self.threshold,
            physicsClientId=self.client_id
        )
        print("IK: joint_angles is {}".format(joint_angles))

        self.move_arm_to_joint_angles(joint_angles)

        state = p.getLinkState(
            bodyUniqueId=self.arm_id, linkIndex=self.end_effector_index, physicsClientId=self.client_id
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
                physicsClientId=self.client_id
            ),
            # BUG: need to reset the arm separately or just call the sync_base_arm_pose?
            self.sync_base_arm_pose()
        else:
            p.resetBasePositionAndOrientation(
                self.base_id,
                pos.position,
                p.getQuaternionFromEuler([0, 0, pos.yaw]),
                physicsClientId=self.client_id
            )
            self.sync_base_arm_pose()

    def get_current_pos(self):
        """
        Get the current position of the robot
        """
        base_position, base_orientation = p.getBasePositionAndOrientation(self.base_id, physicsClientId=self.client_id)
        base_orientation = p.getEulerFromQuaternion(base_orientation)
        return Pose(base_position, base_orientation)

    def set_camera(
        self,
        robot_id: int,
        width: int = 224,
        height: int = 224,
    ):
        # get current position and orientation for robot
        base_position, orientation = p.getBasePositionAndOrientation(robot_id, physicsClientId=self.client_id)

        # set camera position and target position
        base_position, orientation_quat = p.getBasePositionAndOrientation(self.base_id, physicsClientId=self.client_id)
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
            physicsClientId=self.client_id
        )
        proj_mat = p.computeProjectionMatrixFOV(
            fov=60.0,  # field of view
            aspect=0.8,  # scale, default=1
            nearVal=0.01,  # view distance min
            farVal=10,
            physicsClientId=self.client_id
        )
        # get image from the camera
        w, h, rgb, depth, seg = p.getCameraImage(
            width=width,
            height=height,
            viewMatrix=view_mat,
            projectionMatrix=proj_mat,
            physicsClientId=self.client_id
        )
        return w, h, rgb, depth, seg

    def get_image(self, width, height):
        return self.set_camera(self.base_id, width, height)

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

    def label_target_on_rgb_by_segmentation(
        self,
        rgb_img,
        seg_img,
        height=224,
        width=224,
        target_object_id=4,
        file_output_path="./outputs/target_rectangle.png",
    ):
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
        p.resetBaseVelocity(self.base_id, zero_velocity, zero_velocity, physicsClientId=self.client_id)
        return True

    def grasp(self, object_id):
        # get target object position
        object_position_init, orientation = p.getBasePositionAndOrientation(object_id, physicsClientId=self.client_id)

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
                    physicsClientId=self.client_id
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