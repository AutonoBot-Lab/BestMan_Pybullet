"""
@Description :   A few functions used in bestman robot, where the robot has a base and an arm.
@Author      :   Yan Ding 
@Time        :   2023/08/30 20:43:44
"""

import math
import time
import numpy as np
import pybullet as p
from abc import abstractmethod

from .Pose import Pose
from Visualization import Camera
from Controller.PIDController import PIDController


class Bestman_sim:
    
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
        
        self.client = client
        self.client_id = self.client.get_client_id()
        
        self.visualizer = visualizer
        # self.enable_cache = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES   # Enable caching of graphic shapes when loading URDF files
        
        # Init PID controller
        controller_cfg = cfg.Controller
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
        robot_cfg = cfg.Robot
        init_pose = Pose(robot_cfg.init_pose[:3], robot_cfg.init_pose[3:])
        # self.base_id = p.loadURDF(
        #     fileName=robot_cfg.base_urdf_path,
        #     basePosition=init_pose.position,
        #     baseOrientation=p.getQuaternionFromEuler(init_pose.orientation),
        #     useFixedBase=True,
        #     physicsClientId=self.client_id,
        #     flags=self.enable_cache
        # )
        self.base_id = self.client.load_object(
            obj_name='base',
            model_path=robot_cfg.base_urdf_path,
            object_position=init_pose.position,
            object_orientation=init_pose.orientation,
            fixed_base = True
        )
        # setattr(self.client, 'base', self.base_id)
        # if self.client.blender: self.client.register_object(self.base_id, robot_cfg.base_urdf_path)
        
        # Init arm
        # self.arm_id = p.loadURDF(
        #     fileName=robot_cfg.arm_urdf_path,
        #     basePosition=init_pose.position,
        #     baseOrientation=p.getQuaternionFromEuler(init_pose.orientation),
        #     useFixedBase=True,
        #     physicsClientId=self.client_id,
        #     flags=self.enable_cache
        # )
        self.arm_id = self.client.load_object(
            obj_name='arm',
            model_path=robot_cfg.arm_urdf_path,
            object_position=init_pose.position,
            object_orientation=init_pose.orientation,
            fixed_base = True
        )
        
        
        # setattr(self.client, 'arm', self.arm_id)
        # if self.client.blender: self.client.register_object(self.arm_id, robot_cfg.arm_urdf_path)
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
        
        # Init camera
        # Camera_cfg = cfg.Camera
        # self.camera = Camera(Camera_cfg, self.base_id, self.arm_height)
        
        # update image
        # self.camera.update()
        # self.camera.get_rgb_image(True, True)
        # self.camera.get_depth_image(True, True)
        
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
        
        # global parameters
        self.init_pose = init_pose   # Used when resetting the robot position

    
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

    def get_current_base_pose(self):
        """
        Retrieves the current position and orientation of the robot base.

        Returns:
            Pose: A Pose object representing the current pose of the base.
                  The pose contains the position as a list [x, y, z] and the orientation as a list [roll, pitch, yaw].
        """
        base_position, base_orientation = p.getBasePositionAndOrientation(
            self.base_id, physicsClientId=self.client_id
        )
        return Pose(base_position, base_orientation)

    # TODO: 为什么需要这个函数,而且也没有判断是否停下来了
    def stop_base(self):
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
                
                self.sim_sync_base_arm_pose()

            # Ensure final orientation is set accurately
            orientation = angle_to_quaternion(target_yaw)
            position, _ = p.getBasePositionAndOrientation(
                self.base_id, physicsClientId=self.client_id
            )
            p.resetBasePositionAndOrientation(
                self.base_id, position, orientation, physicsClientId=self.client_id
            )
            self.client.run()

        else:
            orientation = angle_to_quaternion(target_yaw)
            position, _ = p.getBasePositionAndOrientation(
                self.base_id, physicsClientId=self.client_id
            )
            p.resetBasePositionAndOrientation(
                self.base_id, position, orientation, physicsClientId=self.client_id
            )
            
            self.sim_sync_base_arm_pose()
            
            self.client.run()
        
        # self.camera.update()
    
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

        p.resetBasePositionAndOrientation(
            self.base_id,
            target_position,
            target_orientation,
            physicsClientId=self.client_id,
        )
        
        self.sim_sync_base_arm_pose()
    
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
            pose = self.get_current_base_pose()
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
                pose = self.get_current_base_pose()
                break
            
            self.sim_action(-output)
            
        self.client.run(10) 
    
    def navigate_base(self, goal_base_pose, path, enable_plot = False):
        """
        Navigate a robot from its current position to a specified goal position

        Args:
            goal_base_pose (Pose): The target pose (position and orientation) for the robot.
        """
        
        # for i, waypoint in enumerate(path, start=1):
        for i in range(len(path)):
            
            next_point = [path[i][0], path[i][1], 0]
            # move to each waypoint
            self.move_base_to_waypoint(
                Pose([path[i][0], path[i][1], 0], goal_base_pose.orientation)
            )
            
            # draw the trajectory
            if i != 0 and enable_plot:
                front_point = [path[i-1][0], path[i-1][1], 0]
                p.addUserDebugLine(front_point, next_point, lineColorRGB=[1, 0, 0], lineWidth=3, physicsClientId=self.client_id)

            # self.camera.update()
        
        self.rotate_base(goal_base_pose.yaw)
        self.client.run(10)
        # self.camera.update()
        print("-" * 20 + "\n" + "Navigation is done!")
    
    
    # ----------------------------------------------------------------
    # functions for arm
    # ----------------------------------------------------------------
    
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
        return self.arm_joints_idx
    
    def get_all_joint_idx(self):
        """
        Retrieves the indices of the joints in the robot arm.
        
        Returns:
            list: A list of indices for the joints in the robot arm.
        """  
        return list(range(p.getNumJoints(self.arm_id, physicsClientId=self.client_id)))
    
    def get_tcp_link(self):
        """
        Retrieves the TCP (Tool Center Point) link of the robot arm.
        
        Returns:
            str: The TCP link of the robot arm.
        """
        return self.tcp_link
    
    def get_tcp_link_height(self):
        """
        Retrieves the TCP (Tool Center Point) link height of the robot arm.
        
        Returns:
            str: The TCP link of the robot arm.
        """
        return self.tcp_height
    
    def get_end_effector_link(self):
        """
        Retrieves the end effector link id 
        
        Returns:
            str: The end effector link id of the robot arm.
        """
        return self.end_effector_index

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

    def get_current_end_effector_pose(self):
        """
        Retrieve arm's end effect information
        """
        
        end_effector_info = p.getLinkState(
            bodyUniqueId=self.arm_id,
            linkIndex=self.end_effector_index,
            physicsClientId=self.client_id,
        )
        return Pose(end_effector_info[0], end_effector_info[1])
    
    def sim_adjust_arm_height(self, height):
        """
        Dynamically adjusts the height of the robot arm.
        
        Args:
            height (float): The new height to set for the robot arm.
        """
        self.arm_height = height
        self.client.run(10)
        print("-" * 20 + "\n" + "Arm height has changed into {}".format(height))
    
    def sim_set_arm_to_joint_values(self, joint_values):
        """
        Set arm to move to a specific set of joint angles, witout considering physics

        Args:
            joint_values: A list of desired joint angles (in radians) for each joint of the arm.
        """
        
        for joint, value in zip(self.arm_joints_idx, joint_values):
            p.resetJointState(self.arm_id, joint, value, targetVelocity=0)
            
    def sim_debug_set_arm_to_joint_values(self):
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
            self.client.run()
            joint_states = p.getJointStates(self.arm_id, self.arm_joints_idx, physicsClientId=self.client_id)
            current_angles = [state[0] for state in joint_states]
            diff_angles = [abs(a - b) for a, b in zip(joint_values, current_angles)]
            if all(diff < self.threshold for diff in diff_angles):
                break
            
            if time.time() - start_time > self.timeout:  # avoid time anomaly
                if p.getContactPoints(self.arm_id):
                    assert(0, "-" * 20 + "\n" + "The arm collides with other objects!")
                # print("-" * 20 + "\n" + "Timeout before reaching target joint position.")
                break
            
    def joints_to_cartesian(self, joint_values):
        """
        Transforms the robot arm's joint angles to its Cartesian coordinates.
        
        Args:
            joint_values (list): A list of joint angles for the robot arm.
        
        Returns:
            tuple: A tuple containing the Cartesian coordinates (position and orientation) of the robot arm.
        """
        
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
        return Pose(position, orientation)

    def cartesian_to_joints(self, pose):
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
            targetPosition=pose.position,
            targetOrientation=p.getQuaternionFromEuler(pose.orientation),
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
            
            self.client.run()

        print("-" * 20 + "\n" + "Rotation completed!")

    def move_end_effector_to_goal_pose(self, end_effector_goal_pose, steps=10):
        """
        Move arm's end effector to a target position.

        Args:
            end_effector_goal_pose (Pose): The desired pose of the end effector (includes both position and orientation).
        """
        
        start_pose = self.get_current_end_effector_pose()
        for t in np.linspace(0, 1, steps):
            interpolated_position = (1 - t) * np.array(start_pose.position) + t * np.array(end_effector_goal_pose.position)
            interpolated_orientation = p.getQuaternionSlerp(p.getQuaternionFromEuler(start_pose.orientation), p.getQuaternionFromEuler(end_effector_goal_pose.orientation), t)
            interpolated_pose = Pose(interpolated_position, interpolated_orientation)
            joint_values = self.cartesian_to_joints(interpolated_pose)
            self.move_arm_to_joint_values(joint_values)
        self.client.run(10)
    
    def execute_trajectory(self, trajectory, enable_plot=False):
        """Execute the path planned by Planner

        Args:
            trajectory: List, each element is a list of angles, corresponding to a transformation
        """
        
        for i in range(len(trajectory)):
            self.move_arm_to_joint_values(trajectory[i])
            
            # if i % 3 == 0:
                # self.visualizer.draw_link_pose(self.arm_id, self.end_effector_index)
            current_point = self.get_current_end_effector_pose().position
            
            # draw the trajectory
            if i != 0 and enable_plot:
                p.addUserDebugLine(front_point, current_point, lineColorRGB=[1, 0, 0], lineWidth=3, physicsClientId=self.client_id)
            
            front_point = current_point
        
        self.client.run(10)
        print("\n" + "-" * 20 + "\n" + "Excite trajectory finished!"+ "\n" + "-" * 20 + "\n")
    
    def calculate_IK_error(self, goal_pose):
        """Calculate the inverse kinematics (IK) error for performing pick-and-place manipulation of an object using a robot arm.

        Args:
            goal_position: The desired goal position for the target object.
            goal_orientation: The desired goal orientation for the target object.
        """
        
        end_effector_pose = p.getLinkState(
            bodyUniqueId=self.arm_id,
            linkIndex=self.end_effector_index,
            physicsClientId=self.client_id
        )
        
        distance = np.linalg.norm(np.array(end_effector_pose[0]) - np.array(goal_pose.position))
        
        return distance
    
    
    # ----------------------------------------------------------------
    # functions between base and arms
    # ----------------------------------------------------------------
    
    def get_robot_max_size(self):
        """Retrieves the maximum size of the robot (arm and base) in meters.

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
            print("unknown name: {}, please input base or arm!".format(name))

        num_joints = p.getNumJoints(id, physicsClientId=self.client_id)
        print("\n" + "-" * 20 + "\n" + "Robot {} has {} joints".format(id, num_joints) + "\n" + "-" * 20 + "\n")
        for i in range(num_joints):
            joint_info = p.getJointInfo(id, i, physicsClientId=self.client_id)
            joint_name = joint_info[1]
            link_name = joint_info[12].decode("UTF-8")
            joint_state = p.getJointState(id, i, physicsClientId=self.client_id)
            joint_angle = joint_state[0]
            print(
                "Joint index:{}, name:{}, angle:{}".format(i, joint_name, joint_angle)
            )
            print("Link index: {}, name: {}".format(i, link_name))
    
    def sim_sync_base_arm_pose(self):
        """
        Synchronizes the pose of the robot arm with the base.
        
        This function ensures that the positions of the robot arm and base are aligned.
        """

        position, orientation = p.getBasePositionAndOrientation(self.base_id, physicsClientId=self.client_id)
        if self.arm_id is not None:
            p.resetBasePositionAndOrientation(
                self.arm_id, [position[0], position[1], self.arm_height] , orientation, physicsClientId=self.client_id
            )
            # self.client.run()

    
    # ----------------------------------------------------------------
    # functions for gripper
    # ----------------------------------------------------------------
    
    # @abstractmethod
    # def sim_active_gripper(self, object, value):
    #     """
    #     Activate or deactivate the gripper.
        
    #     Args:
    #         object_id (init): ID of the object related to gripper action.
    #         value (int): 0 or 1, where 0 means deactivate (ungrasp) and 1 means activate (grasp).
    #     """
    #     pass    
    
    # ----------------------------------------------------------------
    # functions for camera
    # ----------------------------------------------------------------

    def update_camera(self):
        self.camera.update()
        
    def get_camera_rgb_image(self, enable_show=False, enable_save=False):
        self.camera.get_rgb_image(enable_show, enable_save)
    
    def get_camera_depth_image(self, enable_show=False, enable_save=False):
        self.camera.get_depth_image(enable_show, enable_save)
    
    
    # ----------------------------------------------------------------
    # functions for pick / place
    # ----------------------------------------------------------------

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
                self.move_end_effector_to_goal_pose(
                    Pose(target_position, object_goal_orientation)
                )

            if gripper_value == 0 and self.gripper_id != None:
                p.removeConstraint(self.gripper_id, physicsClientId=self.client_id)
                self.gripper_id = None
                self.client.run(10)
                # for _ in range(self.frequency):
                #     p.stepSimulation(physicsClientId=self.client_id)

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

            # p.stepSimulation(physicsClientId=self.client_id)
            self.client.run(10)
    
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