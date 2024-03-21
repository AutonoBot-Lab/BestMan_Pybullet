"""
@Description :   A few functions used in communication with client
@Author      :   Yan Ding 
@Time        :   2023/08/30 22:22:14
"""


import pybullet as p
import pybullet_data
import time
import numpy as np
import matplotlib.pyplot as plt
import random


"""
Client class
"""


class PbClient:
    def __init__(self, cfg):
        """
        Initialize a new PbClient object.

        Parameters:
            enable_GUI (bool): If True, the GUI will be enabled.
            enable_Debug (bool): If False, the debug visualizer will be disabled.

        Attributes:
            client_id (int): The id of the client.
            frequency (int): The simulation step for base and arm.
            timeout (float): The maximum time for planning.
            obstacle_navigation_ids (list): List of navigation obstacle ids.
            obstacle_manipulation_ids (list): List of manipulation obstacle ids.
        """
        
        
        if cfg.enable_GUI:
            if cfg.enable_capture:
                width, height = 1920, 1080
                self.client_id = p.connect(
                    p.GUI, options=f"--width={width} --height={height}"
                )
                p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
            else:
                self.client_id = p.connect(p.GUI)
        else:
            self.client_id = p.connect(p.DIRECT)

        if not cfg.enable_Debug:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(cfg.Gravity[0], cfg.Gravity[1], cfg.Gravity[2])
        p.setPhysicsEngineParameter(
            numSolverIterations=cfg.numSolverIterations
        )   # Set the number of constraint solver iterations; Higher values increase precision but also increase computation time
        
        self.obj_name_to_id = {}    # record object name to its id
        planeId = p.loadURDF(cfg.plane_urdf_path)
        
        # parameters for base
        self.frequency = cfg.frequency  # simulation step for base and arm
        self.timeout = cfg.timeout      # maximum time for planning
        
        # Obstacles in the environment
        self.obstacle_navigation_ids = []  # for navigation
        self.obstacle_manipulation_ids = []  # for manipulation

    # ----------------------------------------------------------------
    # a few helper functions
    # ----------------------------------------------------------------

    def get_client(self):
        return self.client_id

    def disconnect_pybullet(self):
        p.disconnect(physicsClientId=self.client_id)
        print("-" * 20 + "\n" + "The script ends!" + "\n" + "-" * 20)

    def wait(self, x):  # seconds
        time.sleep(x)
        print("-" * 20 + "\n" + "Has waitted {} seconds".format(x))

    def run(self, x):  # steps
        for _ in range(x):
            p.stepSimulation(physicsClientId=self.client_id)
            time.sleep(1.0 / self.frequency)
        print("-" * 20 + "\n" + "Has runned {} simulation steps".format(x))

    # ----------------------------------------------------------------
    # Visualization functions
    # ----------------------------------------------------------------
    def draw_axes(self, length=2.0, lineWidth=2.0, textSize=3.0):
        """
        Draws the x and y axes in the PyBullet environment with text labels.
        
        Parameters:
        - length (float): Length of the axes.
        - lineWidth (float): Width of the axes lines.
        - textSize (float): Size of the text labels.
        """
        origin = [0, 0, 0]  # The start point of the axes

        # Drawing the X-axis (in red)
        p.addUserDebugLine(
            lineFromXYZ=origin,
            lineToXYZ=[length, 0, 0],
            lineColorRGB=[1, 0, 0],
            lineWidth=lineWidth,
            physicsClientId=self.client_id,
        )

        # Drawing the Y-axis (in green)
        p.addUserDebugLine(
            lineFromXYZ=origin,
            lineToXYZ=[0, length, 0],
            lineColorRGB=[0, 1, 0],
            lineWidth=lineWidth,
            physicsClientId=self.client_id,
        )

        # Adding text labels
        p.addUserDebugText(
            text="X",
            textPosition=[length + 0.1, 0, 0],
            textColorRGB=[1, 0, 0],
            textSize=textSize,
            physicsClientId=self.client_id,
        )
        p.addUserDebugText(
            text="Y",
            textPosition=[0, length + 0.1, 0],
            textColorRGB=[0, 1, 0],
            textSize=textSize,
            physicsClientId=self.client_id,
        )

    def enable_vertical_view(self, camera_cfg):
        """
        Set the debug visualizer camera in a vertical view.

        Args:
            dist (float): The distance of the camera from the target point.
            position (list): A list of three floats representing the target position in 3D space.
            yaw (float, optional): The yaw component of the camera orientation. Defaults to 0.
            pitch (float, optional): The pitch component of the camera orientation. Defaults to -89.9.
        """
        p.resetDebugVisualizerCamera(
            cameraDistance=camera_cfg.dist,
            cameraYaw=camera_cfg.yaw,
            cameraPitch=camera_cfg.pitch,
            cameraTargetPosition=camera_cfg.position,
            physicsClientId=self.client_id,
        )
    
    # ----------------------------------------------------------------
    # Video functions
    # ----------------------------------------------------------------

    def start_record(self, fileName):
        """
        Enable and disable recording
        """
        logId = p.startStateLogging(
            p.STATE_LOGGING_VIDEO_MP4,
            "./image/" + fileName + ".mp4",
            physicsClientId=self.client_id,
        )
        print(
            "-" * 20
            + "\n"
            + "The video can be found in "
            + "./image/"
            + fileName
            + ".mp4"
        )
        return logId

    def end_record(self, logId):
        p.stopStateLogging(logId, physicsClientId=self.client_id)

    # ----------------------------------------------------------------
    # Add object functions
    # ----------------------------------------------------------------

    def get_appliance_joint_info(self, appliance_id):
        """
        Get appliance's joint info
        """
        num_joints = p.getNumJoints(appliance_id, physicsClientId=self.client_id)
        print(
            "-" * 20
            + "\n"
            + "The appliance {} has {} joints".format(appliance_id, num_joints)
        )
        for i in range(num_joints):
            joint_info = p.getJointInfo(appliance_id, i, physicsClientId=self.client_id)
            joint_name = joint_info[1]
            joint_state = p.getJointState(
                appliance_id, i, physicsClientId=self.client_id
            )
            joint_angle = joint_state[0]
            print(
                "Joint index:{}, name:{}, angle:{}".format(i, joint_name, joint_angle)
            )

    def change_appliance_joint(
        self, appliance_id, joint_index, target_position, max_force=5
    ):
        """
        Change the state of a specific joint of the appliance.

        Args:
        appliance_id (int): The id of the appliance.
        joint_index (int): The index of the joint to be changed.
        target_position (float): The target position of the joint in radians.
        max_force (float): The maximum force to be applied to achieve the target position.
        """
        p.setJointMotorControl2(
            bodyUniqueId=appliance_id,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_position,
            force=max_force,
            physicsClientId=self.client_id,
        )

    def run_slider_and_update_position(
        self, x, name, min_val, max_val, initial_val, obj_id=None
    ):
        """
        Run the simulation for a number of steps, creating sliders, reading their values,
        and updating the object position at each step.

        Args:
            x (int): The number of simulation steps to run.
            name (str): The base name of the sliders.
                        Three sliders will be created with names: name_x, name_y, name_z.
            min_val (float): The minimum value of the sliders.
            max_val (float): The maximum value of the sliders.
            initial_val (float): The initial value of the sliders.
            obj_id (int, optional): The id of the object to update. Defaults to None.
        """
        slider_ids = [
            p.addUserDebugParameter(
                f"{name}_{coord}",
                min_val,
                max_val,
                initial_val,
                physicsClientId=self.client_id,
            )
            for coord in ["x", "y", "z"]
        ]

        for _ in range(x):
            p.stepSimulation(physicsClientId=self.client_id)
            if obj_id is not None:
                position = [
                    p.readUserDebugParameter(id, physicsClientId=self.client_id)
                    for id in slider_ids
                ]
                orientation = p.getBasePositionAndOrientation(
                    obj_id, physicsClientId=self.client_id
                )[1]
                p.resetBasePositionAndOrientation(
                    obj_id, position, orientation, physicsClientId=self.client_id
                )
            time.sleep(1.0 / self.frequency)

    def load_object(
        self,
        model_path,
        object_position,
        object_orientation,
        scale,
        obj_name,
        fixed_base=False
    ):
        """
        Load a given object into the PyBullet simulation environment.

        Args:
            model_path (str): The path to the URDF file for the object.
            object_position (list): The initial position of the object
            object_orientation (list): The initial orientation of the object
            obj_name (str): The name of the object.

        Returns:
            The ID of the loaded object in the PyBullet simulation.
        """
        
        object_orientation = p.getQuaternionFromEuler(
            object_orientation, physicsClientId=self.client_id
        )
        
        object_id = p.loadURDF(
            model_path,
            basePosition=object_position,
            baseOrientation=object_orientation,
            globalScaling=scale,
            useFixedBase=fixed_base,
            physicsClientId=self.client_id,
        )
        
        self.obj_name_to_id[obj_name] = object_id
        
        # setattr(
        #     self,
        #     f"{obj_name}_id",
        #     p.loadURDF(
        #         model_path,
        #         basePosition=object_position,
        #         baseOrientation=object_orientation,
        #         globalScaling=scale,
        #         useFixedBase=fixed_base,
        #         physicsClientId=self.client_id,
        #     ),
        # )
        # # print(
        # #     "-" * 20
        # #     + "\n"
        # #     + "{}_id: {}".format(obj_name, getattr(self, f"{obj_name}_id"))
        # # )
        # if tag_obstacle_navigate:
        #     self.obstacle_navigation_ids.append(getattr(self, f"{obj_name}_id"))
        # self.obstacle_manipulation_ids.append(getattr(self, f"{obj_name}_id"))
        time.sleep(1.0 / self.frequency)
        # return getattr(self, f"{obj_name}_id")
        return object_id

    # ----------------------------------------------------------------
    # Get info from environment
    # ----------------------------------------------------------------

    # get occupancy network
    def get_occupancy_network(
        self, object_id, x_max=10, y_max=10, resolution=0.1, enable_plot=False
    ):
        """
        Create an occupancy grid for a given environment.

        Args:
        :param x_max: float, maximum x coordinate of the environment
        :param y_max: float, maximum y coordinate of the environment
        :param resolution: float, grid resolution
        :return: numpy array, 2D occupancy grid
        """
        # Create 2D grid
        x = np.arange(-x_max / 2, x_max / 2, resolution)
        y = np.arange(-y_max / 2, y_max / 2, resolution)
        X, Y = np.meshgrid(x, y)

        # Initialize empty occupancy grid
        occupancy_grid = np.zeros_like(X, dtype=np.int)

        # Check each grid cell
        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                # Center coordinates of the grid cell
                x, y = X[i, j], Y[i, j]
                # Check collision between the point and the robot
                points = p.getClosestPoints(
                    bodyA=object_id,
                    bodyB=-1,
                    distance=0,
                    linkIndexA=-1,
                    linkIndexB=-1,
                    physicsClientId=self.client_id,
                )
                if points:
                    occupancy_grid[i, j] = 1
                    print("!debug")

        if enable_plot:
            plt.figure()
            plt.imshow(occupancy_grid, cmap="Greys", origin="lower")
            plt.xlabel("x (m)")
            plt.ylabel("y (m)")
            plt.title("Occupancy Grid")
            plt.show()
        return occupancy_grid

    # sample points within a area
    def generate_point_within_area(self, min_x, min_y, max_x, max_y):
        x = random.uniform(min_x, max_x)
        y = random.uniform(min_y, max_y)
        return x, y

    def generate_point_within_area_with_fixed_z(
        self, min_x, min_y, max_x, max_y, fixed_z
    ):
        x = random.uniform(min_x, max_x)
        y = random.uniform(min_y, max_y)
        return x, y, fixed_z

    def generate_point_outside_area(self, min_x, min_y, max_x, max_y):
        range_x_min = -5
        range_x_max = 5
        range_y_min = -5
        range_y_max = 5

        regions = [
            (range_x_min, range_y_min, min_x, range_y_max),  # Left of table
            (max_x, range_y_min, range_x_max, range_y_max),  # Right of table
            (range_x_min, range_y_min, range_x_max, min_y),  # Below table
            (range_x_min, max_y, range_x_max, range_y_max),  # Above table
        ]

        # Randomly choose a region
        chosen_region = random.choice(regions)

        x = random.uniform(chosen_region[0], chosen_region[2])
        y = random.uniform(chosen_region[1], chosen_region[3])

        return x, y

    def get_bounding_box(
        self, object_id, print_output=False
    ):  # TODO: use a polygon to represent the bounding box
        """
        This function retrieves the bounding box for a given object in the PyBullet simulation environment.

        Args:
            object_id (int): The ID of the object in the PyBullet simulation.
        Prints:
            The function prints the minimum and maximum x, y, z coordinates of the bounding box of the object.
        """
        link_ids = [
            i
            for i in range(
                -1, p.getNumJoints(object_id, physicsClientId=self.client_id)
            )
        ]
        min_x, min_y, min_z = float("inf"), float("inf"), float("inf")
        max_x, max_y, max_z = float("-inf"), float("-inf"), float("-inf")
        for link_id in link_ids:
            (x_min, y_min, z_min), (x_max, y_max, z_max) = p.getAABB(
                object_id, link_id, physicsClientId=self.client_id
            )
            min_x = min(min_x, x_min)
            min_y = min(min_y, y_min)
            min_z = min(min_z, z_min)
            max_x = max(max_x, x_max)
            max_y = max(max_y, y_max)
            max_z = max(max_z, z_max)

        if print_output:
            print("-" * 20 + "\n" + "object_id: {}".format(object_id))
            print(
                "min_x:{:.2f}, min_y:{:.2f}, min_z:{:.2f}".format(min_x, min_y, min_z)
            )
            print(
                "max_x:{:.2f}, max_y:{:.2f}, max_z:{:.2f}".format(max_x, max_y, max_z)
            )

        return [min_x, min_y, min_z, max_x, max_y, max_z]

    def check_collision_xyz(self, box1, box2):
        """
        Check if two bounding boxes collide.
        Each box is defined as [min_x, min_y, min_z, max_x, max_y, max_z].
            :param box1: bounding box of the first object.
            :param box2: bounding box of the second object.
            :return: True if they collide, False otherwise.
        """
        # Check for collision along the x-axis
        if box1[0] > box2[3] or box1[3] < box2[0]:
            return False

        # Check for collision along the y-axis
        if box1[1] > box2[4] or box1[4] < box2[1]:
            return False

        # Check for collision along the z-axis
        if box1[2] > box2[5] or box1[5] < box2[2]:
            return False

        return True

    def check_collision_xy(self, box1, box2):
        # Check for collision along the x-axis
        if box1[0] > box2[3] or box1[3] < box2[0]:
            return False

        # Check for collision along the y-axis
        if box1[1] > box2[4] or box1[4] < box2[1]:
            return False

        return True
