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


"""
Client class
"""
class PbClient:
    def __init__(self, enable_GUI = True, enable_Debug = False):
        if enable_GUI:
            self.client_id = p.connect(p.GUI)
        else:
            self.client_id = p.connect(p.DIRECT)
        if enable_GUI:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        p.setPhysicsEngineParameter(numSolverIterations=1000)  # Set the number of constraint solver iterations; Higher values increase precision but also increase computation time
        p.loadURDF("plane.urdf")
        
        # parameters for base
        self.frequency = 240  # simulation step for base and arm
        self.timeout = 100.0  # maximum time for planning

        # Obstacles in the environment
        self.obstacle_navigation_ids = []  # for navigation
        self.obstacle_manipulation_ids = []  # for manipulation

    def get_client(self):
        return self.client_id

    def disconnect_pybullet(self):
        p.disconnect(physicsClientId=self.client_id)

    def wait(self, x):
        time.sleep(x)

    def run(self, x):
        for _ in range(x):
            p.stepSimulation(physicsClientId=self.client_id)
            time.sleep(1.0 / self.frequency)

    # ----------------------------------------------------------------
    # Visualization functions
    # ----------------------------------------------------------------

    # """
    # This function sets the debug visualizer camera in a vertical view. The vertical view can be useful in various simulation scenarios like overhead view, bird's eye view etc. PyBullet Physics engine's debug visualizer camera is adjusted using this function.
    # Args:
    #     height: The distance of the camera from the target point. This determines how far the camera is placed from the position (0, 0, 0). This value effectively becomes the "height" of the camera, providing a vertical overview.
    # """

    def enable_vertical_view(self, height, position):
        p.resetDebugVisualizerCamera(
            cameraDistance=height,
            cameraYaw=0,
            cameraPitch=-89.9,
            cameraTargetPosition=[0, 0, 0],
            physicsClientId=self.client_id,
        )

    # ----------------------------------------------------------------
    # Video functions
    # ----------------------------------------------------------------
    
    """
    This function is to enable and disable recording.
    """

    def start_record(self, fileName):
        logId = p.startStateLogging(
            p.STATE_LOGGING_VIDEO_MP4,
            "./image/" + fileName + ".mp4",
            physicsClientId=self.client_id,
        )
        print("The video can be found in " + "./image/" + fileName + ".mp4")
        return logId

    def end_record(self, logId):
        p.stopStateLogging(logId, physicsClientId=self.client_id)
    
    # ----------------------------------------------------------------
    # Add object functions
    # ----------------------------------------------------------------

    """
    This function loads a given object into the PyBullet simulation environment. 
    The object is specified by a URDF (Unified Robot Description Format) file, which is a common format for representing a robot or other complex object in a simulation.

    Args:
        model_path (str): The path to the URDF file for the object.
        object_position (list): The initial position of the object in the simulation, as a three-element list or tuple of x, y, z coordinates.
        object_orientation (list): The initial orientation of the object in the simulation, as a three-element list or tuple of roll, pitch, and yaw angles (in radians). This will be converted to a quaternion for use with PyBullet.
        obj_name (str): The name of the object. This is used to create an attribute in the class instance that holds the ID of the object in the PyBullet simulation.

    Returns:
        The ID of the loaded object in the PyBullet simulation.
    """

    def load_object(
        self, model_path, object_position, object_orientation, scale, obj_name
    ):
        object_orientation = p.getQuaternionFromEuler(object_orientation, physicsClientId=self.client_id)
        setattr(
            self,
            f"{obj_name}_id",
            p.loadURDF(
                model_path,
                basePosition=object_position,
                baseOrientation=object_orientation,
                globalScaling=scale,
                physicsClientId=self.client_id
            ),
        )
        print("-" * 20 + "\n" + "{}_id: {}".format(obj_name, getattr(self, f"{obj_name}_id")))
        self.obstacle_navigation_ids.append(getattr(self, f"{obj_name}_id"))
        self.obstacle_manipulation_ids.append(getattr(self, f"{obj_name}_id"))
        time.sleep(1.0 / self.frequency)
        return getattr(self, f"{obj_name}_id")

    # ----------------------------------------------------------------
    # Get info from environment
    # ----------------------------------------------------------------
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

    """
    This function retrieves the bounding box for a given object in the PyBullet simulation environment. 

    Args:
        object_id (int): The ID of the object in the PyBullet simulation.
    Prints:
        The function prints the minimum and maximum x, y, z coordinates of the bounding box of the object.
    """

    def get_bounding_box(self, object_id):
        link_ids = [i for i in range(-1, p.getNumJoints(object_id, physicsClientId=self.client_id))]
        min_x, min_y, min_z = float("inf"), float("inf"), float("inf")
        max_x, max_y, max_z = float("-inf"), float("-inf"), float("-inf")
        for link_id in link_ids:
            (x_min, y_min, z_min), (x_max, y_max, z_max) = p.getAABB(object_id, link_id, physicsClientId=self.client_id)
            min_x = min(min_x, x_min)
            min_y = min(min_y, y_min)
            min_z = min(min_z, z_min)
            max_x = max(max_x, x_max)
            max_y = max(max_y, y_max)
            max_z = max(max_z, z_max)
        # print("-" * 20 + "\n" + "object_id: {}".format(object_id))
        # print("min_x:{:.2f}, min_y:{:.2f}, min_z:{:.2f}".format(min_x, min_y, min_z))
        # print("max_x:{:.2f}, max_y:{:.2f}, max_z:{:.2f}".format(max_x, max_y, max_z))
        return [min_x, min_y, min_z, max_x, max_y, max_z]

    """
    This function checks if two bounding boxes collide.
    Each box is defined as [min_x, min_y, min_z, max_x, max_y, max_z].
        :param box1: bounding box of the first object.
        :param box2: bounding box of the second object.
        :return: True if they collide, False otherwise.
    """

    def check_collision_xyz(self, box1, box2):
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