# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : Client.py
# @Time           : 2024-08-03 15:02:08
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : pybullet client 
"""

import json
import math
import os
import time
from datetime import datetime

import numpy as np
import pybullet as p
import pybullet_data

from Robotics_API import Pose
from Visualization.blender_render import PyBulletRecorder
from lisdf.parsing.sdf_j import load_sdf
from lisdf.utils.transformations import euler_from_quaternion

class Client:
    """
    Pybullet client, Provides some interfaces for interacting with simulation scenes
    """

    def __init__(self, cfg):
        """
        Initialize a new PyBullet Client object.

        Args:
            cfg (Config): Configuration object with various settings for the simulation.
        """

        if cfg.enable_GUI:
            if cfg.enable_capture:
                width, height = cfg.width, cfg.height
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

        if cfg.shadows:
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)  # enable shadows
            p.configureDebugVisualizer(
                p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0
            )  # close segment mark preview
            # p.configureDebugVisualizer(p.COV_ENABLE_PLANAR_REFLECTION, 1)   # enable planar reflection

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(cfg.Gravity[0], cfg.Gravity[1], cfg.Gravity[2])
        p.setPhysicsEngineParameter(numSolverIterations=cfg.numSolverIterations)
        # p.setRealTimeSimulation(1)  # set up real-time simulation

        # pybullet recorder for blender show
        self.blender = cfg.blender
        if self.blender:
            self.recorder = PyBulletRecorder()
            self.mtl_recorder = {}  # record manually added materials

        # parameters
        self.timestep = 1.0 / cfg.frequency
        self.timeout = cfg.timeout  # maximum time for planning

        # Enable caching of graphic shapes when loading URDF files
        self.enable_cache = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        plane_path = cfg.plane_urdf_path
        if plane_path.startswith("Asset"):
            os.path.join("..", plane_path)
        p.loadURDF(cfg.plane_urdf_path, flags=self.enable_cache)
        
        # pybullet data
        self.pybullet_data = pybullet_data.getDataPath()

    # ----------------------------------------------------------------
    # A few basic functions
    # ----------------------------------------------------------------

    def get_client_id(self):
        """
        Get the PyBullet client ID.

        Returns:
            int: The PyBullet client ID.
        """
        return self.client_id

    def get_datapath(self):
        """
        Get the path to the PyBullet data files.

        Returns:
            str: The path to the PyBullet data files.
        """
        return pybullet_data.getDataPath()

    def disconnect(self):
        """
        Disconnect from the PyBullet simulation. Save the recorder data if Blender is enabled.
        """
        if self.blender:
            self.record_save(self.mtl_recorder)
        p.disconnect(physicsClientId=self.client_id)

    def wait(self, x):  # seconds
        """
        Pause the execution for a given number of seconds.

        Args:
            x (float): Number of seconds to wait.
        """
        time.sleep(x)

    def run(self, x=1):  # steps
        """
        Step the simulation for a given number, robot=None of steps.

        Args:
            x (int): Number of simulation steps to run.
        """
        for _ in range(x):
            p.stepSimulation(physicsClientId=self.client_id)
            time.sleep(self.timestep)
            if self.blender:
                self.recorder.add_keyframe()

    def keep_run(self):  # keep steps
        while True:
            p.stepSimulation(physicsClientId=self.client_id)
            time.sleep(self.timestep)
            if self.blender:
                self.recorder.add_keyframe()

    # ----------------------------------------------------------------
    # Load scene
    # ----------------------------------------------------------------

    def load_object(
        self,
        obj_name,
        model_path,
        object_position=[0, 0, 0],
        object_orientation=[0, 0, 0],
        scale=1,
        fixed_base=False,
    ):
        """
        Load a given object into the PyBullet simulation environment.

        Args:
            obj_name (str): The name of the object.
            model_path (str): The path to the URDF file for the object.
            object_position (list): The initial position of the object.
            object_orientation (list): The initial orientation of the object.
            scale (float): The scale of the object.
            fixed_base (bool): If True, the object is fixed in place.

        Returns:
            int: The ID of the loaded object in the PyBullet simulation.
        """

        if (
            isinstance(object_orientation, (tuple, list, np.ndarray))
            and len(object_orientation) == 3
        ):
            object_orientation = p.getQuaternionFromEuler(
                object_orientation, physicsClientId=self.client_id
            )

        if model_path.startswith("Asset"):
            model_path = os.path.join("..", model_path)
        else:
            model_path = os.path.join(self.pybullet_data, model_path)
        
        print(f"[Client] \033[34mInfo\033[0m: Load {obj_name} from {model_path}!")
        
        object_id = p.loadURDF(
            fileName=model_path,
            basePosition=object_position,
            baseOrientation=object_orientation,
            globalScaling=scale,
            useFixedBase=fixed_base,
            physicsClientId=self.client_id,
            flags=self.enable_cache
        )

        if self.blender:
            self.register_object(object_id, model_path, scale)

        setattr(self, obj_name, object_id)

        self.run(10)

        return object_id

    def create_scene(self, scene_path):
        """
        Import the complete environment from the environment file based on the basic environment.

        Args:
            scene_path (str): Path to the scene JSON file.
        """
        print(f"[Client] \033[34mInfo\033[0m: Start load scene from {scene_path}!")
        
        if scene_path.startswith("Asset"):
            scene_path = os.path.join("..", scene_path)

        if scene_path.endswith('.json'):
            with open(scene_path, "r") as f:
                scene_data = json.load(f)
            for object in scene_data:
                object_orientation = [
                    eval(i) if isinstance(i, str) else i
                    for i in object["object_orientation"]
                ]
                self.load_object(
                    object["obj_name"],
                    object["model_path"],
                    object["object_position"],
                    object_orientation,
                    object["scale"],
                    object["fixed_base"],
                )
        elif scene_path.endswith('.lisdf'):
            lissdf_results = load_sdf(scene_path)
            models = lissdf_results.worlds[0].models
            fixed_base=True
            for model in models:
                model_uri = model.uri.replace('../../', 'Asset/Scene/')
                orientation=list(reversed(euler_from_quaternion(list(reversed(model.pose.quat_wxyz)))))
                self.load_object(
                    model.name,
                    model_uri,
                    model.pose.pos,
                    orientation,
                    model.scale[0],
                    fixed_base
                )
        else:
            raise ValueError(
                print("[Client] \033[31merror\033[0m: Scene format must be json or lisdf !")
            )  
            
        self.run(120)  
        print(f"[Client] \033[34mInfo\033[0m: Success load scene from {scene_path}!")

    # ----------------------------------------------------------------
    # object joint info / operate
    # ----------------------------------------------------------------

    def remove_object(self, object):
        """
        Remove an object from the scene.

        Args:
            object (int / str): The ID or name of the object to remove.
        """
        object_id = self.resolve_object_id(object)
        p.removeBody(object_id)
        self.run(10)

    def change_object_joint_angle(
        self, object, joint_index, target_position, max_force=5
    ):
        """
        Change the state of a specific joint of the object.

        Args:
            object (int / str): The ID or name of the object.
            joint_index (int): The index of the joint to be changed.
            target_position (float): The target position of the joint in radians.
            max_force (float): The maximum force to be applied to achieve the target position.
        """

        object_id = self.resolve_object_id(object)
        p.setJointMotorControl2(
            bodyUniqueId=object_id,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_position,
            force=max_force,
            physicsClientId=self.client_id,
        )

        self.run(10)

    # ----------------------------------------------------------------
    # Get info from environment
    # ----------------------------------------------------------------

    def get_object_id(self, object_name):
        """
        Get the ID of an object by its name.

        Args:
            object_name (str): The name of the object.

        Returns:
            int: The ID of the object.
        """
        return getattr(self, object_name)

    def resolve_object_id(self, object):
        """
        Resolve the object ID from either an ID or name.

        Args:
            object (int / str): The ID or name of the object.

        Returns:
            int: The ID of the object.

        Raises:
            AssertionError: If the object name does not exist or the input type is incorrect.
        """
        if isinstance(object, str):
            assert hasattr(self, object), f"scene has not object named {object}!"
            object_id = self.get_object_id(object)
        elif isinstance(object, int):
            object_id = object
        else:
            assert (0, "error input type")
        return object_id

    def print_object_joint_info(self, object):
        """
        Print information about the links and joints of an object.

        Args:
            object (int / str): The ID or name of the object.
        """

        object_id = self.resolve_object_id(object)

        num_joints = p.getNumJoints(object_id, physicsClientId=self.client_id)
        print(
            "[Client] \033[34mInfo\033[0m: The object {} has {} joints".format(
                object_id, num_joints
            )
        )
        for i in range(num_joints):
            joint_info = p.getJointInfo(object_id, i, physicsClientId=self.client_id)
            joint_name = joint_info[1]
            joint_state = p.getJointState(object_id, i, physicsClientId=self.client_id)
            joint_angle = joint_state[0]
            print(
                "[Client] \033[34mInfo\033[0m: Joint index:{}, name:{}, angle:{}".format(
                    i, joint_name, joint_angle
                )
            )

    def get_object_pose(self, object):
        """
        Retrieve the pose of a object.

        Args:
            object (int / str): The ID or name of the object.

        Returns:
            Pose: The pose of the object.
        """

        object_id = self.resolve_object_id(object)
        position, orientation = p.getBasePositionAndOrientation(object_id)
        return Pose(position, orientation)

    def get_object_link_pose(self, object, link_id):
        """
        Retrieve the pose of a given link of an object.

        Args:
            object (int / str): The ID or name of the object.
            link_id (int): The ID of the link.

        Returns:
            Pose: The pose of the link.
        """

        object_id = self.resolve_object_id(object)
        link_state = p.getLinkState(object_id, link_id)
        return Pose(link_state[0], link_state[1])

    def get_link_bounding_box(self, object, link_id):
        """
        Retrieve the bounding box of a specific link of an object in the PyBullet simulation environment.

        Args:
            object (int / str): The ID or name of the object.
            link_id (int): The ID of the link.

        Returns:
            list: The minimum and maximum x, y, z coordinates of the bounding box.
        """

        object_id = self.resolve_object_id(object)

        (min_x, min_y, min_z), (max_x, max_y, max_z) = p.getAABB(
            object_id, link_id, physicsClientId=self.client_id
        )
        return [min_x, min_y, min_z, max_x, max_y, max_z]

    def get_bounding_box(self, object):
        """
        Retrieve the bounding box of a given object in the PyBullet simulation environment.

        Args:
            object (int / str): The ID or name of the object.

        Returns:
            list: The minimum and maximum x, y, z coordinates of the bounding box.
        """

        object_id = self.resolve_object_id(object)

        link_ids = [
            i
            for i in range(
                -1, p.getNumJoints(object_id, physicsClientId=self.client_id)
            )
        ]

        aabb_bounds = np.array(
            [
                p.getAABB(object_id, link_id, physicsClientId=self.client_id)
                for link_id in link_ids
            ]
        )

        min_bounds = aabb_bounds[:, 0, :]
        max_bounds = aabb_bounds[:, 1, :]
        min_x, min_y, min_z = np.min(min_bounds, axis=0)
        max_x, max_y, max_z = np.max(max_bounds, axis=0)

        return [min_x, min_y, min_z, max_x, max_y, max_z]

    def get_all_link_bounding_box(self, object):
        """
        Retrieve all bounding box of a given object in the PyBullet simulation environment.

        Args:
            object (int / str): The ID or name of the object.

        Returns:
            list: The minimum and maximum x, y, z coordinates of the bounding box for all object link.
        """

        object_id = self.resolve_object_id(object)

        link_ids = [
            i
            for i in range(
                -1, p.getNumJoints(object_id, physicsClientId=self.client_id)
            )
        ]

        aabb_bounds = [
            p.getAABB(object_id, link_id, physicsClientId=self.client_id)
            for link_id in link_ids
        ]

        return aabb_bounds

    # ----------------------------------------------------------------
    # For blender
    # ----------------------------------------------------------------

    def register_object(self, object_id, model_path, scale=1.0):
        """
        Register an object with the PyBullet-blender recorder.

        Args:
            object_id (int): The ID of the object.
            model_path (str): The path to the model file of the object.
            scale (float): The scale of the object.
        """
        self.recorder.register_object(object_id, model_path, scale)

    def record_save(self, mtl_recorder):
        """
        Save the current pybullet-blender recording to a file with a timestamped name.
        """
        current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.recorder.save(f"../Examples/record/{current_time}.pkl", mtl_recorder)


if __name__ == "__main__":
    os.chdir(os.path.dirname(os.getcwd()))
    print(os.getcwd())
