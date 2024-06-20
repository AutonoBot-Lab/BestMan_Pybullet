"""
@Description :   A few functions used in communication with client
@Author      :   Yan Ding 
@Time        :   2023/08/30 22:22:14
"""

import math
import json
import time
import random
import numpy as np
import pybullet as p
import pybullet_data

  
class Client:
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
                # width, height = 1920, 1080
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
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(cfg.Gravity[0], cfg.Gravity[1], cfg.Gravity[2])
        p.setPhysicsEngineParameter(
            numSolverIterations=cfg.numSolverIterations
        )
        
        # Enable caching of graphic shapes when loading URDF files
        self.enable_cache = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        planeId = p.loadURDF(cfg.plane_urdf_path, flags=self.enable_cache)
        
        # parameters
        self.timestep = 1. / cfg.frequency
        self.timeout = cfg.timeout      # maximum time for planning

    
    # ----------------------------------------------------------------
    # A few basic functions
    # ----------------------------------------------------------------

    def get_client_id(self):
        return self.client_id
    
    def disconnect(self):
        p.disconnect(physicsClientId=self.client_id)
        print("-" * 20 + "\n" + "The client is disconnected!" + "\n" + "-" * 20)

    def wait(self, x):  # seconds
        time.sleep(x)
        print("-" * 20 + "\n" + "The client has waitted {} seconds".format(x))

    def run(self, x):  # steps
        for _ in range(x):
            p.stepSimulation(physicsClientId=self.client_id)
            time.sleep(self.timestep)
        print("-" * 20 + "\n" + "The client has runned {} simulation steps".format(x)) 
        

    # ----------------------------------------------------------------
    # Load scene
    # ----------------------------------------------------------------
    
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
            fileName=model_path,
            basePosition=object_position,
            baseOrientation=object_orientation,
            globalScaling=scale,
            useFixedBase=fixed_base,
            physicsClientId=self.client_id,
            flags=self.enable_cache
        )
        
        setattr(
            self,
            obj_name,
            object_id
        )
        
        self.wait(self.timestep)
        
        return object_id


    def create_scene(self, json_path):
        """
        Import the complete environment from the environment file based on the basic environment
        
        Args:
            json_path(str): scene json file path
        
        """
        
        with open(json_path, 'r') as f:
            scene_data = json.load(f)
        
        for object in scene_data:
            
            object_orientation = [eval(i) if isinstance(i, str) else i for i in object['object_orientation']]

            self.load_object(
                object['model_path'],
                object['object_position'],
                object_orientation,
                object['scale'],
                object['obj_name'],
                object['fixed_base']
            )
        
        print(f'success load scene from {json_path}')
    
    
    # ----------------------------------------------------------------
    # object joint info / operate
    # ----------------------------------------------------------------
    
    def print_object_joint_info(self, object_id):
        """
        Get object's joint info
        """
        
        num_joints = p.getNumJoints(object_id, physicsClientId=self.client_id)
        print("-" * 20 + "\n" + "The object {} has {} joints".format(object_id, num_joints))
        for i in range(num_joints):
            joint_info = p.getJointInfo(object_id, i, physicsClientId=self.client_id)
            joint_name = joint_info[1]
            joint_state = p.getJointState(object_id, i, physicsClientId=self.client_id)
            joint_angle = joint_state[0]
            print("Joint index:{}, name:{}, angle:{}".format(i, joint_name, joint_angle))
    
    def change_object_joint_angle(
        self, object_name, joint_index, target_position, max_force=5
    ):
        """
        Change the state of a specific joint of the object.

        Args:
        object_id (int): The id of the object.
        joint_index (int): The index of the joint to be changed.
        target_position (float): The target position of the joint in radians.
        max_force (float): The maximum force to be applied to achieve the target position.
        """
        
        object_id = getattr(self, object_name)
        p.setJointMotorControl2(
            bodyUniqueId=object_id,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_position,
            force=max_force,
            physicsClientId=self.client_id,
        )
        
        self.run(240)
    
    
    # ----------------------------------------------------------------
    # Get info from environment
    # ----------------------------------------------------------------

    def get_bounding_box(self, object_id):
        """
        This function retrieves the bounding box for a given object in the PyBullet simulation environment.

        Args:
            object_id (int): The ID of the object in the PyBullet simulation.
        Prints:
            The function prints the minimum and maximum x, y, z coordinates of the bounding box of the object.
        """
        
        link_ids = [i for i in range(-1, p.getNumJoints(object_id, physicsClientId=self.client_id))]

        aabb_bounds = np.array([
            p.getAABB(object_id, link_id, physicsClientId=self.client_id)
            for link_id in link_ids
        ])

        min_bounds = aabb_bounds[:, 0, :]
        max_bounds = aabb_bounds[:, 1, :]
        min_x, min_y, min_z = np.min(min_bounds, axis=0)
        max_x, max_y, max_z = np.max(max_bounds, axis=0)

        return [min_x, min_y, min_z, max_x, max_y, max_z]
