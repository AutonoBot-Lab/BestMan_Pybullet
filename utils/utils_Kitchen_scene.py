"""
@Description :   A few functions that load objects in kitchen, where objects are form others' side
@Author      :   Yan Ding 
@Time        :   2023/08/30 23:01:42
"""

"""
Get the utils module path
"""
import sys
import os

current_path = os.path.abspath(__file__)
parent_path = os.path.dirname(current_path)
sys.path.append(parent_path)

import pybullet as p
import pybullet_data
import math
import time
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import random
import xml.etree.ElementTree as ET

from utils_PbVisualizer import PbVisualizer
from utils_PbClient import PbClient
from utils_PIDController import PIDController

"""
Add kitchen
"""

class Kitchen:
    def __init__(self, pb_client, lisdf_id):
        """
        Initialize the kitchen environment.

        Parameters:
            pb_client (object): The pybullet client object.
            lisdf_id (str): different lisdf file.

        Attributes:
            pb_client (object): The pybullet client object.
            client_id (int): The client id returned by the pybullet client.
        """
        self.pb_client = pb_client
        self.client_id = self.pb_client.get_client()
        
        if lisdf_id == 0:
            lisdf_file = './Kitchen_models/scenes/kitchen_basics.lisdf'
        elif lisdf_id == 1:
            lisdf_file = './Kitchen_models/scenes/kitchen_counter.lisdf'
        elif lisdf_id == 2:
            lisdf_file = './Kitchen_models/scenes/kitchen_lunch.lisdf'
        else:
            print('Uknown lisdf_id, please input 0 or 1 or 2')
            sys.exit()
        models, xyz, point_to = self.parse_lisdf(lisdf_file)
        cameraDistance, cameraYaw, cameraPitch = self.compute_camera_angles(xyz, point_to)
        self.load_models(models)

        # Set the vertical view after loading the models # TODO: other's file is not accurate
        # self.pb_client.enable_vertical_view(height=cameraDistance, position=point_to, yaw=cameraYaw, pitch=cameraPitch)
    
    def parse_lisdf(self, file_path):
        """
        Parse the lisdf file to extract models and camera information.

        Parameters:
            file_path (str): Path to the lisdf file.

        Returns:
            list: List of models with their name, uri, pose and scale.
            list: XYZ coordinates of the camera.
            list: Point to coordinates for the camera.
        """
        tree = ET.parse(file_path)
        root = tree.getroot()

        models = []
        for include in root.iter('include'):
            model = {
                'name': include.get('name'),
                'uri': include.find('uri').text,
                'pose': include.find('pose').text,
                'scale': float(include.find('scale').text)
            }
            models.append(model)
        
        gui = root.find('.//gui')
        camera = gui.find('camera')
        xyz = list(map(float, camera.find('xyz').text.split()))
        point_to = list(map(float, camera.find('point_to').text.split()))

        return models, xyz, point_to
    
    def compute_camera_angles(self, camera_pos, target_pos):
        """
        Compute the camera angles from camera position and target position.

        Parameters:
            camera_pos (list): XYZ coordinates of the camera.
            target_pos (list): XYZ coordinates of the target position.

        Returns:
            float: The distance of the camera from the target.
            float: The yaw angle of the camera.
            float: The pitch angle of the camera.
        """
        dx = target_pos[0] - camera_pos[0]
        dy = target_pos[1] - camera_pos[1]
        dz = target_pos[2] - camera_pos[2]

        cameraDistance = math.sqrt(dx*dx + dy*dy + dz*dz)
        cameraYaw = math.degrees(math.atan2(dy, dx))
        cameraPitch = math.degrees(math.atan2(dz, math.sqrt(dx*dx + dy*dy)))

        return cameraDistance, cameraYaw, cameraPitch

    def load_models(self, models):
        """
        Load the models in the pybullet environment.

        Parameters:
            models (list): List of models with their name, uri, pose and scale.
        """
        for model in models:
            name = model['name']
            relative_url = model['uri']
            modified_url = relative_url.replace('../models', 'Kitchen_models/models')
            absolute_url = os.path.join(os.getcwd(), modified_url)
            pose = list(map(float, model['pose'].split()))
            scale = model['scale']
            self.pb_client.load_object(absolute_url, pose[:3], pose[3:], scale, name, fixed_base=True)
            
            print('-' * 20 + '\n' + 'name:{}; url:{}; pose:{}; scale:{}'.format(name, absolute_url, pose, scale))