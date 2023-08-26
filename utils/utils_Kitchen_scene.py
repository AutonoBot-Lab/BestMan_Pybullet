import pybullet as p
import pybullet_data
import math
import time
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import random

from utils.utils_PbVisualizer import PbVisualizer
from utils.utils_PbClient import PbClient
from utils.utils_PIDController import PIDController

import xml.etree.ElementTree as ET
import os

"""
Add kitchen
"""

class Kitchen:
    def __init__(self, pb_client, lisdf_file):
        self.pb_client = pb_client
        self.client_id = self.pb_client.get_client()
        
        models, xyz, point_to = self.parse_lisdf(lisdf_file)
        cameraDistance, cameraYaw, cameraPitch = self.compute_camera_angles(xyz, point_to)
        self.load_models(models)

        # Set the vertical view after loading the models
        self.pb_client.enable_vertical_view(height=cameraDistance, position=point_to, yaw=cameraYaw, pitch=cameraPitch)
    
    def parse_lisdf(self, file_path):
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
        dx = target_pos[0] - camera_pos[0]
        dy = target_pos[1] - camera_pos[1]
        dz = target_pos[2] - camera_pos[2]

        cameraDistance = math.sqrt(dx*dx + dy*dy + dz*dz)
        cameraYaw = math.degrees(math.atan2(dy, dx))
        cameraPitch = math.degrees(math.atan2(dz, math.sqrt(dx*dx + dy*dy)))

        return cameraDistance, cameraYaw, cameraPitch

    def load_models(self, models):
        for model in models:
            name = model['name']
            relative_url = model['uri']
            modified_url = relative_url.replace('../models', 'Kitchen/models')
            absolute_url = os.path.join(os.getcwd(), modified_url)
            pose = list(map(float, model['pose'].split()))
            scale = model['scale']
            print('-' * 20 + '\n' + 'name:{}; url:{}; pose:{}; scale:{}'.format(name, absolute_url, pose, scale))
            self.pb_client.load_object(absolute_url, pose[:3], pose[3:], scale, name)