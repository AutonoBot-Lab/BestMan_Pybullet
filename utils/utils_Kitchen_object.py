"""
@Description :   A few functions that load objects in kitchen, where objects are form Yan's side
@Author      :   Yan Ding 
@Time        :   2023/08/30 23:01:42
"""


import pybullet as p
import pybullet_data
import math
import time
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import random

"""
Get the utils module path
"""
import sys
import os
# customized package
current_path = os.path.abspath(__file__)
utils_path = os.path.dirname(current_path)
if os.path.basename(utils_path) != 'utils':
    raise ValueError('Not add the path of folder "utils", please check again!')
sys.path.append(utils_path)
from utils_PbVisualizer import PbVisualizer
from utils_PbClient import PbClient
from utils_PIDController import PIDController

"""
Add kitchen
"""

class Kitchen:
    def __init__(self, pb_client):
        self.pb_client = pb_client
        self.client_id = self.pb_client.get_client()
        
        # ----------------------------------------------------------------
        # This is Element A, where there are a oven, and a few drawers
        # ----------------------------------------------------------------
        self.elementA_id = self.pb_client.load_object(
            model_path="./Kitchen_models/models_yan/elementA/urdf/kitchen_part_right_gen_convex.urdf",
            object_position=[4, 2, 1.477],
            object_orientation=[0, 0, math.pi],
            scale=1.0,
            obj_name='elementA',
            fixed_base=True,
        )
        self.elementA_drawer_to_joint_id = {
            1: 17,
            2: 21,
            3: 26,
            4: 30,
            5: 36,
            6: 39,
            7: 47,
            8: 52,
            9: 55,
            10: 57,
            11: 13,
        }
        self.elementA_drawer_to_joint_limits = {
            1: (0, 1.5),
            2: (0, -1.5),
            3: (0, -1.5),
            4: (0, 1.5),
            5: (0.0, 0.4),
            6: (0.0, 0.4),
            7: (0, 1.5),
            8: (0, -1.5),
            9: (0.0, 0.4),
            10: (0.0, 0.4),
            11: (0, 1.5),
        }
        print(
            "-" * 20
            + "\n"
            + "Element A's drawer id in kithen: {}".format(
                self.elementA_drawer_to_joint_id
            )
        )

        # ----------------------------------------------------------------
        # This is Element B, where there are a sink, and a few container
        # ----------------------------------------------------------------
        self.elementB_id = self.pb_client.load_object(
            model_path="./Kitchen_models/models_yan/elementB/urdf/kitchen_assembly.urdf",
            object_position=[4.3, 5.95, 0],
            object_orientation=[0, 0, math.pi],
            scale=1.0,
            obj_name='elementB',
            fixed_base=True,
        )

        # ----------------------------------------------------------------
        #  This is Element C (i.e., a dishwasher)
        # ----------------------------------------------------------------
        self.elementC_id = self.pb_client.load_object(
            model_path="./Kitchen_models/models_yan/elementC/dishwasher.urdf",
            object_position=[3.95, 3.08, 0.43],
            object_orientation=[0, 0, - math.pi / 2.0],
            scale=1.0,
            obj_name='elementC',
            fixed_base=True,
        )
        self.elementC_drawer_to_joint_id = {
            1: 1,
            2: 2,
            3: 3,
        }
        self.elementC_drawer_to_joint_limits = {
            1: (0, 1.5),
            2: (0, -0.3),
            3: (0, -0.3),
        }
        print(
            "-" * 20
            + "\n"
            + "Element C's drawer id in kithen: {}".format(
                self.elementC_drawer_to_joint_id
            )
        )

        # ----------------------------------------------------------------
        # This is Element D (i.e., a microwave)
        # ----------------------------------------------------------------
        self.elementD_id = self.pb_client.load_object(
            model_path="./Kitchen_models/models_yan/elementD/microwave.urdf",
            object_position=[4.0, 2.9, 0.95],
            object_orientation=[0, 0, math.pi / 2.0],
            scale=1.0,
            obj_name='elementD',
            fixed_base=True,
        )
        self.elementD_drawer_to_joint_id = {
            1: 1,
        }
        self.elementD_drawer_to_joint_limits = {
            1: (0, -1.5),
        }
        print(
            "-" * 20
            + "\n"
            + "Element D's drawer id in kithen: {}".format(
                self.elementD_drawer_to_joint_id
            )
        )

        # ----------------------------------------------------------------
        # This is Element E (i.e., a refrigerator)
        # ----------------------------------------------------------------
        self.elementE_id = self.pb_client.load_object(
            model_path="./Kitchen_models/models_yan/elementE/refrigerator.urdf",
            object_position=[4.1, 6.42, 0.05],
            object_orientation=[0, 0, -math.pi / 2.0],
            scale=1.0,
            obj_name='elementE',
            fixed_base=True,
        )
        self.elementE_drawer_to_joint_id = {
            1: 1,
            2: 2,
        }
        self.elementE_drawer_to_joint_limits = {
            1: (0, -1.5),
            2: (0, -1.5),
        }
        print(
            "-" * 20
            + "\n"
            + "Element E's drawer id in kithen: {}".format(
                self.elementE_drawer_to_joint_id
            )
        )

        # ----------------------------------------------------------------
        # Set element A B C D E colors
        # ----------------------------------------------------------------
        self.visualizer = PbVisualizer(pb_client)
        self.visualizer.set_elementA_visual_color(self.elementA_id)
        self.visualizer.set_elementB_visual_color(self.elementB_id)
        self.visualizer.set_elementC_visual_color(self.elementC_id)
        self.visualizer.set_elementD_visual_color(self.elementD_id)
        self.visualizer.set_elementE_visual_color(self.elementE_id)

    # ----------------------------------------------------------------
    # Open drawer
    # ----------------------------------------------------------------
    def open_it(self, elementName, drawer_id):
        if elementName == "elementA":
            joint_id = self.elementA_drawer_to_joint_id[drawer_id]
            open_angle = self.elementA_drawer_to_joint_limits[drawer_id][1]
            print('elementA: joint_id:{}, open_angle:{}'.format(joint_id, open_angle))
            p.setJointMotorControl2(
                bodyIndex=self.elementA_id,
                jointIndex=joint_id,
                controlMode=p.POSITION_CONTROL,
                targetPosition=open_angle,
                maxVelocity=1.0,
            )
        elif elementName == "elementC":
            joint_id = self.elementC_drawer_to_joint_id[drawer_id]
            open_angle = self.elementC_drawer_to_joint_limits[drawer_id][1]
            print('elementC: joint_id:{}, open_angle:{}'.format(joint_id, open_angle))
            p.setJointMotorControl2(
                bodyIndex=self.elementC_id,
                jointIndex=joint_id,
                controlMode=p.POSITION_CONTROL,
                targetPosition=open_angle,
                maxVelocity=1.0,
            )
        elif elementName == "elementD":
            joint_id = self.elementD_drawer_to_joint_id[drawer_id]
            open_angle = self.elementD_drawer_to_joint_limits[drawer_id][1]
            print('elementD (open): joint_id:{}, open_angle:{}'.format(joint_id, open_angle))
            p.setJointMotorControl2(
                bodyIndex=self.elementD_id,
                jointIndex=joint_id,
                controlMode=p.POSITION_CONTROL,
                targetPosition=open_angle,
                maxVelocity=1.0,
            )
        elif elementName == "elementE":
            joint_id = self.elementE_drawer_to_joint_id[drawer_id]
            open_angle = self.elementE_drawer_to_joint_limits[drawer_id][1]
            print('elementE: joint_id:{}, open_angle:{}'.format(joint_id, open_angle))
            p.setJointMotorControl2(
                bodyIndex=self.elementE_id,
                jointIndex=joint_id,
                controlMode=p.POSITION_CONTROL,
                targetPosition=open_angle,
                maxVelocity=1.0,
            )
        self.pb_client.run(240 * 5)

    # ----------------------------------------------------------------
    # Close drawer
    # ----------------------------------------------------------------
    def close_drawer(self, elementName, drawer_id):
        if elementName == "elementA":
            joint_id = self.elementA_drawer_to_joint_id[drawer_id]
            close_angle = self.elementA_drawer_to_joint_limits[drawer_id][0]
            p.setJointMotorControl2(
                bodyIndex=self.elementA_id,
                jointIndex=joint_id,
                controlMode=p.POSITION_CONTROL,
                targetPosition=close_angle,
                maxVelocity=1.0,
            )
        elif elementName == "elementC":
            joint_id = self.elementC_drawer_to_joint_id[drawer_id]
            close_angle = self.elementC_drawer_to_joint_limits[drawer_id][0]
            p.setJointMotorControl2(
                bodyIndex=self.elementC_id,
                jointIndex=joint_id,
                controlMode=p.POSITION_CONTROL,
                targetPosition=close_angle,
                maxVelocity=1.0,
            )
            self.pb_client.run(240 * 5)
            
        elif elementName == "elementD":
            joint_id = self.elementD_drawer_to_joint_id[drawer_id]
            close_angle = self.elementD_drawer_to_joint_limits[drawer_id][0]
            print('elementD (close): joint_id:{}, open_angle:{}'.format(joint_id, close_angle))
            p.setJointMotorControl2(
                bodyIndex=self.elementD_id,
                jointIndex=joint_id,
                controlMode=p.POSITION_CONTROL,
                targetPosition=close_angle,
                maxVelocity=1.0,
            )
        elif elementName == "elementE":
            joint_id = self.elementE_drawer_to_joint_id[drawer_id]
            close_angle = self.elementE_drawer_to_joint_limits[drawer_id][0]
            p.setJointMotorControl2(
                bodyIndex=self.elementE_id,
                jointIndex=joint_id,
                controlMode=p.POSITION_CONTROL,
                targetPosition=close_angle,
                maxVelocity=1.0,
            )
        self.pb_client.run(240 * 5)
