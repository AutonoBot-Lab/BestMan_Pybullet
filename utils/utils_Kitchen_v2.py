"""
@Description :   kitchen 2
@Author      :   Yan Ding 
@Time        :   2024/01/07 16:51:36
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
# from utils_PIDController import PIDController

"""
Add kitchen
"""

class Kitchen:
    def __init__(self, pb_client):
        self.pb_client = pb_client
        self.client_id = self.pb_client.get_client()
        
        self.object_ids = [] # store object id in loaded kitchen scene

        print('!'*50)
        self.elementG1_id = self.pb_client.load_object(
            model_path = "./URDF_models/test/model.urdf",
            object_position=[1.5, 2.8, 0],
            object_orientation=[0, 0.0, math.pi*1.5],
            scale=1,
            obj_name='elementG1',
            fixed_base=True,
        )
        self.object_ids.append("elementG2_id")

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
        self.object_ids.append("elementA_id")
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
            1: (0, math.pi/2.0),
            4: (0, math.pi/2.0),
            7: (0, math.pi/2.0),
            5: (0.0, 0.4),
            6: (0.0, 0.4),
            9: (0.0, 0.4),
            10: (0.0, 0.4),
            11: (0, math.pi/2.0),
            2: (0, -math.pi/2.0),
            3: (0, -math.pi/2.0),
            8: (0, -math.pi/2.0),
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
        self.elementB1_id = self.pb_client.load_object(
            model_path="./Kitchen_models/models_yan/elementB/model.urdf",
            object_position=[4.1, 4.55, 0.55],
            object_orientation=[0, 0, math.pi/2*3],
            scale=1.1,
            obj_name='elementB1',
            fixed_base=True,
        )
        self.object_ids.append("elementB1_id")

        self.elementB2_id = self.pb_client.load_object(
            model_path="./Kitchen_models/models_yan/elementB/model.urdf",
            object_position=[4.1, 5.25, 0.55],
            object_orientation=[0, 0, math.pi/2*3],
            scale=1.1,
            obj_name='elementB2',
            fixed_base=True,
        )
        self.object_ids.append("elementB2_id")

        # ----------------------------------------------------------------
        #  This is Element C (i.e., a dishwasher)
        # ----------------------------------------------------------------
        self.elementC_id = self.pb_client.load_object(
            model_path="./Kitchen_models/models/Dishwasher/2085/mobility.urdf",
            object_position=[3.85, 3.2, 0.35],
            object_orientation=[0, 0, 0],
            scale=0.75,
            obj_name='elementC',
            fixed_base=True,
        )
        self.object_ids.append("elementC_id")
        self.elementC_drawer_to_joint_id = {
            1: 1,
        }
        self.elementC_drawer_to_joint_limits = {
            1: (0, math.pi/2.0),
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
            model_path="./Kitchen_models/models/Microwave/7128/mobility.urdf",
            object_position=[4.0, 2.9, 1.1],
            object_orientation=[0, 0, 0],
            scale=0.5,
            obj_name='elementD',
            fixed_base=True,
        )
        self.object_ids.append("elementD_id")
        self.elementD_drawer_to_joint_id = {
            1: 1,
        }
        self.elementD_drawer_to_joint_limits = {
            1: (0, math.pi/2.0),
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
            model_path="./Kitchen_models/models/Fridge/10144/mobility.urdf",
            object_position=[4.1, 5.42, 1.055],
            object_orientation=[0, 0, 0],
            scale=1.1,
            obj_name='elementE',
            fixed_base=True,
        )
        self.object_ids.append("elementE_id")
        self.elementE_drawer_to_joint_id = {
            1: 1
        }
        self.elementE_drawer_to_joint_limits = {
            1: (0, math.pi/2.0),
        }
        print(
            "-" * 20
            + "\n"
            + "Element E's drawer id in kithen: {}".format(
                self.elementE_drawer_to_joint_id
            )
        )

        # ----------------------------------------------------------------
        # This is Element F (i.e., a table)
        # ----------------------------------------------------------------
        self.elementF_id = self.pb_client.load_object(
            model_path="./URDF_models/furniture_table_rectangle_high/table.urdf",
            object_position=[1.5, 2, 0],
            object_orientation=[0, 0, math.pi/2.0],
            scale=0.75,
            obj_name='elementF',
            fixed_base=True,
        )
        self.object_ids.append("elementF_id")

        # ----------------------------------------------------------------
        # This is Element G1 (i.e., chairs)
        # ----------------------------------------------------------------
        # self.elementG1_id = self.pb_client.load_object(
        #     model_path = "./URDF_models/furniture_chair/model.urdf",
        #     object_position=[1.5, 2.8, 0],
        #     object_orientation=[math.pi/2.0*3, 0.0, math.pi*1.5],
        #     scale=1,
        #     obj_name='elementG1',
        #     fixed_base=True,
        # )
        # self.object_ids.append("elementG2_id")

        self.elementG2_id = self.pb_client.load_object(
            model_path = "./URDF_models/furniture_chair/model.urdf",
            object_position=[2.1, 2.2, 0],
            object_orientation=[math.pi/2.0*3, 0.0, math.pi],
            scale=1,
            obj_name='elementG2',
            fixed_base=True,
        )
        self.object_ids.append("elementG2_id")

        self.elementG3_id = self.pb_client.load_object(
            model_path = "./URDF_models/furniture_chair/model.urdf",
            object_position=[1.5, 1.0, 0],
            object_orientation=[math.pi/2.0*3, 0.0, math.pi/2.0],
            scale=1,
            obj_name='elementG3',
            fixed_base=True,
        )
        self.object_ids.append("elementG3_id")

        self.elementG4_id = self.pb_client.load_object(
            model_path = "./URDF_models/furniture_chair/model.urdf",
            object_position=[2.1, 2.7, 0],
            object_orientation=[math.pi/2.0*3, 0.0, math.pi],
            scale=1,
            obj_name='elementG4',
            fixed_base=True,
        )
        self.object_ids.append("elementG4_id")

        # ----------------------------------------------------------------
        # This is Element H (i.e., small objects)
        # ----------------------------------------------------------------
        self.elementH1_id = self.pb_client.load_object(
            model_path = "./URDF_models/plastic_peach/model.urdf",
            object_position = [1.8, 2.0, 0.8],
            object_orientation = [0, 0, math.pi],
            scale=1.1,
            obj_name='Peach',
            fixed_base=False,
        )
        self.object_ids.append("elementH1_id")

        self.elementH2_id = self.pb_client.load_object(
            model_path = "./URDF_models/bowl/model.urdf",
            object_position = [1.7, 2.2, 0.8],
            object_orientation = [0, 0, math.pi],
            scale=1.1,
            obj_name='Bowl_Target',
            fixed_base=False,
        )
        self.object_ids.append("elementH2_id")

        self.elementH3_id = self.pb_client.load_object(
            model_path = "./URDF_models/plastic_banana/model.urdf",
            object_position = [1.6, 1.8, 0.8],
            object_orientation = [0, 0, math.pi],
            scale=1.0,
            obj_name='Banana',
            fixed_base=False,
        )
        self.object_ids.append("elementH3_id")

        self.elementH4_id = self.pb_client.load_object(
            model_path = "./URDF_models/plastic_orange/model.urdf",
            object_position = [1.7, 1.7, 0.8],
            object_orientation = [0, 0, 0],
            scale=1.0,
            obj_name='Bread',
            fixed_base=False,
        )
        self.object_ids.append("elementH4_id")

        self.elementH5_id = self.pb_client.load_object(
            model_path = "./URDF_models/plastic_lemon/model.urdf",
            object_position = [1.8, 1.8, 0.8],
            object_orientation = [0, 0, 0],
            scale=1.0,
            obj_name='Lemon',
            fixed_base=False,
        )
        self.object_ids.append("elementH5_id")

        self.elementH6_id = self.pb_client.load_object(
            model_path = "./URDF_models/round_plate_2/model.urdf",
            object_position = [1.3, 1.8, 0.8],
            object_orientation = [0, 0, 0],
            scale=1.0,
            obj_name='round_plate_2',
            fixed_base=False,
        )
        self.object_ids.append("elementH6_id")

        self.elementH7_id = self.pb_client.load_object(
            model_path = "./URDF_models/mug/model.urdf",
            object_position = [3.9, 4.6, 1.0],
            object_orientation = [0, 0, 0],
            scale=1.3,
            obj_name='mug',
            fixed_base=False,
        )
        self.object_ids.append("elementH7_id")

        self.elementH8_id = self.pb_client.load_object(
            model_path = "./URDF_models/toothpaste_1/model.urdf",
            object_position = [3.9, 4.9, 1.0],
            object_orientation = [math.pi, 0, 0],
            scale=1.0,
            obj_name='toothpaste_1',
            fixed_base=False,
        )
        self.object_ids.append("elementH8_id")  

        self.elementH9_id = self.pb_client.load_object(
            model_path = "./URDF_models/pudding_box/model.urdf",
            object_position = [4.0, 0.8, 1.0],
            object_orientation = [math.pi, 0, 0],
            scale=1.0,
            obj_name='pudding_box',
            fixed_base=False,
        )
        self.object_ids.append("elementH9_id")

        self.elementH10_id = self.pb_client.load_object(
            model_path = "./URDF_models/sugar_box/model.urdf",
            object_position = [3.8, 0.7, 1.0],
            object_orientation = [math.pi, 0, 0],
            scale=1.0,
            obj_name='sugar_box',
            fixed_base=False,
        )
        self.object_ids.append("elementH10_id")

        # self.elementH11_id = self.pb_client.load_object(
        #     model_path = "./Kitchen_models/models/MilkBottle/4043/mobility.urdf",
        #     object_position = [4.1, 5.42, 1.5],
        #     object_orientation = [0.0, 0, 0],
        #     scale=0.1,
        #     obj_name='MilkBottle',
        #     fixed_base=False,
        # )
        # self.object_ids.append("elementH11_id")

        # ----------------------------------------------------------------
        # Set element A B C D E colors
        # ----------------------------------------------------------------
        self.visualizer = PbVisualizer(pb_client)
        self.visualizer.set_elementA_visual_color(self.elementA_id)
        self.visualizer.set_elementB_visual_color(self.elementB1_id)
        self.visualizer.set_elementB_visual_color(self.elementB2_id)
        self.visualizer.set_elementC_visual_color(self.elementC_id)
        self.visualizer.set_elementD_visual_color(self.elementD_id)
        self.visualizer.set_elementE_visual_color(self.elementE_id)

    # ----------------------------------------------------------------
    # Open drawer
    # ----------------------------------------------------------------
    def open_it(self, elementName, drawer_id, open_angle=None):
        if elementName == "elementA":
            joint_id = self.elementA_drawer_to_joint_id[drawer_id]
            if open_angle is None:
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
            if open_angle is None:
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
            if open_angle is None:
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
            if open_angle is None:
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
    def close_it(self, elementName, drawer_id, open_angle=None):
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