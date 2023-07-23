import pybullet as p
import pybullet_data
import math
import time
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import random

"""
Add kitchen
"""

class Kitchen:
    def __init__(self):
        self.kitchen_id = p.loadURDF(
            "./URDF_models/kitchen_description/urdf/kitchen_part_right_gen_convex.urdf",
            basePosition=[4, 2, 1.477],
            baseOrientation=p.getQuaternionFromEuler([0, 0, math.pi]),
            useFixedBase=True,
        )
        self.drawer_to_joint_id = {
            1: 18,
            2: 22,
            3: 27,
            4: 31,
            5: 37,
            6: 40,
            7: 48,
            8: 53,
            9: 56,
            10: 58,
            11: 14,
        }
        self.drawer_to_joint_limits = {
            1: (0, 1.57),
            2: (-1.57, 0),
            3: (-1.57, 0),
            4: (0, 1.57),
            5: (0.0, 0.4),
            6: (0.0, 0.4),
            7: (0, 1.57),
            8: (-1.57, 0),
            9: (0.0, 0.4),
            10: (0.0, 0.4),
            11: (0, 1.57),
        }
        print(
            "-" * 20
            + "\n"
            + "drawer id in the kithen: {}".format(self.drawer_to_joint_id)
        )

    def open_drawer(self, drawer_id):
        joint_id = self.drawer_to_joint_id[drawer_id]
        open_angle = self.drawer_to_joint_limits[drawer_id][1]
        p.setJointMotorControl2(
            bodyIndex=self.kitchen_id,
            jointIndex=joint_id,
            controlMode=p.POSITION_CONTROL,
            targetPosition=open_angle,
            maxVelocity=0.5,
        )
        for _ in range(100):
            p.stepSimulation()
            time.sleep(1.0 / 100.0)

    def close_drawer(self, drawer_id):
        joint_id = self.drawer_to_joint_id[drawer_id]
        close_angle = self.drawer_to_joint_limits[drawer_id][0]
        p.setJointMotorControl2(
            bodyIndex=self.kitchen_id,
            jointIndex=joint_id,
            controlMode=p.POSITION_CONTROL,
            targetPosition=close_angle,
            maxVelocity=0.5,
        )
        for _ in range(100):
            p.stepSimulation()
            time.sleep(1.0 / 100.0)
