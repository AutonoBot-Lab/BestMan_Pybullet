"""
@Description :   A few functions used in bestman robot, where the robot has a base and an arm.
@Author      :   Yan Ding 
@Time        :   2023/08/30 20:43:44
"""

import math
import pybullet as p
from .Bestman_sim import Bestman_sim


class Bestman_sim_ur5e_vacuum_long(Bestman_sim):
    
    def __init__(self, client, visualizer,  cfg):
        """Bestman_sim for ur5e arm.
        """
        
        # Init parent class: BestMan_sim
        super().__init__(client, visualizer,  cfg)
    
        self.gripper_id = None
    
    # ----------------------------------------------------------------
    # functions for gripper
    # ----------------------------------------------------------------

    def sim_active_gripper(self, object, value):
        """
        Activate or deactivate the gripper.

        Args:
            object_id (init): ID of the object related to gripper action.
            value (int): 0 or 1, where 0 means deactivate (ungrasp) and 1 means activate (grasp).
        """

        if isinstance(object, str):
            if hasattr(self.client, object):
                object_id = getattr(self.client, object)
            else:
                raise AttributeError(f"scene has not {object} object!")
        else:
            object_id = object
          
        # gripper_status = {"ungrasp": 0, "grasp": 1}
        # gripper_value = (
        #     gripper_status["grasp"] if value == 1 else gripper_status["ungrasp"]
        # )
        
        # 
        if value == 0 and self.gripper_id != None:
            p.removeConstraint(self.gripper_id, physicsClientId=self.client_id)
            self.gripper_id = None
            # for _ in range(self.frequency):
            #     p.stepSimulation(physicsClientId=self.client_id)
            print("-" * 20 + "\n" + "Gripper has been deactivated!")
        
        
        if value == 1 and self.gripper_id == None:
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
                print("-" * 20 + "\n" + "Gripper has been activated!")
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
                print("-" * 20 + "\n" + "Gripper has been activated!")
        
        self.client.run(240)