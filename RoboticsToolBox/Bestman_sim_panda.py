"""
@Description :   A few functions used in bestman robot, where the robot has a base and an arm.
@Author      :   Yan Ding 
@Time        :   2023/08/30 20:43:44
"""

import math
import pybullet as p

from .Bestman_sim import Bestman_sim
from .Pose import Pose

class Bestman_sim_panda(Bestman_sim):
    
    def __init__(self, client, visualizer,  cfg):
        """BestMan_sim for panda arm.
        """
        
        # Init parent class: BestMan_sim
        super().__init__(client, visualizer,  cfg)
        
        # Create a gear constraint to keep the fingers symmetrically centered
        c = p.createConstraint(
            self.arm_id,
            9,
            self.arm_id,
            10,
            jointType=p.JOINT_GEAR,
            jointAxis=[1, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0]
        )
        
        # Constraint parameters
        p.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=50)
        
        # for i in range(p.getNumJoints(self.arm_id)):
        #     p.changeDynamics(self.arm_id, i, linearDamping=0, angularDamping=0)

    
    # ----------------------------------------------------------------
    # functions for gripper
    # ----------------------------------------------------------------
    
    def sim_active_gripper(self, value):
        """
        Activate or deactivate the gripper.

        Args:
            object_id (init): ID of the object related to gripper action.
            value (int): 0 or 1, where 0 means deactivate (ungrasp) and 1 means activate (grasp).
        """

        if value == 1:
            for i in [9, 10]:
                p.setJointMotorControl2(self.arm_id, i, p.POSITION_CONTROL, 0.04, force=10)
        elif value == 0:
            for i in [9, 10]:
                p.setJointMotorControl2(self.arm_id, i, p.POSITION_CONTROL, 0.001, force=10)
        else:
            raise(ValueError("gripper value must be 0 / 1 !"))
        
        self.client.run(30)
    
    
    # ----------------------------------------------------------------
    # functions for pick and place
    # ----------------------------------------------------------------
    
    def pick(self, object):
        object_id = self.client.resolve_object_id(object)
        position, _ = p.getBasePositionAndOrientation(object_id)
        goal_pose = Pose([position[0], position[1], position[2]+0.015], [0, math.pi, 0])
        self.move_end_effector_to_goal_pose(goal_pose)
        goal_pose = Pose([position[0], position[1], position[2]-0.005], [0, math.pi, 0])
        self.move_end_effector_to_goal_pose(goal_pose)
        self.sim_active_gripper(0)
    
    def place(self, goal_pose):
        init_pose = self.get_current_end_effector_pose()
        init_pos, _ = init_pose.position, init_pose.orientation
        goal_pos, goal_orn = goal_pose.position, goal_pose.orientation
        tmp_pose = Pose([init_pos[0], init_pos[1], goal_pos[2]], goal_orn)
        self.move_end_effector_to_goal_pose(tmp_pose)
        self.move_end_effector_to_goal_pose(goal_pose, 30)
        self.sim_active_gripper(1)
    
    def pick_place(self, object, goal_pose):
        self.pick(object)
        self.place(goal_pose)