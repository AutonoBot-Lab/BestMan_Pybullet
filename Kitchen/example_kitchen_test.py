import pybullet as p
import time
from utils import Bestman, Pose, Kitchen
import math
import time

# Connect pybullet
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.81)

# Load models
refrigeratorStartPos = [0.2, -0.45, 0]
refrigeratorOrientation = p.getQuaternionFromEuler([0, 0, 1.57])
robotId = p.loadURDF("./refrigerator/refrigerator.urdf", refrigeratorStartPos, refrigeratorOrientation)

microwave_ovenStartPos = [0.4, 1, 0.6]
microwave_ovenOrientation = p.getQuaternionFromEuler([0, 0, -1.57])
microwave_ovenId = p.loadURDF("./microwave-oven/microwave-oven.urdf", microwave_ovenStartPos, microwave_ovenOrientation)

dishwasherStartPos = [0.3, 4.7, 0.43]
dishwasherOrientation = p.getQuaternionFromEuler([0, 0, 1.57])
dishwasherId = p.loadURDF("./dishwasher/dishwasher.urdf", dishwasherStartPos, dishwasherOrientation)

Kitchen_modelStartPos = [0, 0, 0]
Kitchen_modelOrientation = p.getQuaternionFromEuler([0, 0, 0])
Kitchen_modelId = p.loadURDF("./kitchen_assembly/kitchen_assembly.urdf", Kitchen_modelStartPos, Kitchen_modelOrientation)

kitchen_id = Kitchen()

# open drawer
for i in range(10):
    drawer_id = i+1
    kitchen_id.open_drawer(drawer_id)
    time.sleep(0.1)

#close all drawers
for i in range(10):
    drawer_id = i+1
    kitchen_id.close_drawer(drawer_id)
    time.sleep(0.1)


# 获取机器人模型的关节数量
num_joints = p.getNumJoints(robotId)

# 遍历关节，获取每个关节的信息和ID号
for joint_index in range(num_joints):
    joint_info = p.getJointInfo(robotId, joint_index)
    joint_name = joint_info[1].decode("utf-8")  # 解码字节字符串为Unicode字符串
    joint_id = joint_info[0]
    print(f"关节名称: \n {joint_name}, ID号: {joint_id}")
drawer_to_joint_limits = {
    1: (-1.57, 0),
    2: (-1.57, 0),
    3: (1.57,0),
    4: (-0.3,0),
    5: (-0.3,0)
}

# while True:
joint_id = 1
open_angle = drawer_to_joint_limits[joint_id][0]
p.setJointMotorControl2(
    bodyIndex= microwave_ovenId,
    jointIndex=joint_id,
    controlMode=p.POSITION_CONTROL,
    targetPosition=open_angle,
    maxVelocity=0.5,
)

open_angle = drawer_to_joint_limits[joint_id][0]
p.setJointMotorControl2(
    bodyIndex= robotId,
    jointIndex=joint_id,
    controlMode=p.POSITION_CONTROL,
    targetPosition=open_angle,
    maxVelocity=0.5,
)

open_angle = drawer_to_joint_limits[3][0]
p.setJointMotorControl2(
    bodyIndex= dishwasherId,
    jointIndex=1,
    controlMode=p.POSITION_CONTROL,
    targetPosition=open_angle,
    maxVelocity=0.5,
)
for _ in range(1000):
    p.stepSimulation()
    time.sleep(1./100.)
open_angle = drawer_to_joint_limits[4][0]
p.setJointMotorControl2(
    bodyIndex= dishwasherId,
    jointIndex=2,
    controlMode=p.POSITION_CONTROL,
    targetPosition=open_angle,
    maxVelocity=0.5,
)

open_angle = drawer_to_joint_limits[5][0]
p.setJointMotorControl2(
    bodyIndex= dishwasherId,
    jointIndex= 3,
    controlMode=p.POSITION_CONTROL,
    targetPosition=open_angle,
    maxVelocity=0.5,
)

for _ in range(1000):
    p.stepSimulation()
    time.sleep(1./100.)

joint_id = 2
open_angle = drawer_to_joint_limits[joint_id][0]
p.setJointMotorControl2(
    bodyIndex= robotId,
    jointIndex=joint_id,
    controlMode=p.POSITION_CONTROL,
    targetPosition=open_angle,
    maxVelocity=0.5,
)

for _ in range(1000):
    p.stepSimulation()
    time.sleep(1./100.)

joint_id = 1
close_angle = drawer_to_joint_limits[joint_id][1]
p.setJointMotorControl2(
    bodyIndex= robotId,
    jointIndex=joint_id,
    controlMode=p.POSITION_CONTROL,
    targetPosition=close_angle,
    maxVelocity=0.5,
)

close_angle = drawer_to_joint_limits[joint_id][1]
p.setJointMotorControl2(
    bodyIndex= microwave_ovenId,
    jointIndex=joint_id,
    controlMode=p.POSITION_CONTROL,
    targetPosition=close_angle,
    maxVelocity=0.5,
)

close_angle = drawer_to_joint_limits[3][0]
p.setJointMotorControl2(
    bodyIndex= dishwasherId,
    jointIndex=2,
    controlMode=p.POSITION_CONTROL,
    targetPosition=close_angle,
    maxVelocity=0.5,
)
close_angle = drawer_to_joint_limits[3][0]
p.setJointMotorControl2(
    bodyIndex= dishwasherId,
    jointIndex=3,
    controlMode=p.POSITION_CONTROL,
    targetPosition=close_angle,
    maxVelocity=0.5,
)

for _ in range(1000):
    p.stepSimulation()
    time.sleep(1./100.)

close_angle = drawer_to_joint_limits[3][1]
p.setJointMotorControl2(
    bodyIndex= dishwasherId,
    jointIndex=joint_id,
    controlMode=p.POSITION_CONTROL,
    targetPosition=close_angle,
    maxVelocity=0.5,
)


joint_id = 2
close_angle = drawer_to_joint_limits[joint_id][1]
p.setJointMotorControl2(
    bodyIndex= robotId,
    jointIndex=joint_id,
    controlMode=p.POSITION_CONTROL,
    targetPosition=close_angle,
    maxVelocity=0.5,
)

for _ in range(1000):
    p.stepSimulation()
    time.sleep(1./100.)

# 断开连接
p.disconnect()

