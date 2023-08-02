import pybullet as p
import time

def run(x):
    for _ in range(x):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

# Connect pybullet
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.81)

# Load refrigerator model
applianceStartPos = [0, 0, 0]
applianceStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
# robotId = p.loadURDF("./Kitchen/kitchen_model/kitchen_model.urdf", applianceStartPos, applianceStartOrientation)
robotId = p.loadURDF("./Kitchen/kitchen_assembly/kitchen_assembly.urdf", applianceStartPos, applianceStartOrientation)

# Get joint info of microwave model
num_joints = p.getNumJoints(robotId)
for joint_index in range(num_joints):
    joint_info = p.getJointInfo(robotId, joint_index)
    joint_name = joint_info[1].decode("utf-8")
    joint_id = joint_info[0]
    print(f"joint:{joint_name}, id:{joint_id}")

run(240 * 10)

p.disconnect()

