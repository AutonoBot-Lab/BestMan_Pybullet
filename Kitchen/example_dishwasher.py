import pybullet as p
import time


def run(x):
    for _ in range(x):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)


def manipulate_appliance(joint_id, angle):
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=joint_id,
        controlMode=p.POSITION_CONTROL,
        targetPosition=angle,
        maxVelocity=0.5,
    )


# Connect pybullet
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.81)

# Load dishwasher model
applianceStartPos = [0, 0, 0]
applianceStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF(
    "./Kitchen/dishwasher/dishwasher.urdf", applianceStartPos, applianceStartOrientation
)

# Get joint info of dishwasher model
num_joints = p.getNumJoints(robotId)
for joint_index in range(num_joints):
    joint_info = p.getJointInfo(robotId, joint_index)
    joint_name = joint_info[1].decode("utf-8")
    joint_id = joint_info[0]
    print(f"joint:{joint_name}, id:{joint_id}")

# drawer_to_joint_limits = {1: (1.57, 0), 2: (-0.3, 0), 3: (-0.3, 0)}

joint_id = 1  # dishwasher's door
open_angle = 1.57
manipulate_appliance(joint_id, open_angle)
run(240 * 5)
# close_angle = 0.0
# manipulate_appliance(joint_id, close_angle)
# run(240 * 5)

joint_id = 2  # dishwasher's drawer 1
open_angle = -0.3
manipulate_appliance(joint_id, open_angle)
run(240 * 5)
# close_angle = 0.0
# manipulate_appliance(joint_id, close_angle)
# run(240 * 5)

joint_id = 3  # dishwasher's drawer 2
open_angle = -0.3
manipulate_appliance(joint_id, open_angle)
run(240 * 5)
# close_angle = 0.0
# manipulate_appliance(joint_id, close_angle)
# run(240 * 5)


p.disconnect()
