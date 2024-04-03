import pybullet as p
import pybullet_data
import math
import time

# 初始化 PyBullet
client_id = p.connect(p.GUI, options=f"--width=1920 --height=1080")

# 加载物理世界
p.setGravity(0, 0, -9.8)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")

# 加载刚性物体
object_path = '/GithubCode/BestMan_Pybullet/Kitchen_models/models_yan/elementA/urdf/kitchen_part_right_gen_convex.urdf'
# object_path = '/GithubCode/BestMan_Pybullet/Kitchen_models/models/Fridge/10144/mobility.urdf'
# object_path = '/GithubCode/BestMan_Pybullet/refactor/Asset/URDF_models/furniture_chair/model.urdf'
object_orientation = [0, 0, math.pi]
object_orientation = p.getQuaternionFromEuler(
    object_orientation, physicsClientId=client_id
)
object_id = p.loadURDF(
    fileName=object_path,
    basePosition=[4, 2, 1.477],
    baseOrientation=object_orientation,
    globalScaling=1.0,
    useFixedBase=True,
    physicsClientId=client_id
)

num_joints = p.getNumJoints(object_id, physicsClientId=client_id)
print(
    "-" * 20
    + "\n"
    + "The object {} has {} joints".format(object_id, num_joints)
)
for i in range(num_joints):
    joint_info = p.getJointInfo(object_id, i, physicsClientId=client_id)
    joint_name = joint_info[1]
    joint_state = p.getJointState(
        object_id, i, physicsClientId=client_id
    )
    joint_angle = joint_state[0]
    print(
        "Joint index:{}, name:{}, angle:{}".format(i, joint_name, joint_angle)
    )
    
p.setJointMotorControl2(
    bodyUniqueId=object_id,
    jointIndex=19,
    controlMode=p.POSITION_CONTROL,
    targetPosition=math.pi / 2.0,
    maxVelocity=1.0
)

for _ in range(240 * 5):
    p.stepSimulation(physicsClientId=client_id)
    time.sleep(1.0 / 480)

# 断开 PyBullet
time.sleep(1000)
p.disconnect()