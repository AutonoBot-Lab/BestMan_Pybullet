from utils.utils_Bestman import Bestman, Pose
from utils.utils_PbClient import PbClient
from utils.utils_PbVisualizer import PbVisualizer
from utils.utils_Kitchen_object import Kitchen
from utils.utils_PbOMPL import PbOMPL
import math
import time
import pybullet as p
import numpy as np

# This script demonstrates retrieving an object from inside a drawer using a vacuum tool.

# NOTE: test with vacuum
# fileName="./urdf/ur5e_vacuum.urdf",

drawer_id = 5
threshold_distance = 0.2  # TODO test
nav_target_position = [3.1, 2.4, 0]  # hard code
drop_position = [3.6, 2.4, 0.6]  # hard code
pose1 = [0, -1.57, 2.0, -1.57, -1.57, 0]
target_orientation = [0.0, math.pi / 2.0, 0.0]
joint_idx = [0, 1, 2, 3, 4, 5]

# load cleint
pb_client = PbClient(enable_GUI=True)
pb_client.enable_vertical_view(6, [0, 0, 0], yaw=-220, pitch=-13.5)

# load visualizer
pb_visualizer = PbVisualizer(pb_client)

# load robot
init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose, pb_client)

# load kitchen
kitchen_id = Kitchen(pb_client)

# load OMPL planner
ompl = PbOMPL(
    robot_id=demo.arm_id,
    joint_idx=joint_idx,
    obstacles=[],
    planner="RRTConnect",
    threshold=threshold_distance,
)
ompl.add_scene_obstacles(display=True)
ompl.check_obstacles()

# reset arm joint position
demo.move_arm_to_joint_angles(pose1)

# open the drawer
kitchen_id.open_drawer("elementA", drawer_id)
time.sleep(0.1)

# load bowl
bowl_id = pb_client.load_object("./URDF_models/utensil_bowl_blue/model.urdf", drop_position, [0.0, 0.0, 0.0], 1.0, "bowl")

# navigate to standing position
demo.navigate_base(Pose(nav_target_position, [0.0, 0.0, 4 * math.pi / 2.0]))

# grasp target object
ee_position, ee_orientation = demo.get_end_effector_info()
ompl.set_target(bowl_id)
start = demo.cartesian_to_joints_without_gripper(position=ee_position, orientation=p.getEulerFromQuaternion(ee_orientation))
goal = demo.cartesian_to_joints_without_gripper(position=drop_position, orientation=target_orientation)
print('start:{} goal:{}'.format(start, goal))
ompl.grasp_object(start=start, goal=goal, end_effector_link_index=demo.end_effector_index)

pb_client.wait(20)

pb_client.disconnect_pybullet()