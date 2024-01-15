"""
@Description :   Load a specific kitchen environment
@Author      :   Yan Ding 
@Time        :   2024/01/07 16:41:55
"""

import math
import sys
import os
import pybullet as p

"""
Get the utils module path
"""
# customized package
current_path = os.path.abspath(__file__)
utils_path = os.path.dirname(os.path.dirname(current_path)) + '/utils'
if os.path.basename(utils_path) != 'utils':
    raise ValueError('Not add the path of folder "utils", please check again!')
sys.path.append(utils_path)
from utils_Bestman import Bestman, Pose
from utils_PbClient import PbClient
from utils_PbVisualizer import PbVisualizer
from utils_PbOMPL import PbOMPL

# load kitchen from three scenarios
index = 2
if index == 0:
    from utils_Kitchen_v0 import Kitchen
elif index == 1:
    from utils_Kitchen_v1 import Kitchen
elif index == 2:
    from utils_Kitchen_v2 import Kitchen
else:
    assert False, "index should be 0, 1 or 2"

# pb_client = PbClient(enable_GUI=True, enable_Debug=True)
pb_client = PbClient(enable_GUI=True)
# pb_client.enable_vertical_view(1.0, [2.61, 5.33, 1.29], -90, -29.9) # view 1
# pb_client.enable_vertical_view(2.0, [2.80, 4.69, 1.20], -89.9, -89.9) # view 2
pb_client.enable_vertical_view(2.0, [2.80, 5.5, 1.20], -89.9, -89.9) # view 3
# pb_client.enable_vertical_view(2.6, [1.80, 2.25, 1.20], -89.9, -89.9) # view 4
pb_visualizer = PbVisualizer(pb_client)
# logID = pb_client.start_record("example_manipulation") # start recording
init_pose = Pose([2, 8, 0], [0.0, 0.0, math.pi / 2])
# init_pose = Pose([2.5, 1.5, 0], [0.0, 0.0, math.pi / 2]) # example 4
# init_pose = Pose([0.5, 1.7, 0], [0.0, 0.0, 0.0]) # example 5
demo = Bestman(init_pose, pb_client)  # load robot
demo.get_joint_link_info("arm")  # get info about arm
init_joint = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(init_joint)  # reset arm joint position

# load kitchen
kitchen = Kitchen(pb_client)
print("object ids in loaded kitchen:\n{}".format(kitchen.object_ids))

# load OMPL planner
threshold_distance = 0.05
ompl = PbOMPL(
    pb_client=pb_client,
    arm_id=demo.arm_id,
    joint_idx=demo.arm_joint_indexs,
    tcp_link=demo.tcp_link,
    obstacles=[],
    planner="RRTConnect",
    threshold=threshold_distance,
)

# add obstacles
ompl.add_scene_obstacles(display=True)
ompl.check_obstacles()

# open fridge
kitchen.open_it("elementE", 1, open_angle=math.pi/4)

# load bowl
MilkBottle_position = [3.95, 5.42, 1.05]  # TODO: object goes flying
MilkBottle_id = pb_client.load_object("./Kitchen_models/models/MilkBottle/4043/mobility.urdf", MilkBottle_position, [0.0, 0.0, 0.0], 0.1, "MilkBottle")
pb_client.run(100)
_, _, min_z, _, _, max_z = pb_client.get_bounding_box(MilkBottle_id)
MilkBottle_position[2] = max_z + demo.tcp_height # consider tcp's height

# navigate to standing position

# example 1: far from fridge
# standing_position = [4, 7, 0]
# standing_orientation = [0.0, 0.0, -math.pi/2]
# demo.navigate_base(Pose(standing_position, standing_orientation))

# example 2: behind fridge door
# standing_position = [3.0, 4.5, 0]
# standing_orientation = [0.0, 0.0, 0.0]
# demo.navigate_base(Pose(standing_position, standing_orientation))

# example 3: correct
# standing_position = [3.0, 5.7, 0]
# standing_orientation = [0.0, 0.0, 0.0]
# demo.navigate_base(Pose(standing_position, standing_orientation))

# example 3: correct
standing_position = [2.8, 5.6, 0]
standing_orientation = [0.0, 0.0, 0.0]
demo.navigate_base(Pose(standing_position, standing_orientation))

# example 4: far from table
# standing_position = [2.15, 3.2, 0]
# standing_orientation = [0.0, 0.0, math.pi/4*5]
# demo.navigate_base(Pose(standing_position, standing_orientation))

# set target object for grasping
# ompl.set_target(MilkBottle_id)
# target_orientation = [0.0, math.pi / 2.0, 0.0] # vertical
# # target_orientation = [0.0, math.pi, 0.0] # horizontal
# goal = demo.cartesian_to_joints(position=MilkBottle_position, orientation=target_orientation)
# print("-" * 20 + "\n" + "Goal configuration:{}".format(goal))

# reach target object
# result = ompl.reach_object(start=demo.get_arm_joint_angle(), goal=goal, end_effector_link_index=demo.end_effector_index)

pb_visualizer.capture_screen(enable_Debug=True)

# disconnect pybullet
pb_client.wait(10)
pb_client.disconnect_pybullet()