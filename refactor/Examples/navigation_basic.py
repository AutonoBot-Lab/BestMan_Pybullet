"""
@Description :   This script shows how to navigate to a goal position
@Author      :   Yan Ding 
@Time        :   2023/08/31 03:01:50
"""

import math
import sys
import os

"""
Get the utils module path
"""
# customized package
current_path = os.path.abspath(__file__)
utils_path = os.path.dirname(os.path.dirname(current_path)) + "/utils"
if os.path.basename(utils_path) != "utils":
    raise ValueError('Not add the path of folder "utils", please check again!')
sys.path.append(utils_path)
from utils_Bestman import Bestman, Pose
from utils_PbClient import PbClient
from utils_PbVisualizer import PbVisualizer

# load kitchen from three scenarios
index = 0
if index == 0:
    from utils_Kitchen_v0 import Kitchen
elif index == 1:
    from utils_Kitchen_v1 import Kitchen
else:
    assert False, "index should be 0 or 1"


pb_client = PbClient(enable_GUI=True)
pb_client.enable_vertical_view(4.0, [1.0, 1.0, 0])
pb_visualizer = PbVisualizer(pb_client)
# logID = pb_client.start_record("example_manipulation") # start recording
init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose, pb_client)  # load robot
demo.get_joint_link_info("arm")  # get info about arm
init_joint = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(init_joint)  # reset arm joint position

# load table, bowl, and chair
table_id = pb_client.load_object(
    "./URDF_models/furniture_table_rectangle_high/table.urdf",
    [1.0, 1.0, 0.0],
    [0.0, 0.0, 0.0],
    1.0,
    "table",
    fixed_base=True,
)
bowl_id = pb_client.load_object(
    "./URDF_models/utensil_bowl_blue/model.urdf",
    [0.6, 0.6, 0.85],
    [0.0, 0.0, 0.0],
    1.0,
    "bowl",
    tag_obstacle_navigate=False,
)
chair_id = pb_client.load_object(
    "./URDF_models/furniture_chair/model.urdf",
    [-0.3, 0.8, 0.1],
    [math.pi / 2.0 * 3, 0.0, math.pi / 2.0],
    1.5,
    "chair",
    tag_obstacle_navigate=False,
)

# add obstacles in the navigation
print("obstacles for navigation: {}".format(pb_client.obstacle_navigation_ids))

# get bounding box of objects
aabb_table = pb_client.get_bounding_box(table_id)
pb_visualizer.draw_aabb(table_id)
print("-" * 20 + "\n" + "aabb_table:{}".format(aabb_table))

# plot line connecting init and goal positions
target_position = [5.0, 1.0, 0]
pb_visualizer.draw_line([1, 0, 0], target_position)

# navigate segbot
demo.navigate_base(Pose(target_position, [0.0, 0.0, math.pi / 2.0]))

# # end recording
# pb_client.end_record(logID)

pb_client.wait(5)
pb_client.disconnect_pybullet()
