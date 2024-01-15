"""
@Description :   This script shows how to navigate to a goal position
@Author      :   Yan Ding 
@Time        :   2023/08/31 03:01:50
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
countertop_id = pb_client.load_object(
    "./Kitchen_models/models_yan/elementB/model.urdf",
    [0.0, 0.0, 0.7],
    [0.0, 0.0, math.pi / 2],
    1.0,
    "countertop",
    fixed_base=True,
)

pb_visualizer.set_elementB_visual_color(countertop_id)

bowl_id = pb_client.load_object(
    "./URDF_models/utensil_bowl_blue/model.urdf",
    [0.3, 0.8, 1.2],
    [0.0, 0.0, 0.0],
    1.0,
    "bowl",
)

pb_client.wait(2)
pb_client.run(1000)

# disconnect from server
pb_client.wait(5)
pb_client.disconnect_pybullet()
