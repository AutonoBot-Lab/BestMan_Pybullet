"""
@Description :   This script shows how to grasp a bowl from a fridge
@Author      :   Yan Ding 
@Time        :   2023/09/01 07:47:46
"""

import math
import sys
import os
import pybullet as p

# customized package
# current_path = os.path.abspath(__file__)
# utils_path = os.path.dirname(os.path.dirname(current_path))
# if os.path.basename(utils_path) != "refactor":
#     raise ValueError('Not add the path of folder "utils", please check again!')
sys.path.append('/BestMan_Pybullet/refactor')

from Motion_Planning.Robot.Bestman import Bestman
from Motion_Planning.Robot.Pose import Pose
from Env.PbClient import PbClient
from Visualization.PbVisualizer import PbVisualizer
from utils_PbOMPL import PbOMPL

# load kitchen from three scenarios
index = 1
if index == 0:
    from Env.Kitchen_v0 import Kitchen
elif index == 1:
    from Env.Kitchen_v1 import Kitchen
else:
    assert False, "index should be 0 or 1"


# pb_client = PbClient(enable_GUI=True, enable_Debug=True)
pb_client = PbClient(enable_GUI=True)
# pb_client.enable_vertical_view(2.4, [1.75, 0, 1.46], yaw=90.8, pitch=10.5)
pb_client.enable_vertical_view(2.4, [1.6, 0, 1.66], yaw=90.0, pitch=-89.9)
pb_visualizer = PbVisualizer(pb_client)
# logID = pb_client.start_record("example_manipulation") # start recording
init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose, pb_client)  # load robot
demo.get_joint_link_info("arm")  # get info about arm
# init_joint = [0, -1.57, 2.0, -1.57, -1.57, 0]
init_joint = [0, -1.57, 1.5, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(init_joint)  # reset arm joint position

# load bowl
bowl_position = [0.85, 0.5, 1.45]
bowl_id = pb_client.load_object(
    "/BestMan_Pybullet/refactor/Asset/URDF_models/bowl/model.urdf",
    bowl_position,
    [0.0, 0.0, 0.0],
    1.0,
    "bowl",
    tag_obstacle_navigate=False,
)

demo.debug_set_joint_values()

# disconnect pybullet
pb_client.wait(1000)
pb_client.disconnect_pybullet()
