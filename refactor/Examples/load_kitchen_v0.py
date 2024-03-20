"""
@Description :   This script shows how to grasp a bowl from a fridge
@Author      :   Yan Ding 
@Time        :   2023/09/01 07:47:46
"""

import math
import sys
import os
import pybullet as p

sys.path.append('/BestMan_Pybullet/refactor')

from Motion_Planning.Robot.Bestman import Bestman
from Motion_Planning.Robot.Pose import Pose
from Env.PbClient import PbClient
from Visualization.PbVisualizer import PbVisualizer
# from utils_PbOMPL import PbOMPL
from Utils.load_config import load_config

# load kitchen from three scenarios
index = 0
if index == 0:
    from Env.Kitchen_v0 import Kitchen
elif index == 1:
    from Env.Kitchen_v1 import Kitchen
else:
    assert False, "index should be 0 or 1"

config_path = '/BestMan_Pybullet/refactor/config/default.yaml'
cfg = load_config(config_path)
# print(cfg)

pb_client = PbClient(cfg.Client)
pb_client.enable_vertical_view(1.0, [1.7, 3.68, 1.95], -86.4, -52.3)
pb_visualizer = PbVisualizer(pb_client)
# logID = pb_client.start_record("example_manipulation") # start recording
demo = Bestman(pb_client, cfg.Robot)  # load robot
demo.get_joint_link_info("arm")  # get info about arm
init_joint = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(init_joint)  # reset arm joint position

# load kitchen
kitchen = Kitchen(pb_client)
print("object ids in loaded kitchen:\n{}".format(kitchen.object_ids))

# # load OMPL planner
# threshold_distance = 0.1
# ompl = PbOMPL(
#     pb_client=pb_client,
#     arm_id=demo.arm_id,
#     joint_idx=demo.arm_joint_indexs,
#     tcp_link=demo.tcp_link,
#     obstacles=[],
#     planner="RRTConnect",
#     threshold=threshold_distance,
# )

# # add obstacles
# ompl.add_scene_obstacles(display=True)
# ompl.check_obstacles()

# # open fridge
# kitchen.open_it("elementE", 1)

# disconnect pybullet
pb_client.wait(10)
pb_client.disconnect_pybullet()
