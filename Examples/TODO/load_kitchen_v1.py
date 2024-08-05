"""
@Description :   load a kitchen scenario
@Author      :   Yan Ding 
@Time        :   2023/08/30 20:43:44
"""

import math
import sys
from yacs.config import CfgNode as CN
import os

sys.path.append("/BestMan_Pybullet/refactor")

from RoboticsToolBox.Bestman import Bestman
from RoboticsToolBox.Pose import Pose
from refactor.Env.Client import PbClient
from refactor.Visualization.Visualizer import PbVisualizer
from utils_PbOMPL import PbOMPL

# load kitchen from three scenarios
index = 1
if index == 0:
    from Env.Kitchen_v0 import Kitchen
elif index == 1:
    from Env.Kitchen_v1 import Kitchen
else:
    assert False, "index should be 0 or 1"


# load config parameters
with open("/BestMan_Pybullet/refactor/config/default.yaml", "r") as file:
    cfg = CN.load_cfg(file)

# create client
pb_client = PbClient(cfg.Client)
pb_client.enable_vertical_view(2.2, [1.9, 7.35, 1.54], yaw=88.8, pitch=-31.5)
pb_visualizer = PbVisualizer(pb_client)

# logID = pb_client.start_record("example_manipulation") # start recording
init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose, pb_client, cfg.Controller)  # load robot
demo.get_joint_link_info("arm")  # get info about arm
init_joint = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(init_joint)  # reset arm joint position

# load kitchen
kitchen = Kitchen(pb_client, lisdf_id=0)

# disconnect pybullet
pb_client.wait(50)
pb_client.disconnect_pybullet()
