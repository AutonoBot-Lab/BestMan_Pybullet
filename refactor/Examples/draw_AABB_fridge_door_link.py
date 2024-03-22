"""
@Description :   load a kitchen scenario
@Author      :   Yan Ding 
@Time        :   2023/08/30 20:43:44
"""

import math
import sys
import os


sys.path.append('/BestMan_Pybullet/refactor')

from Motion_Planning.Robot.Bestman import Bestman
from Motion_Planning.Robot.Pose import Pose
from refactor.Env.Client import PbClient
from refactor.Visualization.Visualizer import PbVisualizer
from Utils.load_config import load_config

# load kitchen from three scenarios
index = 0
if index == 0:
    from Env.Kitchen_v0 import Kitchen
elif index == 1:
    from Env.Kitchen_v1 import Kitchen
else:
    assert False, "index should be 0 or 1"


# load config
config_path = '/BestMan_Pybullet/refactor/config/test_gripper.yaml'
cfg = load_config(config_path)
print(cfg)

pb_client = PbClient(cfg.Client)
pb_client.enable_vertical_view(cfg.Client.Camera_params)
pb_visualizer = PbVisualizer(pb_client)

# logID = pb_client.start_record("example_manipulation")    # start recording
demo = Bestman(pb_client, cfg.Robot)    # load robot
demo.get_joint_link_info("arm")                 # get info about arm

# load kitchen
kitchen = Kitchen(pb_client)
print("object ids in loaded kitchen:\n{}".format(kitchen.object_ids))

# # get occupancy grid
# occupancy_grid = pb_client.get_occupancy_network(demo.base_id)
# print('occupancy_grid:{}'.format(occupancy_grid))

# after open fridge
kitchen.open_it("elementE", 1)

pb_visualizer.draw_aabb_link(kitchen.elementE_id, 1)

# # get occupancy grid
# occupancy_grid = pb_client.get_occupancy_network(demo.base_id, enable_plot=True)
# print('occupancy_grid:{}'.format(occupancy_grid))

# disconnect pybullet
pb_client.wait(100)
pb_client.disconnect_pybullet()
