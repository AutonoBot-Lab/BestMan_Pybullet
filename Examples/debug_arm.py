"""
@Description :   This script shows how to grasp a bowl from a fridge
@Author      :   Yan Ding 
@Time        :   2023/09/01 07:47:46
"""

import sys

sys.path.append('/BestMan_Pybullet/refactor')

from Motion_Planning.Robot.Bestman import Bestman
from refactor.Env.Client import PbClient
from refactor.Visualization.Visualizer import PbVisualizer
from Utils.load_config import load_config

# load kitchen from three scenarios
index = 1
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

# create client
pb_client = PbClient(cfg.Client)

# pb_client.enable_vertical_view(2.4, [1.75, 0, 1.46], yaw=90.8, pitch=10.5)
pb_client.enable_vertical_view(cfg.Client.Camera_params)
pb_visualizer = PbVisualizer(pb_client)

# logID = pb_client.start_record("example_manipulation")  # start recording
bestman = Bestman(pb_client, cfg.Robot)  # load robot
bestman.get_joint_link_info("arm")     # get info about arm

# load bowl
bowl_position = [0.85, 0.5, 1.45]
bowl_id = pb_client.load_object(
    "/BestMan_Pybullet/refactor/Asset/URDF_models/bowl/model.urdf",
    bowl_position,
    [0.0, 0.0, 0.0],
    1.0,
    "bowl"
)

bestman.debug_set_joint_values()

# disconnect pybullet
pb_client.wait(1000)
pb_client.disconnect_pybullet()
