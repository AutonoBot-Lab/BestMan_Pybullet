"""
@Description :   load a kitchen scenario
@Author      :   Yan Ding 
@Time        :   2023/08/30 20:43:44
"""

import math
from Motion_Planning.Robot import Bestman, Pose
from Env import Client
from Visualization import Visualizer
from Utils import load_config


# load config
config_path = '/GithubCode/BestMan_Pybullet/refactor/config/draw_AABB_fridge_door_link.yaml'
cfg = load_config(config_path)
print(cfg)

# initial client
client = Client(cfg.Client)

# load scene
scene_path = '/GithubCode/BestMan_Pybullet/refactor/Asset/Scene/Kitchen.json'
client.create_scene(scene_path)

# initial visualizer
visualizer = Visualizer(client, cfg.Visualizer)

# logID = pb_client.start_record("example_manipulation")    # start recording
# Init robot
bestman = Bestman(client, cfg.Robot, cfg.Controller)

# open fridge
client.change_object_joint_angle('elementE', 1, math.pi / 2)

# draw fridge aabb link
visualizer.draw_aabb_link('elementE', 1)

# disconnect pybullet
client.wait(1000)
client.disconnect_pybullet()
