"""
@Description :   This script answers a question that could the robot detect the opened door in the navigation.
@Author      :   Yan Ding 
@Time        :   2023/08/31 03:01:50
"""

import sys

sys.path.append("/BestMan_Pybullet/refactor")

from Motion_Planning.manipulation.OMPL_Planner import OMPL_Planner
from refactor.Env.Client import PbClient
from refactor.Visualization.Visualizer import PbVisualizer

from RoboticsToolBox.Bestman import Bestman
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
config_path = "/BestMan_Pybullet/refactor/config/test_gripper.yaml"
cfg = load_config(config_path)
print(cfg)

# init engine and visualzer
pb_client = PbClient(cfg.Client)
pb_client.enable_vertical_view(cfg.Client.Camera_params)
pb_visualizer = PbVisualizer(pb_client)

# init robot
# logID = pb_client.start_record("example_navigation_with_opened_fridge") # start recording
demo = Bestman(pb_client, cfg.Robot)  # load robot
demo.get_joint_link_info("arm")  # get info about arm

# load kitchen
kitchen = Kitchen(pb_client)
print("--------------------")
print("Object information contained in the scene:\n{}".format(kitchen.object_ids))
print("--------------------")

# load bowl
bowl_position = [4.15, 4.3, 1.0]  # TODO: object goes flying
bowl_id = pb_client.load_object(
    "/BestMan_Pybullet/refactor/Asset/URDF_models/utensil_bowl_blue/model.urdf",
    bowl_position,
    [0.0, 0.0, 0.0],
    1.0,
    "bowl",
    fixed_base=False,
)

# load OMPL planner
planner = OMPL_Planner(demo, cfg.Planner)

# add obstacles
planner.add_scene_obstacles()
planner.get_obstacles_info()

pb_client.run(1000)

# _, _, min_z, _, _, max_z = pb_client.get_bounding_box(bowl_id, True)
# goal_position = bowl_position[:2]
# goal_position.append(max_z + demo.tcp_height * 2)     # consider tcp's height
# print("goal position:{}".format(goal_position))

# set target object for grasping
planner.set_target(bowl_id)
# target_orientation = [0.0, math.pi / 2.0, 0.0]  # vertical
# goal = demo.cartesian_to_joints(position=goal_position, orientation=target_orientation)
# print("-" * 20 + "\n" + "Goal configuration:{}".format(goal))

# reach target object
pb_visualizer.change_arm_color(demo.arm_id, light_color=True)
res, trajectory = planner.plan_and_excute(start=demo.get_arm_joints_angle())

pb_client.run(100)
# print('trajectory:{}'.format(trajectory))

# perform action
pb_visualizer.change_arm_color(demo.arm_id, light_color=False)
# demo.execute_trajectory(trajectory)

# # grasp object
# demo.active_gripper(bowl_id, 1)

# # disconnect pybullet
pb_client.wait(1000)
pb_client.disconnect_pybullet()
