"""
@Description :   This script shows how to grasp a bowl from a drawer
@Author      :   Yan Ding 
@Time        :   2023/09/01 07:47:46
"""

import math
import pybullet as p
from Motion_Planning.Robot import Bestman, Pose
from Env import Client
from Visualization import Visualizer
from Motion_Planning.Manipulation import OMPL_Planner
from Motion_Planning.Navigation import AStarPlanner
from Utils import load_config


# from Motion_Planning.Manipulation.Old import PbOMPL


# Load config
config_path = '../Config/draw_AABB_fridge_door_link.yaml'
cfg = load_config(config_path)
print(cfg)

# Init client and visualizer
client = Client(cfg.Client)
visualizer = Visualizer(client, cfg.Visualizer)

# Load scene
scene_path = '../Asset/Scene/Kitchen.json'
client.create_scene(scene_path)

# logID = pb_client.start_record("example_manipulation")    # start recording
# Init robot
bestman = Bestman(client, cfg)
visualizer.change_robot_color(bestman.get_base_id(), bestman.get_arm_id(), False)

# open the drawer
client.change_object_joint_angle("elementA", 36, 0.4)

# visualizer.draw_aabb(getattr(client, "elementA"))

# # Init OMPL planner
# ompl_planner = OMPL_Planner(
#     bestman, 
#     cfg.Planner
# )

# # add obstacles
# ompl_planner.add_scene_obstacles()
# ompl_planner.get_obstacles_info()

# ompl_planner = PbOMPL(
#     pb_client=client,
#     arm_id=bestman.arm_id,
#     joint_idx=bestman.arm_joint_indexs,
#     tcp_link=bestman.tcp_link,
#     obstacles=[],
#     # planner="BITstar",
#     planner="RRTConnect",
#     threshold=cfg.Planner.threshold,
# )

# # add obstacles
# ompl_planner.add_scene_obstacles(display=True)
# ompl_planner.check_obstacles()

# load bowl (target object must be added after ompl creation)
bowl_id = client.load_object(
    "../Asset/URDF_models/utensil_bowl_blue/model.urdf",
    [3.6, 2.4, 0.6],
    [0.0, 0.0, 0.0],
    1.0,
    "bowl",
    False,
)

# navigate to standing position
standing_pose = Pose([2.85, 2.4, 0], [0.0, 0.0, 0.0])
nav_planner = AStarPlanner(
    robot_size = bestman.get_robot_size(), 
    obstacles_bounds = client.get_Nav_obstacles_bounds(), 
    resolution = 0.05, 
    enable_plot = True
)

path = nav_planner.plan(bestman.get_current_pose(), standing_pose)

bestman.navigate_base(standing_pose, path)

# # ompl_planner.remove_obstacles(bowl_id)
# ompl_planner.set_target(bowl_id)
# ompl_planner.plan_and_excute()

# # reach target object
# result = ompl_planner.reach_object(
#     start=bestman.get_arm_joints_angle(),
#     goal=goal,
#     end_effector_link_index=bestman.end_effector_index,
# )

# disconnect pybullet
client.wait(1000)
client.disconnect()
