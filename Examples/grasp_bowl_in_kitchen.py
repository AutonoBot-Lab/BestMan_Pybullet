# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : grasp_bowl_in_kitchen.py
# @Time           : 2024-08-03 15:03:43
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : A example to grasp bowl in kitchen
"""


import math
import os

from Env import Client
from Motion_Planning.Manipulation import OMPL_Planner
from Motion_Planning.Navigation import *
from RoboticsToolBox import Bestman_sim_ur5e_vacuum_long, Pose
from SLAM import simple_slam
from Utils import load_config
from Visualization import Visualizer


def main(filename):

    # Load config
    config_path = "../Config/grasp_bowl_in_kitchen.yaml"
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)
    visualizer.draw_axes()

    # Start record
    visualizer.start_record(filename)

    # Load scene
    scene_path = "../Asset/Scene/Kitchen.json"
    client.create_scene(scene_path)

    # Init robot
    bestman = Bestman_sim_ur5e_vacuum_long(client, visualizer, cfg)
    visualizer.change_robot_color(bestman.sim_get_base_id(), bestman.sim_get_arm_id(), False)

    # Open fridge
    client.change_object_joint_angle("microwave", 1, math.pi / 2.0)

    # Simple SLAM
    nav_obstacles_bounds = simple_slam(client, bestman, True)

    # Navigation
    standing_pose = Pose([2.9, 2.55, 0], [0.0, 0.0, 0.0])
    nav_planner = AStarPlanner(
        robot_size=bestman.sim_get_robot_size(),
        obstacles_bounds=nav_obstacles_bounds,
        resolution=0.05,
        enable_plot=False,
    )
    path = nav_planner.plan(bestman.sim_get_current_base_pose(), standing_pose)
    bestman.sim_navigate_base(standing_pose, path)
    
    # load bowl
    bowl_id = client.load_object(
        "bowl",
        "../Asset/URDF_models/utensil_bowl_blue/model.urdf",
        [3.8, 2.4, 0.95],
        [0.0, 0.0, 0.0],
        1.0,
    )

    visualizer.draw_object_pose("bowl")

    # get rgb image
    # bestman.update_camera()
    # bestman.get_camera_rgb_image(False, False)

    # Init ompl planner
    ompl_planner = OMPL_Planner(bestman, cfg.Planner)
    ompl_planner.get_obstacles_info()

    # Planning
    goal = ompl_planner.set_target("bowl")
    start = bestman.sim_get_current_joint_values()
    path = ompl_planner.plan(start, goal)

    # Robot execute
    bestman.sim_execute_trajectory(path)

    # grasp target object
    bestman.sim_create_fixed_constraint(bowl_id)

    # # End record
    visualizer.end_record()

    # disconnect pybullet
    client.wait(10)
    client.disconnect()


if __name__ == "__main__":

    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # get current file name
    filename = os.path.splitext(os.path.basename(__file__))[0]

    main(filename)
