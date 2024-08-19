# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : navigation_basic.py
# @Time           : 2024-08-03 15:05:07
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : A example about robot navigation
"""


import math
import os

from Config import load_config
from Env.Client import Client
from Motion_Planning.Navigation import *
from RoboticsToolBox import Bestman_sim_ur5e_vacuum_long, Pose
from SLAM import simple_slam
from Visualization import Visualizer


def main(filename):

    # Load config
    config_path = "../Config/navigation_basic.yaml"
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)

    # Start record
    visualizer.start_record(filename)

    # Load scene
    scene_path = "../Asset/Scene/Kitchen.json"
    client.create_scene(scene_path)

    # Init robot
    bestman = Bestman_sim_ur5e_vacuum_long(client, visualizer, cfg)

    # Load table, bowl, and chair
    table_id = client.load_object(
        "table",
        "../Asset/URDF_models/furniture_table_rectangle_high/table.urdf",
        [1.0, 1.0, 0.0],
        [0.0, 0.0, 0.0],
        1.0,
        True,
    )

    bowl_id = client.load_object(
        "bowl",
        "../Asset/URDF_models/utensil_bowl_blue/model.urdf",
        [0.6, 0.6, 0.85],
        [0.0, 0.0, 0.0],
        1.0,
    )

    chair_id = client.load_object(
        "chair",
        "../Asset/URDF_models/furniture_chair/model.urdf",
        [-0.3, 0.8, 0.0],
        [0.0, 0.0, 0.0],
        1.5,
        True,
    )

    # Get bounding box of objects
    aabb_table = client.get_bounding_box(table_id)
    visualizer.draw_aabb(table_id)
    print("-" * 20 + "\n" + "aabb_table:{}".format(aabb_table))

    # Simple SLAM
    # nav_obstacles_bounds = simple_slam(client, bestman, True)
    nav_obstacles_bounds = simple_slam(client, bestman, False)

    # navigate algorithm
    goal_base_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2.0])
    nav_planner = AStarPlanner(
        robot_size=bestman.sim_get_robot_size(),
        obstacles_bounds=nav_obstacles_bounds,
        resolution=0.05,
        enable_plot=False,
    )

    # nav_planner = RRTPlanner(
    #     robot_size = bestman.get_robot_max_size(),
    #     obstacles_bounds = client.get_Nav_obstacles_bounds(),
    #     enable_plot = True
    # )

    # nav_planner = PRMPlanner(
    #     robot_size = bestman.get_robot_max_size(),
    #     obstacles_bounds = client.get_Nav_obstacles_bounds(),
    #     enable_plot = True
    # )

    path = nav_planner.plan(
        start_pose=bestman.sim_get_current_base_pose(), goal_pose=goal_base_pose
    )

    # navigate segbot
    bestman.sim_navigate_base(goal_base_pose, path)

    # End record
    visualizer.end_record()

    client.wait(5)
    client.disconnect()


if __name__ == "__main__":

    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # get current file name
    file_name = os.path.splitext(os.path.basename(__file__))[0]

    main(file_name)
