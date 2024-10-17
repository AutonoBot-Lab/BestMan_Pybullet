# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : load_realman.py
# @Time           : 2024-08-03 15:04:49
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : A example to load realman robot
"""


import math
import os

from Config import load_config
from Env.Client import Client
from Motion_Planning.Navigation import *
from Robotics_API import Bestman_sim_realman, Pose
from SLAM import simple_slam
from Visualization.Visualizer import Visualizer


def main(filename):

    # load config
    config_path = "Config/load_realman.yaml"
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)
    # visualizer.draw_axes()
    
    # Start record
    visualizer.start_record(filename)

    # Init robot
    realman = Bestman_sim_realman(client, visualizer, cfg)

    # # Rotate base
    # realman.sim_rotate_base(180, "counter-clockwise")

    # realman.sim_move_base_forward(2)

    # realman.sim_move_base_backward(2)

    # realman.sim_move_base_left(2)

    # realman.sim_move_base_right(2)

    # visualizer.draw_object_pose(realman.sim_get_arm_id())

    # nav_obstacles_bounds = simple_slam(client, realman, False)

    # # navigate algorithm
    # goal_base_pose = Pose([5, 0, 0], [0.0, 0.0, math.pi / 2.0])
    # nav_planner = AStarPlanner(
    #     robot_size=realman.sim_get_robot_size(),
    #     obstacles_bounds=nav_obstacles_bounds,
    #     resolution=0.05,
    #     enable_plot=False,
    # )

    # path = nav_planner.plan(
    #     start_pose=realman.sim_get_current_base_pose(), goal_pose=goal_base_pose
    # )

    # # navigate segbot
    # realman.sim_navigate_base(goal_base_pose, path)

    # client.wait(3)
    # visualizer.capture_screen("test")

    # End record
    visualizer.end_record()

    # disconnect
    # client.keep_run(realman)
    # realman.sim_interactive_set_arm()

    client.wait(5)
    client.disconnect()


if __name__ == "__main__":

    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # get current file name
    file_name = os.path.splitext(os.path.basename(__file__))[0]

    main(file_name)
