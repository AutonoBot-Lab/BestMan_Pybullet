# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : draw_fridge_door_link.py
# @Time           : 2024-08-03 15:03:09
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : A example to draw fridge door link
"""


import math
import os

from Config import load_config
from Env import Client
from Robotics_API import Bestman_sim_ur5e_vacuum_long
from Visualization import Visualizer


def main(filename):

    # Load config
    config_path = "Config/draw_AABB_fridge_door_link.yaml"
    cfg = load_config(config_path)
    print(cfg)

    # Initial client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)

    # Start recording
    visualizer.start_record(filename)

    # Load scene
    scene_path = "Asset/Scene/Scene/Kitchen.json"
    client.create_scene(scene_path)

    # Init robot
    bestman = Bestman_sim_ur5e_vacuum_long(client, visualizer, cfg)

    # Init visualizer
    visualizer.change_robot_color(
        bestman.sim_get_base_id(), bestman.sim_get_arm_id(), False
    )

    # Open fridge joint between handle and door
    client.change_object_joint_angle("fridge", 1, math.pi / 2)

    # Draw fridge aabb link
    visualizer.draw_aabb_link("fridge", 1)  # door
    visualizer.draw_aabb_link("fridge", 2)  # handle

    # End record
    visualizer.end_record()

    # Disconnect pybullet
    client.wait(10)
    client.disconnect()


if __name__ == "__main__":

    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # get current file name
    filename = os.path.splitext(os.path.basename(__file__))[0]

    main(filename)
