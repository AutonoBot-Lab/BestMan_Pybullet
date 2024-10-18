# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : get_robot_info.py
# @Time           : 2024-08-03 15:03:21
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : A example to get robot joint info
"""


import os

from Config import load_config
from Env import Client
from Robotics_API import Bestman_sim_ur5e_vacuum_long
from Visualization import Visualizer


def main(filename):

    # Load config
    config_path = "Config/navigation_basic.yaml"
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)

    # Start recording
    visualizer.start_record(filename)

    # Load scene
    scene_path = "Asset/Scene/Scene/Kitchen.json"
    client.create_scene(scene_path)

    # Init robot
    bestman = Bestman_sim_ur5e_vacuum_long(client, visualizer, cfg)

    # Get info about arm
    jointInfo = bestman.sim_get_arm_all_jointInfo()
    for info in jointInfo:
        print(info)

    # End record
    visualizer.end_record()

    # Disconnect pybullet
    client.wait(5)
    client.disconnect()


if __name__ == "__main__":

    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # get current file name
    filename = os.path.splitext(os.path.basename(__file__))[0]

    main(filename)
