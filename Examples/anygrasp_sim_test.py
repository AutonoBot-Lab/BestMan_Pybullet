# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : load_kitchen.py
# @Time           : 2024-08-03 15:04:25
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : A example to load kitchen
"""

import os

import numpy as np
import open3d as o3d

from Env import Client
from RoboticsToolBox import Bestman_sim_panda
from Utils import load_config
from Visualization import Visualizer


def main():

    # Load config
    config_path = "../Config/anygrasp_sim_test.yaml"
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)

    # Load scene
    scene_path = "../Asset/Scene/Kitchen_anygrasp.json"
    client.create_scene(scene_path)

    # Init robot
    bestman = Bestman_sim_panda(client, visualizer, cfg)
    bestman.get_camera_rgb_image(False, True, "rgb_test")
    bestman.get_camera_depth_image(False, True, "depth_test")
    bestman.visualize_3d_points()

    # disconnect pybullet
    client.wait(30)
    client.disconnect()


if __name__ == "__main__":

    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    main()
