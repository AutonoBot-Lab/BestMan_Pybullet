# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : blender_test.py
# @Time           : 2024-08-03 15:02:38
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : A example to record pybullet scene and render in blender
"""


import os

from Config import load_config
from Env import Client


def main(filename):

    # Load config
    config_path = "Config/blender_test.yaml"
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)

    # Load table, bowl, and chair
    table_id = client.load_object(
        "table",
        "Asset/URDF_models/furniture_table_rectangle_high/table.urdf",
        [1.0, 1.0, 0.0],
        [0.0, 0.0, 0.0],
        1.0,
        True,
    )

    chair_id = client.load_object(
        "chair",
        "Asset/URDF_models/furniture_chair/model.urdf",
        [-0.3, 0.8, 0.0],
        [0.0, 0.0, 0.0],
        1.5,
        True,
    )

    bowl_id = client.load_object(
        "bowl",
        "Asset/URDF_models/utensil_bowl_blue/model.urdf",
        [0.6, 0.6, 0.85],
        [0.0, 0.0, 0.0],
        1.0,
    )

    # Disconnect pybullet
    client.wait(5)
    client.disconnect()


if __name__ == "__main__":

    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # get current file name
    filename = os.path.splitext(os.path.basename(__file__))[0]

    main(filename)
