# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : load_bowl_on_countertop.py
# @Time           : 2024-08-03 15:04:15
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : A example to load bowl on countertop
"""


import math
import os

from Config import load_config
from Env import Client
from Motion_Planning.Navigation import *
from Robotics_API import Bestman_sim_ur5e_vacuum_long
from Visualization import Visualizer


def main(filename):

    # Load config
    config_path = "Config/load_bowl_on_countertop.yaml"
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)

    # Start record
    visualizer.start_record(filename)

    # Init robot
    bestman = Bestman_sim_ur5e_vacuum_long(client, visualizer, cfg)
    visualizer.change_robot_color(
        bestman.sim_get_base_id(), bestman.sim_get_arm_id(), False
    )

    # load table, bowl, and chair
    countertop_id = client.load_object(
        "countertop",
        "Asset/Scene/Kitchen_models/models_yan/elementB/model.urdf",
        [0.0, 0.0, 0.7],
        [0.0, 0.0, math.pi / 2],
        1.0,
        fixed_base=True,
    )

    # visualizer.set_object_color(countertop_id, "light_white")

    bowl_id = client.load_object(
        "bowl",
        "Asset/Scene/URDF_models/utensil_bowl_blue/model.urdf",
        [0.0, 0.5, 1.05],
        [0.0, 0.0, 0.0],
        1.0,
    )

    client.run(240)

    # End record
    visualizer.end_record()

    # disconnect from server
    client.wait(5)
    client.disconnect()


if __name__ == "__main__":

    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # get current file name
    filename = os.path.splitext(os.path.basename(__file__))[0]

    main(filename)
