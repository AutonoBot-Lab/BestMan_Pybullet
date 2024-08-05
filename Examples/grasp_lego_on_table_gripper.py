# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : grasp_lego_on_table_gripper.py
# @Time           : 2024-08-03 15:04:03
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : A example to grasp lego on table use gripper
"""


import os
import math
from RoboticsToolBox import Bestman_sim_panda, Pose
from Env import Client
from Visualization import Visualizer
from Motion_Planning.Navigation import *
from Utils import load_config


def main(filename):

    # Load config
    config_path = "../Config/grasp_lego_on_table_gripper.yaml"
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)
    visualizer.draw_axes()

    # Start record
    visualizer.start_record(filename)

    # Init robot
    bestman = Bestman_sim_panda(client, visualizer, cfg)

    # Load table and lego
    table_id = client.load_object(
        "table",
        "../Asset/URDF_models/furniture_table_rectangle_high/table.urdf",
        [1.0, 1.0, 0.0],
        [0.0, 0.0, 0.0],
        1.0,
        fixed_base=True,
    )

    lego_id = client.load_object(
        "lego",
        os.path.join(client.get_datapath(), "lego/lego.urdf"),
        [0.6, 0.6, 0.85],
        [0.0, 0.0, 0.0],
        1.0,
    )

    # grasp target object
    bestman.sim_active_gripper_fixed(0)
    visualizer.draw_pose("lego")
    object_goal_pose = Pose([1.2, 0.85, 0.88], [0, math.pi, 0])
    bestman.pick_place("lego", object_goal_pose)

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
