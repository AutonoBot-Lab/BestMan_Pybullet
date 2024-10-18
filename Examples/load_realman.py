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
    visualizer.draw_axes()
    
    # Start record
    visualizer.start_record(filename)

    # Init robot
    realman = Bestman_sim_realman(client, visualizer, cfg)

    # Rotate base
    realman.sim_rotate_base(180, "counter-clockwise")

    realman.sim_move_base_forward(2)

    realman.sim_move_base_backward(2)

    realman.sim_move_base_left(2)

    realman.sim_move_base_right(2)

    # End record
    visualizer.end_record()

    # disconnect
    client.wait(10)
    client.disconnect()


if __name__ == "__main__":

    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # get current file name
    file_name = os.path.splitext(os.path.basename(__file__))[0]

    main(file_name)
