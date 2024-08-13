# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : load_ur5e.py
# @Time           : 2024-08-03 15:04:49
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : A example to load ur5e robot
"""


import os

from Env.Client import Client
from RoboticsToolBox import Bestman_sim_ur5e_vacuum_long
from Utils.load_config import load_config
from Visualization.Visualizer import Visualizer


def main(filename):

    # load config
    config_path = "../Config/load_ur5e.yaml"
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)

    # Start record
    visualizer.start_record(filename)

    # Init robot
    ur5e = Bestman_sim_ur5e_vacuum_long(client, visualizer, cfg)
    visualizer.change_robot_color(ur5e.sim_get_base_id(), ur5e.sim_get_arm_id(), False)

    # End record
    visualizer.end_record()

    # disconnect pybullet
    client.wait(5)
    client.disconnect()


if __name__ == "__main__":

    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # get current file name
    file_name = os.path.splitext(os.path.basename(__file__))[0]

    main(file_name)
