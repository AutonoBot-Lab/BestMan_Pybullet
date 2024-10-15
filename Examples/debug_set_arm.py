# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : debug_set_arm.py
# @Time           : 2024-08-03 15:02:59
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : A Example to set all arm joint values by user
"""


import os

from Config import load_config
from Env import Client
from Robotics_API import Bestman_sim_panda
from Visualization import Visualizer


def main(filename):

    # Load config
    config_path = "Config/debug_set_arm.yaml"
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)

    # Start recording
    visualizer.start_record(filename)

    # Init robot
    bestman = Bestman_sim_panda(client, visualizer, cfg)
    visualizer.change_robot_color(
        bestman.sim_get_base_id(), bestman.sim_get_arm_id(), False
    )

    # Debug set arm joints
    bestman.sim_debug_set_arm_to_joint_values()

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
