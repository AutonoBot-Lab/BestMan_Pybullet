# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : anygrasp.py
# @Time           : 2024-08-03 15:02:27
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : A example for anygrasp module use in pybullet sim
"""


import os

from Env import Client
from Perception.Grasp_Pose_Estimation import Anygrasp
from Utils import load_config


def main(filename):

    # Load config
    config_path = "../Config/blender_test.yaml"
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)

    # Init anygrasp module
    anygrasp = Anygrasp(cfgs)


if __name__ == "__main__":

    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # get current file name
    filename = os.path.splitext(os.path.basename(__file__))[0]

    main(filename)
