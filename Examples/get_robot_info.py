#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
# @FileName      : get_robot_info
# @Time          : 2024-08-01 20:05:44
# @Author        : kui yang
# @Email         : yangkui1127@gmail.com
# @description   : A example to get robot joint info
"""

import os 
from Env import Client
from Utils import load_config
from Visualization import Visualizer
from RoboticsToolBox import Bestman_sim_ur5e_vacuum_long

def main(filename):
    
    # Load config
    config_path = '../Config/navigation_basic.yaml'
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)

    # Start recording
    visualizer.start_record(filename)
    
    # Load scene
    scene_path = '../Asset/Scene/Kitchen.json'
    client.create_scene(scene_path)

    # Init robot
    bestman = Bestman_sim_ur5e_vacuum_long(client, visualizer, cfg)
    
    bestman.print_joint_link_info('arm')

    # End record
    visualizer.end_record()

    # Disconnect pybullet
    client.wait(5)
    client.disconnect()
    
if __name__=='__main__':
    
    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    # get current file name
    filename = os.path.splitext(os.path.basename(__file__))[0]

    main(filename)
