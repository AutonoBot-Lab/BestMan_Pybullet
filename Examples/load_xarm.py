# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : grasp_bowl_on_table_vacuum_gripper.py
# @Time           : 2024-08-03 15:03:52
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : A example to grasp bowl on table use vacuum_gripper (simplified to just display robot model)
"""


import os
from Config import load_config
from Env import Client
from Robotics_API import Bestman_sim_xarm
from Visualization import Visualizer

def main(filename):
    
    # Load config
    config_path = "Config/load_xarm.yaml"
    cfg = load_config(config_path)
    print(cfg)
    
    # Init client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)
    visualizer.draw_axes()

    # Start record
    visualizer.start_record(filename)
    
    # Init robot
    xarm = Bestman_sim_xarm(client, visualizer, cfg)
    
    xarm.sim_interactive_set_arm(1000)
    
    client.wait(10)
    visualizer.capture_screen('xarm')
    
    # End record (Optional, can comment out if not needed)
    visualizer.end_record()

    # disconnect from server
    client.wait(20)
    client.disconnect()
    

if __name__ == "__main__":

    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # get current file name
    filename = os.path.splitext(os.path.basename(__file__))[0]

    main(filename)
