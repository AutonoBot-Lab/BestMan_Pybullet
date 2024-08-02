#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
# @FileName      : load_kitchen
# @Time          : 2024-08-01 20:17:20
# @Author        : kui yang
# @Email         : yangkui1127@gmail.com
# @description   : A example to load kitchen
"""

import os
from Env import Client
from Utils import load_config


def main():
    
    # Load config 
    config_path = '../Config/load_kitchen.yaml'
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)
    
    # Load scene
    scene_path = '../Asset/Scene/Kitchen.json'
    client.create_scene(scene_path)

    # disconnect pybullet
    client.wait(10)
    client.disconnect()
    

if __name__=='__main__':
    
    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    main()
