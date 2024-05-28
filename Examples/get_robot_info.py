"""
@Description :   This script shows how to navigate to a goal position
@Author      :   Yan Ding 
@Time        :   2023/08/31 03:01:50
"""

import os 
import math
from RoboticsToolBox import Bestman, Pose
from Env.Client import Client
from Visualization import Visualizer
from Motion_Planning.Navigation import *
from Utils import load_config

def main():
    
    # Load config
    config_path = '../Config/navigation_basic.yaml'
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)

    # Load scene
    scene_path = '../Asset/Scene/Kitchen.json'
    client.create_scene(scene_path)

    # logID = pb_client.start_record("example_manipulation")    # start recording
    # Init robot
    bestman = Bestman(client, visualizer, cfg)
    
    bestman.print_joint_link_info('arm')

    client.wait(1000)
    client.disconnect()
    
if __name__=='__main__':
    
    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    main()
