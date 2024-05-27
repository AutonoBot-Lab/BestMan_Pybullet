"""
@Description :   This script shows how to grasp a bowl from a fridge
@Author      :   Yan Ding 
@Time        :   2023/09/01 07:47:46
"""

import os
from Motion_Planning.Robot import Bestman
from Env import Client
from Visualization import Visualizer
from Utils import load_config


def main():
    
    # Load config 
    config_path = '../Config/load_kitchen.yaml'
    cfg = load_config(config_path)
    # print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)
    
    # Load scene
    scene_path = '../Asset/Scene/Kitchen.json'
    client.create_scene(scene_path)

    # # logID = pb_client.start_record("example_manipulation")    # start recording
    # # Init robot
    # bestman = Bestman(client, visualizer, cfg)
    # visualizer.change_robot_color(bestman.get_base_id(), bestman.get_arm_id(), False)

    # disconnect pybullet
    client.wait(50)
    client.disconnect()
    

if __name__=='__main__':
    
    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    main()
