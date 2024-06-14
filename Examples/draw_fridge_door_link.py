"""
@Description :   load a kitchen scenario
@Author      :   Yan Ding 
@Time        :   2023/08/30 20:43:44
"""

import os
import math
from RoboticsToolBox import Bestman_sim
from Env import Client
from Visualization import Visualizer
from Utils import load_config

def main():
    
    # load config
    config_path = '../Config/draw_AABB_fridge_door_link.yaml'
    cfg = load_config(config_path)
    print(cfg)

    # initial client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)

    # load scene
    scene_path = '../Asset/Scene/Kitchen.json'
    client.create_scene(scene_path)
    
    # Init robot
    bestman = Bestman_sim(client, visualizer, cfg)
    
    # Init visualizer
    visualizer.change_robot_color(bestman.get_base_id(), bestman.get_arm_id(), False)

    # open fridge
    client.change_object_joint_angle('elementE', 1, math.pi / 2)

    # draw fridge aabb link
    visualizer.draw_aabb_link('elementE', 1)

    # disconnect pybullet
    client.wait(1000)
    client.disconnect()
    
if __name__=='__main__':
    
    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    main()
