"""
@Description :   This script shows how to grasp a bowl from a fridge
@Author      :   Yan Ding 
@Time        :   2023/09/01 07:47:46
"""

import os
from RoboticsToolBox import Bestman_sim_ur5e_vacuum_long
from Env.Client import Client
from Visualization.Visualizer import Visualizer
from Utils.load_config import load_config

def main(filename):
    
    # load config
    config_path = '../Config/load_ur5e.yaml'
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)

    # Start record
    visualizer.start_record(filename)
    
    # Init robot
    ur5e = Bestman_sim_ur5e_vacuum_long(client, visualizer, cfg)
    visualizer.change_robot_color(ur5e.get_base_id(), ur5e.get_arm_id(), False)
    
    # End record
    visualizer.end_record()
    
    # disconnect pybullet
    client.wait(5)
    client.disconnect()


if __name__=='__main__':
    
    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    # get current file name
    file_name = os.path.splitext(os.path.basename(__file__))[0]
    
    main(file_name)
