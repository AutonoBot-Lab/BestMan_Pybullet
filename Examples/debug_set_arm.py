"""
@Description :   This script shows how to grasp a bowl from a fridge
@Author      :   Yan Ding 
@Time        :   2023/09/01 07:47:46
"""

import os
from RoboticsToolBox.Bestman import Bestman
from Env.Client import Client
from Visualization.Visualizer import Visualizer
from Utils.load_config import load_config

def main():
    
    # load config
    config_path = '../Config/debug_arm.yaml'
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)

    # logID = pb_client.start_record("example_manipulation")    # start recording
    # Init robot
    bestman = Bestman(client, visualizer, cfg)
    visualizer.change_robot_color(bestman.get_base_id(), bestman.get_arm_id(), False)
    
    bestman.print_joint_link_info("arm")     # get info about arm
    
    # debug set arm joints
    bestman.debug_set_joint_values()

    # disconnect pybullet
    client.wait(1000)
    client.disconnect_pybullet()


if __name__=='__main__':
    
    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    main()
