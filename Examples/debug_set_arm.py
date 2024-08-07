"""
@Description :   This script shows how to grasp a bowl from a fridge
@Author      :   Yan Ding 
@Time        :   2023/09/01 07:47:46
"""

import os
from Env import Client
from Utils import load_config
from Visualization import Visualizer
from RoboticsToolBox import Bestman_sim_panda

def main(filename):
    
    # Load config
    config_path = '../Config/debug_set_arm.yaml'
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)

    # Start recording
    visualizer.start_record(filename)
    
    # Init robot
    bestman = Bestman_sim_panda(client, visualizer, cfg)
    visualizer.change_robot_color(bestman.get_base_id(), bestman.get_arm_id(), False)
    
    # Get info about arm
    bestman.print_joint_link_info("arm")
    
    # Debug set arm joints
    bestman.sim_debug_set_arm_to_joint_values()
    
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
