"""
@Description :   This script shows how to grasp a bowl from a fridge
@Author      :   Yan Ding 
@Time        :   2023/09/01 07:47:46
"""

import os
from RoboticsToolBox.Bestman_sim_panda import Bestman_sim_panda
from Env.Client import Client
from Visualization.Visualizer import Visualizer
from Utils.load_config import load_config

def main():
    
    # load config
    config_path = '../Config/load_panda.yaml'
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)

    # logID = pb_client.start_record("example_manipulation")    # start recording
    # Init robot
    panda = Bestman_sim_panda(client, visualizer, cfg)
    visualizer.change_robot_color(panda.get_base_id(), panda.get_arm_id(), False)

    for _ in range(10):
        panda.sim_active_gripper(0)
        client.wait(2)
        panda.sim_active_gripper(1)
        client.wait(2)
    
    
    # disconnect pybullet
    client.wait(1000)
    client.disconnect()


if __name__=='__main__':
    
    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    main()
