"""
@Description :   This script shows how to grasp a bowl from a fridge
@Author      :   Yan Ding 
@Time        :   2023/09/01 07:47:46
"""

import os
import math
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
    # visualizer.start_record(filename)
    
    # Load table, bowl, and chair
    table_id = client.load_object(
        "../Asset/URDF_models/furniture_table_rectangle_high/table.urdf",
        [1.0, 1.0, 0.0],
        [0.0, 0.0, 0.0],
        1.0,
        "table"
    )

    # bowl_id = client.load_object(
    #     "../Asset/URDF_models/utensil_bowl_blue/model.urdf",
    #     [0.6, 0.6, 0.85],
    #     [0.0, 0.0, 0.0],
    #     1.0,
    #     "bowl"
    # )
    
    # chair_id = client.load_object(
    #     "../Asset/URDF_models/furniture_chair/model.urdf",
    #     [-0.3, 0.8, 0.5],
    #     [math.pi / 2.0 * 3, 0.0, math.pi / 2.0],
    #     1.5,
    #     "chair"
    # )
    
    # End record
    # visualizer.end_record()
    client.record_save('table_test')

    # Disconnect pybullet
    client.wait(5)
    client.disconnect()


if __name__=='__main__':
    
    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
   # get current file name
    filename = os.path.splitext(os.path.basename(__file__))[0]

    main(filename)
