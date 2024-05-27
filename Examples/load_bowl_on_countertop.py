"""
@Description :   This script shows how to navigate to a goal position
@Author      :   Yan Ding 
@Time        :   2023/08/31 03:01:50
"""


import os 
import math
from Motion_Planning.Robot import Bestman, Pose
from Env import Client
from Visualization import Visualizer
from Motion_Planning.Navigation import *
from Utils import load_config


def main():
    
     # Load config
    config_path = '../Config/load_bowl_on_countertop.yaml'
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)
    
    # start_record
    # logID = visualizer.start_record(os.path.splitext(os.path.basename(__file__))[0])    # start recording
    
    # Init robot
    bestman = Bestman(client, visualizer, cfg)
    visualizer.change_robot_color(bestman.get_base_id(), bestman.get_arm_id(), False)

    # load table, bowl, and chair
    countertop_id = client.load_object(
        "../Asset/Kitchen_models/models_yan/elementB/model.urdf",
        [0.0, 0.0, 0.7],
        [0.0, 0.0, math.pi / 2],
        1.0,
        "countertop",
        fixed_base=True,
    )
    
    client.get_bounding_box(countertop_id, True)

    visualizer.set_object_color(countertop_id, "light_white")

    bowl_id = client.load_object(
        "../Asset/URDF_models/utensil_bowl_blue/model.urdf",
        [0.0, 0.5, 1.05],
        [0.0, 0.0, 0.0],
        1.0,
        "bowl",
        nav_obstacle_tag=False,
    )

    # disconnect from server
    client.wait(50)
    client.disconnect()
    
    
if __name__=='__main__':
    
    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    main()

