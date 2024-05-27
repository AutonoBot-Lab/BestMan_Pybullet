"""
@Description :   This script shows how to manipulate the arm to grasp object, not consider collision detection
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
    config_path = '../Config/grasp_bowl_on_table.yaml'
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

    # load table and bowl
    table_id = client.load_object(
        "../Asset/URDF_models/furniture_table_rectangle_high/table.urdf",
        [1.0, 1.0, 0.0],
        [0.0, 0.0, 0.0],
        1.0,
        "table",
        fixed_base=True,
    )

    bowl_id = client.load_object(
        "../Asset/URDF_models/utensil_bowl_blue/model.urdf",
        [0.6, 0.6, 0.85],
        [0.0, 0.0, 0.0],
        1.0,
        "bowl"
    )

    # grasp target object
    object_goal_pose = Pose([0.9, 0.7, 0.85], [0.0, math.pi / 2.0, 0.0])
    bestman.pick_place(bowl_id, object_goal_pose)

    # end record
    # visualizer.end_record(logID)

    # disconnect from server
    client.wait(50)
    client.disconnect()
    

if __name__=='__main__':
    
    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    main()
