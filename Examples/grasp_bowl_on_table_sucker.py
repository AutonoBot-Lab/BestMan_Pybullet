"""
@Description :   This script shows how to manipulate the arm to grasp object, not consider collision detection
@Author      :   Yan Ding 
@Time        :   2023/08/31 03:01:50
"""

import os 
import math
from RoboticsToolBox import Bestman_sim_ur5e_vacuum_long, Pose
from Env import Client
from Visualization import Visualizer
from Motion_Planning.Navigation import *
from Utils import load_config

def main(filename):
    
    # Load config
    config_path = '../Config/grasp_bowl_on_table_sucker.yaml'
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)
    visualizer.draw_axes()
    
    # Start record
    visualizer.start_record(filename)
    
    # Init robot
    bestman = Bestman_sim_ur5e_vacuum_long(client, visualizer, cfg)
    visualizer.change_robot_color(bestman.get_base_id(), bestman.get_arm_id(), False)

    # Load table and bowl
    table_id = client.load_object(
        "table",
        "../Asset/URDF_models/furniture_table_rectangle_high/table.urdf",
        [1.0, 1.0, 0.0],
        [0.0, 0.0, 0.0],
        1.0,
        fixed_base=True,
    )

    bowl_id = client.load_object(
        "bowl",
        "../Asset/URDF_models/utensil_bowl_blue/model.urdf",
        [0.6, 0.6, 0.85],
        [0.0, 0.0, 0.0],
        1.0
    )
    
    # grasp target object
    object_goal_pose = Pose([1.4, 1.2, 1.0], [0.0, math.pi / 2.0, 0.0])
    bestman.pick_place("bowl", object_goal_pose)
    
    # End record
    visualizer.end_record()

    # disconnect from server
    client.wait(5)
    client.disconnect()
    

if __name__=='__main__':
    
    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    # get current file name
    filename = os.path.splitext(os.path.basename(__file__))[0]
    
    main(filename)

