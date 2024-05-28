"""
@Description :   This script shows how to navigate to a goal position
@Author      :   Yan Ding 
@Time        :   2023/08/31 03:01:50
"""

import os 
import math
from RoboticsToolBox import Bestman, Pose
from Env.Client import Client
from Visualization import Visualizer
from Motion_Planning.Navigation import *
from Utils import load_config


def main():
    
    # Load config
    config_path = '../Config/navigation_basic.yaml'
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)

    # Load scene
    scene_path = '../Asset/Kitchen_models/scenes/kitchen_counter.lisdf'
    client.create_scene_lisdf(scene_path)

    # logID = pb_client.start_record("example_manipulation")    # start recording
    # Init robot
    bestman = Bestman(client, visualizer, cfg)
    visualizer.change_robot_color(bestman.get_base_id(), bestman.get_arm_id(), False)

    # load table, bowl, and chair
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
        "bowl",
        nav_obstacle_tag=False,
    )

    chair_id = client.load_object(
        "../Asset/URDF_models/furniture_chair/model.urdf",
        [-0.3, 0.8, 0.1],
        [math.pi / 2.0 * 3, 0.0, math.pi / 2.0],
        1.5,
        "chair",
        nav_obstacle_tag=False,
    )

    # get bounding box of objects
    aabb_table = client.get_bounding_box(table_id)
    visualizer.draw_aabb(table_id)
    print("-" * 20 + "\n" + "aabb_table:{}".format(aabb_table))
    
    client.wait(1000)
    client.disconnect()
    
if __name__=='__main__':
    
    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    main()
