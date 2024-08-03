# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : grasp_bowl_from_drawer_in_kitchen.py
# @Time           : 2024-08-03 15:03:31
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : A example to grasp bowl from drawer in kitchen
"""


import os
from RoboticsToolBox import Bestman_sim_ur5e_vacuum_long, Pose
from Env import Client
from Visualization import Visualizer
from Motion_Planning.Manipulation import OMPL_Planner
from Motion_Planning.Navigation import *
from SLAM import simple_slam
from Utils import load_config


def main(filename):
    
    # Load config
    config_path = '../Config/grasp_bowl_from_drawer_in_kitchen.yaml'
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer    
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)

    # Load scene
    scene_path = '../Asset/Scene/Kitchen.json'
    client.create_scene(scene_path)

    # Start record
    visualizer.start_record(filename)
    
    # Init robot
    bestman = Bestman_sim_ur5e_vacuum_long(client, visualizer, cfg)
    
    # Open the drawer
    client.change_object_joint_angle("elementA", 36, 0.4)

    visualizer.draw_aabb_link("elementA", 36)
    
    # Simple SLAM
    nav_obstacles_bounds = simple_slam(client, bestman, True)
    
    # Navigate to standing position
    standing_pose = Pose([2.85, 2.4, 0], [0.0, 0.0, 0.0])
    nav_planner = AStarPlanner(
        robot_size = bestman.get_robot_max_size(), 
        obstacles_bounds = nav_obstacles_bounds, 
        resolution = 0.05, 
        enable_plot = False
    )
    # nav_planner = RRTPlanner(
    #     robot_size = bestman.get_robot_max_size(), 
    #     obstacles_bounds = client.get_Nav_obstacles_bounds(), 
    #     enable_plot=False
    # )
    path = nav_planner.plan(bestman.get_current_base_pose(), standing_pose)
    bestman.navigate_base(standing_pose, path, enable_plot=True)

    # Load bowl
    bowl_id = client.load_object(
        "bowl",
        "../Asset/URDF_models/utensil_bowl_blue/model.urdf",
        [3.6, 2.4, 0.6],
        [0.0, 0.0, 0.0],
        1.0,
        False
    )
    
    # Init planner
    ompl_planner = OMPL_Planner(
        bestman,
        cfg.Planner
    )
    
    # Get obstacles info
    ompl_planner.get_obstacles_info()
    
    # Get rgb image
    # bestman.update_camera()
    # bestman.get_camera_rgb_image(True, True)

    # Planning
    goal = ompl_planner.set_target("bowl")
    start = bestman.get_current_joint_values()
    path = ompl_planner.plan(start, goal)
    
    # Robot execute
    bestman.execute_trajectory(path, enable_plot=True)
    
    # grasp target object
    bestman.sim_active_gripper_fixed("bowl", 1)
    
    # End record
    visualizer.end_record()
    
    # disconnect
    client.wait(20)
    client.disconnect()


if __name__=='__main__':
    
    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    # get current file name
    filename = os.path.splitext(os.path.basename(__file__))[0]

    main(filename)