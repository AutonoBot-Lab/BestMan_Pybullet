"""
@Description :   This script shows how to grasp a bowl from a fridge
@Author      :   Yan Ding 
@Time        :   2023/09/01 07:47:46
"""

import os
import math
from RoboticsToolBox import Bestman_sim_ur5e_vacuum_long, Pose
from Env import Client
from Visualization import Visualizer
from Motion_Planning.Manipulation import OMPL_Planner
from Motion_Planning.Navigation import *
from Utils import load_config


def main(filename):
    
    # Load config
    config_path = '../Config/grasp_bowl_in_kitchen.yaml'
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)\
    
    # Start record
    visualizer.start_record(filename)

    # Load scene
    scene_path = '../Asset/Scene/Kitchen.json'
    client.create_scene(scene_path)
    
    # Init robot
    bestman = Bestman_sim_ur5e_vacuum_long(client, visualizer, cfg)
    visualizer.change_robot_color(bestman.get_base_id(), bestman.get_arm_id(), False)

    # load OMPL planner
    ompl_planner = OMPL_Planner(
        bestman,
        cfg.Planner
    )
    
    # Get obstacles info
    ompl_planner.get_obstacles_info()

    # Open fridge
    client.change_object_joint_angle("elementE", 1, math.pi / 2.0)

    # Navigation
    standing_pose = Pose([3.1, 2.4, 0], [0.0, 0.0, 0.0])
    nav_planner = AStarPlanner(
        robot_size = bestman.get_robot_max_size(), 
        obstacles_bounds = client.get_Nav_obstacles_bounds(), 
        resolution = 0.05, 
        enable_plot = False
    )
    path = nav_planner.plan(bestman.get_base_pose(), standing_pose)
    bestman.navigate_base(standing_pose, path)
    
    # Init ompl planner
    ompl_planner = OMPL_Planner(
        bestman,
        cfg.Planner
    )
    
    # load bowl
    bowl_id = client.load_object(
        "../Asset/URDF_models/utensil_bowl_blue/model.urdf",
        [3.8, 2.4, 0.95],
        [0.0, 0.0, 0.0],
        1.0,
        "bowl",
        nav_obstacle_tag=False,
    )
    
    # # get rgb image
    # bestman.update_camera()
    # bestman.get_camera_rgb_image(False, False)
    
    # # set target object for grasping
    # ompl_planner.set_target(bowl_id)
    
    # # reach target object
    # ompl_planner.plan_execute()
    
    # Planning
    goal = ompl_planner.set_target("bowl")
    start = bestman.get_current_joint_values()
    path = ompl_planner.plan(start, goal)
    
    # Robot execute
    bestman.execute_trajectory(path)
    
    # grasp target object
    bestman.sim_active_gripper(bowl_id, 1)
    
    # End record
    visualizer.end_record()
    
    # disconnect pybullet
    client.wait(5)
    client.disconnect()
    

if __name__=='__main__':
    
    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    # get current file name
    filename = os.path.splitext(os.path.basename(__file__))[0]
    
    main(filename)
