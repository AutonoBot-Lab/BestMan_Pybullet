"""
@Description :   load a kitchen scenario
@Author      :   Yan Ding 
@Time        :   2023/08/30 20:43:44
"""

import os
import math
import numpy as np
from Env import Client
from Utils import load_config
from Visualization import Visualizer
from SLAM import simple_slam
from Motion_Planning.Manipulation import OMPL_Planner
from Motion_Planning.Navigation import AStarPlanner
from RoboticsToolBox import Pose, Bestman_sim_ur5e_vacuum_long

def main(filename):
    
    # Load config
    cfg = load_config()
    print(cfg)

    # Initial client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)
    visualizer.draw_axes()
    
    # Start recording
    visualizer.start_record(filename)

    # Load scene
    scene_path = '../Asset/Scene/Kitchen.json'
    client.create_scene(scene_path)
    
    # Init robot
    bestman = Bestman_sim_ur5e_vacuum_long(client, visualizer, cfg)
    
    # Init visualizer
    visualizer.change_robot_color(bestman.get_base_id(), bestman.get_arm_id(), False)

    # # Open fridge
    # client.change_object_joint_angle('fridge', 1, math.pi / 2)

    # Draw fridge aabb link
    visualizer.draw_aabb_link('fridge', 2)

    # Simple SLAM
    nav_obstacles_bounds = simple_slam(client, bestman, False)
    
    # Navigate to fridge
    stand_pose1 = Pose([3.0, 5.36, 0.0], [0.0, 0.0, 0.0])
    nav_planner = AStarPlanner(
        robot_size = bestman.get_robot_max_size(), 
        obstacles_bounds = nav_obstacles_bounds, 
        resolution = 0.05, 
        enable_plot = False
    )
    path = nav_planner.plan(bestman.get_current_base_pose(), stand_pose1)
    bestman.navigate_base(stand_pose1, path)

    # Init planner
    ompl_planner = OMPL_Planner(
        bestman,
        cfg.Planner
    )
    # get target object bounds
    min_x, min_y, min_z, max_x, max_y, max_z = client.get_link_bounding_box('fridge', 1)
    
    # set target object Pose
    goal_pose = Pose([min_x - bestman.get_tcp_link_height() - 0.05, (min_y + max_y) / 2, (min_z + max_z) / 2], [0.0, 0.0, 0.0])
    
    # get goal angle
    goal = bestman.cartesian_to_joints(goal_pose)
    
    # goal = ompl_planner.set_target_pose()
    start = bestman.get_current_joint_values()
    path = ompl_planner.plan(start, goal)
    print(len(path))
    
    # Reach fridge door
    # bestman.move_end_effector_to_goal_pose(Pose([3.55, 5.3, 1.0], [0, 0, 0]))
    bestman.execute_trajectory(path, True)
    
    bestman.sim_test_active_gripper('fridge', 2, 1)
    
    # client.change_object_joint_angle('fridge', 1, math.pi / 2)
    # bestman.move_end_effector_to_goal_pose(Pose(np.array(goal_pose.position) - np.array([0.2, 0.0, 0.0]), goal_pose.orientation))
    
    # End record
    visualizer.end_record()

    # Disconnect pybullet
    client.wait(20)
    client.disconnect()
    
if __name__=='__main__':
    
    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    # get current file name
    filename = os.path.splitext(os.path.basename(__file__))[0]

    main(filename)