#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
# @FileName      : open_drawer
# @Time          : 2024-08-01 20:20:03
# @Author        : kui yang
# @Email         : yangkui1127@gmail.com
# @description   : A example to open drawer in kitchen
"""

import os
import math
import numpy as np
import pybullet as p
from Env import Client
from Utils import load_config
from Visualization import Visualizer
from SLAM import simple_slam
from Motion_Planning.Manipulation import OMPL_Planner
from Motion_Planning.Navigation import AStarPlanner
from RoboticsToolBox import Pose, Bestman_sim_ur5e_vacuum_long

def pull_out(init_pose, i, distance):
    """
    旋转点 (x, y, z) 绕过原点 (a, b, c) 旋转角度 theta(弧度)。
    
    参数：
    x -- 初始点的 x 坐标
    y -- 初始点的 y 坐标
    z -- 初始点的 z 坐标
    a -- 旋转轴原点的 x 坐标
    b -- 旋转轴原点的 y 坐标
    c -- 旋转轴原点的 z 坐标
    theta -- 旋转角度（弧度）
    
    返回：
    (x_final, y_final, z_final) -- 旋转后的点的坐标
    """
    
    
    init_quaternion = p.getQuaternionFromEuler(init_pose.orientation)
    rotation_matrix = np.array(p.getMatrixFromQuaternion(init_quaternion)).reshape(3, 3)
    front_direction = rotation_matrix[:, 0]
    
    new_position = np.array(init_pose.position) - front_direction * i * distance
    
    return Pose(new_position, init_pose.orientation)

def main(filename):
    
    # Load config
    config_path = '../Config/open_drawer.yaml'
    cfg = load_config(config_path)
    print(cfg)
    
    # Initial client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)
    visualizer.draw_axes()
    
    # Start recording
    visualizer.start_record(filename)

    # Load drawer
    drawer_id = client.load_object(
        "elementA",
        "../Asset/Kitchen_models/models_yan/elementA/urdf/kitchen_part_right_gen_convex.urdf",
        [4, 2, 1.477],
        [0, 0, math.pi],
        1.0,
        True
    )
    
    # Init robot
    bestman = Bestman_sim_ur5e_vacuum_long(client, visualizer, cfg)
    
    # Init visualizer
    visualizer.change_robot_color(bestman.get_base_id(), bestman.get_arm_id(), False)

    # Draw drawer link
    visualizer.draw_aabb_link('elementA', 36)
    
    # Init planner
    ompl_planner = OMPL_Planner(
        bestman,
        cfg.Planner
    )
    
    # Get goal joint values
    min_x, min_y, min_z, max_x, max_y, max_z = client.get_link_bounding_box('elementA', 36)
    goal_pose = Pose([min_x + 0.01, (min_y + max_y) / 2, (min_z + max_z) / 2], [0.0, 0.0, 0.0])
    goal = ompl_planner.set_target_pose(goal_pose)
    
    # Plan / Execute / Suctate drawer
    start = bestman.get_current_joint_values()
    path = ompl_planner.plan(start, goal)
    bestman.execute_trajectory(path, True)
    bestman.sim_active_gripper_movable('elementA', 36, 1)

    visualizer.remove_all_line()
    
    # The end effector Move along the specified trajectory get effector to open the drawer
    init_pose = bestman.get_current_end_effector_pose()
    pull_joints = [bestman.cartesian_to_joints(pull_out(init_pose, i, 0.004)) for i in range(0, 50)]
    bestman.execute_trajectory(pull_joints, True)
    
    # Wait
    client.wait(5)
    
    # End record / Disconnect pybullet
    visualizer.end_record()
    client.disconnect()
    
if __name__=='__main__':
    
    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    # get current file name
    filename = os.path.splitext(os.path.basename(__file__))[0]

    main(filename)