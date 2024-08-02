#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
# @FileName      : open_fridge
# @Time          : 2024-08-01 20:21:10
# @Author        : kui yang
# @Email         : yangkui1127@gmail.com
# @description   : A example to open fridge use ur5e robot
"""

import os
import math
import numpy as np
import pybullet as p
from Env import Client
from Utils import load_config
from Visualization import Visualizer
from Motion_Planning.Manipulation import OMPL_Planner
from RoboticsToolBox import Pose, Bestman_sim_ur5e_vacuum_long


def rotate_point_3d_around_axis(init_pose, rotate_axis, theta, clockwise=True):
    """Rotate the point (x, y, z) around the rotation axis (a, b, c) by theta angle (radians).
    
    Parameters:
        init_pose -- the initial point's pose (including position and Euler angle pose)
        rotate_axis -- the origin coordinates of the rotation axis (a, b, c)
        theta -- the rotation angle (radians)

    Returns:
        Pose -- the pose of the rotated point (including position and Euler angle pose)
    """
    
    init_position = np.array(init_pose.position)
    rotate_axis = np.array(rotate_axis)
    
    translated_position = init_position - rotate_axis
    
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    
    if clockwise:
        rotation_matrix = np.array([
            [cos_theta, sin_theta, 0],
            [-sin_theta, cos_theta, 0],
            [0, 0, 1]
        ])
        rotated_quaternion = p.getQuaternionFromEuler([0, 0, -theta/2])
        rotated_orientation = p.multiplyTransforms([0, 0, 0], rotated_quaternion, [0, 0, 0], p.getQuaternionFromEuler(init_pose.orientation))[1]
    else:
        rotation_matrix = np.array([
            [cos_theta, -sin_theta, 0],
            [sin_theta, cos_theta, 0],
            [0, 0, 1]
        ])
        rotated_quaternion = p.getQuaternionFromEuler([0, 0, theta/2])
        rotated_orientation = p.multiplyTransforms([0, 0, 0], rotated_quaternion, [0, 0, 0], p.getQuaternionFromEuler(init_pose.orientation))[1]
    
    rotated_position = np.dot(rotation_matrix, translated_position)
    final_position = rotated_position + rotate_axis
    rotated_euler_angles = p.getEulerFromQuaternion(rotated_orientation)
    return Pose(final_position, rotated_euler_angles)


def main(filename):
    
    # Load config
    config_path = '../Config/open_fridge.yaml'
    cfg = load_config(config_path)
    print(cfg)

    # Initial client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)
    visualizer.draw_axes()
    
    # Load scene
    scene_path = '../Asset/Scene/Kitchen.json'
    client.create_scene(scene_path)
    
    # Start recording
    visualizer.start_record(filename)
    
    # Init robot
    bestman = Bestman_sim_ur5e_vacuum_long(client, visualizer, cfg)
    
    # Init visualizer
    visualizer.change_robot_color(bestman.get_base_id(), bestman.get_arm_id(), False)

    # Draw fridge door handle
    visualizer.draw_aabb_link('fridge', 2)
    
    # Init planner
    ompl_planner = OMPL_Planner(
        bestman,
        cfg.Planner
    )
    
    # Get goal joint values
    min_x, min_y, min_z, max_x, max_y, max_z = client.get_link_bounding_box('fridge', 2)
    goal_pose = Pose([min_x - bestman.get_tcp_link_height() - 0.05, (min_y + max_y) / 2, (min_z + max_z) / 2], [0.0, 0.0, 0.0])
    goal = ompl_planner.set_target_pose(goal_pose)
    
    # Plan / Execute / Suctate fridge handle
    start = bestman.get_current_joint_values()
    path = ompl_planner.plan(start, goal)
    bestman.execute_trajectory(path, True)
    bestman.sim_active_gripper_movable('fridge', 1, 1)

    visualizer.remove_all_line()
    
    # The end effector Move along the specified trajectory get effector to open the door
    init_pose = bestman.get_current_end_effector_pose()
    rotate_axis = p.getLinkState(client.get_object_id("fridge"), 1)[4]
    angles = 15
    heta_values = [math.radians(deg) for deg in range(0, angles+1)]
    rotated_joints = [bestman.cartesian_to_joints(rotate_point_3d_around_axis(init_pose, rotate_axis, theta, False)) for theta in heta_values]
    bestman.execute_trajectory(rotated_joints, True)
    
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