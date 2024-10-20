# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : open_drawer.py
# @Time           : 2024-08-03 15:05:17
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : A example to open drawer in kitchen
"""


import math
import os

import numpy as np
import pybullet as p

from Config import load_config
from Env import Client
from Motion_Planning.Manipulation.OMPL_Planner import OMPL_Planner
from Motion_Planning.Navigation import AStarPlanner
from Robotics_API import Bestman_sim_ur5e_vacuum_long, Pose
from SLAM import simple_slam
from Visualization import Visualizer


def pull_out(init_pose, i, distance):
    rotation_matrix = init_pose.get_orientation("rotation_matrix")
    front_direction = rotation_matrix[:, 0]
    new_position = np.array(init_pose.get_position()) - front_direction * i * distance
    return Pose(new_position, init_pose.get_orientation())


def main(filename):

    # Load config
    config_path = "Config/open_drawer.yaml"
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
        "Asset/Scene/Kitchen_models/models_yan/elementA/urdf/kitchen_part_right_gen_convex.urdf",
        [4, 2, 1.477],
        [0, 0, math.pi],
        1.0,
        True,
    )

    # Init robot
    ur5e = Bestman_sim_ur5e_vacuum_long(client, visualizer, cfg)

    # Init visualizer
    visualizer.change_robot_color(
        ur5e.sim_get_base_id(), ur5e.sim_get_arm_id(), False
    )

    # Draw drawer link
    visualizer.draw_aabb_link("elementA", 38)
    
    # Init planner
    ompl_planner = OMPL_Planner(ur5e, cfg.Planner)

    # Get goal joint values
    min_x, min_y, min_z, max_x, max_y, max_z = client.get_link_bounding_box(
        "elementA", 38
    )
    
    goal_pose = Pose(
        [min_x - ur5e.sim_get_tcp_link_height()- 0.05, (min_y + max_y) / 2, (min_z + max_z) / 2], [0.0, 0.0, 0.0]
    )
    goal = ompl_planner.set_target_pose(goal_pose)

    # Plan / Execute / Suctate drawer
    start = ur5e.sim_get_current_joint_values()
    path = ompl_planner.plan(start, goal)
    ur5e.sim_execute_trajectory(path, True)
    ur5e.sim_create_movable_constraint("elementA", 36)

    # The end effector Move along the specified trajectory get effector to open the drawer
    init_pose = ur5e.sim_get_current_end_effector_pose()
    pull_joints = [
        ur5e.sim_cartesian_to_joints(pull_out(init_pose, i, 0.05))
        for i in range(0, 5)
    ]
    ur5e.sim_execute_trajectory(pull_joints, True)

    # Wait
    client.wait(5)

    # End record / Disconnect pybullet
    visualizer.end_record()
    client.disconnect()


if __name__ == "__main__":

    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # get current file name
    filename = os.path.splitext(os.path.basename(__file__))[0]

    main(filename)
