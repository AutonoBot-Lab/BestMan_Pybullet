# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : slam.py
# @Time           : 2024-08-03 15:08:53
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : A simple slam method in pybullet sim
"""

import pybullet as p
import matplotlib.pyplot as plt
from .utils import *


def simple_slam(client, robot, enable_plot=False):
    """
    Perform a simple SLAM (Simultaneous Localization and Mapping) operation.

    Args:
        client (pybullet): The pybullet client object.
        robot (object): The robot object.
        enable_plot (bool, optional): Flag to enable or disable plotting of the SLAM visualization. Defaults to False.

    Returns:
        list: A list of obstacle bounds in the format [x_min, y_min, x_max, y_max].
    """
    nav_obstacles_bounds = []
    nav_obstacle_ids = list(range(p.getNumBodies()))
    nav_obstacle_ids.remove(0)  # remove plane
    nav_obstacle_ids.remove(robot.get_base_id())  # remove base
    nav_obstacle_ids.remove(robot.get_arm_id())  # remove arm

    for object_id in nav_obstacle_ids:
        object_link_bounds = client.get_all_link_bounding_box(object_id)
        for link_bounds in object_link_bounds:
            nav_obstacles_bounds.append(
                [
                    link_bounds[0][0],
                    link_bounds[0][1],
                    link_bounds[1][0],
                    link_bounds[1][1],
                ]
            )

    if enable_plot:
        plt.clf()
        for x_min, y_min, x_max, y_max in nav_obstacles_bounds:
            plot_rectangle(x_min, y_min, x_max, y_max)
        area = AreaBounds(nav_obstacles_bounds)
        plt.axis([area.x_min, area.x_max, area.y_min, area.y_max])
        plt.axis("equal")
        # plt.grid(True)    # grid line
        plt.title("SLAM Visualization")
        plt.pause(0.01)
        plt.show()

    return nav_obstacles_bounds
