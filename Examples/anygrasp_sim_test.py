# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : load_kitchen.py
# @Time           : 2024-08-03 15:04:25
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : A example to load kitchen
"""

import os

from Env import Client
from Perception.Grasp_Pose_Estimation import Anygrasp
from Perception.Object_detection import Lang_SAM
from RoboticsToolBox import Bestman_sim_panda, Pose
from Utils import load_config
from Visualization import Visualizer
from Motion_Planning.Manipulation import OMPL_Planner
import math

def main():

    # Load config
    config_path = "../Config/anygrasp_sim_test.yaml"
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)

    # Load scene
    scene_path = "../Asset/Scene/Kitchen_anygrasp.json"
    client.create_scene(scene_path)

    # Init robot
    bestman = Bestman_sim_panda(client, visualizer, cfg)
    end_effector_pose = bestman.sim_get_current_end_effector_pose()
    end_effector_pose.print("init end effector pose")
    visualizer.draw_pose(end_effector_pose)
    
    # Debug, look for rgb and depth
    # bestman.sim_get_camera_rgb_image(False, True, "rgb_test")
    # bestman.sim_get_camera_depth_image(False, True, "depth_test")
    # bestman.sim_visualize_camera_3d_points()
    
    # debug, look for camera pose
    # camera_pose = bestman.sim_get_camera_pose()
    # visualizer.draw_pose(camera_pose)
    
    # Init Lang_SAM and segment
    lang_sam = Lang_SAM()
    query = str(input("\033[34mInfo: Enter a Object name in the image: \033[0m"))
    seg_mask, bbox = lang_sam.detect_obj(
        bestman.camera.image,
        query,
        save_box=True,
        box_filename="./output/sim_test/box.png",
        save_mask=True,
        mask_filename="./output/sim_test/mask.png",
    )

    # Init AnyGrasp
    anygrasp = Anygrasp(cfg.Grasp_Pose_Estimation.AnyGrasp)
    best_pose = anygrasp.Grasp_Pose_Estimation(bestman.camera, seg_mask, bbox)
    best_pose = bestman.sim_trans_camera_to_world(best_pose)
    best_pose = bestman.align_grasp_pose_to_tcp([0, 0, -1], best_pose)
    visualizer.draw_pose(best_pose)
    
    # Init ompl
    # ompl_planner = OMPL_Planner(bestman, cfg.Planner)
    # goal = ompl_planner.set_target_pose(best_pose)
    # ompl_planner.remove_obstacle("banana")
    # start = bestman.get_current_joint_values()
    # path = ompl_planner.plan(start, goal)
    # bestman.execute_trajectory(path, enable_plot=True)
    
    bestman.sim_open_gripper()
    bestman.sim_move_end_effector_to_goal_pose(best_pose, 50)
    visualizer.draw_link_pose(bestman.sim_get_arm_id(), bestman.sim_get_end_effector_link())
    
    # client.wait(5)
    bestman.sim_close_gripper()
    
    # disconnect pybullet
    client.wait(100)
    client.disconnect()


if __name__ == "__main__":

    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    main()
