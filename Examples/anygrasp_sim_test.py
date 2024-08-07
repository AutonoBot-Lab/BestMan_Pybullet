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

import numpy as np
import open3d as o3d

from Env import Client
from Perception.Grasp_Pose_Estimation import Anygrasp
from Perception.Object_detection import Lang_SAM
from RoboticsToolBox import Bestman_sim_panda
from Utils import load_config
from Visualization import Visualizer


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

    # Debug, look for rgb and depth
    bestman.get_camera_rgb_image(False, True, "rgb_test")
    bestman.get_camera_depth_image(False, True, "depth_test")
    bestman.visualize_3d_points()

    # Init Lang_SAM and segment
    lang_sam = Lang_SAM()
    query = str(input("Enter a Object name in the image: "))
    seg_mask, bbox = lang_sam.detect_obj(
        bestman.camera.image, query, save_box=False, save_mask=False
    )

    # Init AnyGrasp
    anygrasp = Anygrasp(cfg.Grasp_Pose_Estimation.AnyGrasp)
    best_pose = anygrasp.Grasp_Pose_Estimation(bestman.camera, seg_mask, bbox)

    print(best_pose)

    # disconnect pybullet
    client.wait(30)
    client.disconnect()


if __name__ == "__main__":

    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    main()
