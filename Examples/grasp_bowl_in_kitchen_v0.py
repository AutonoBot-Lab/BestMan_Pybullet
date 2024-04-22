"""
@Description :   This script shows how to grasp a bowl from a fridge
@Author      :   Yan Ding 
@Time        :   2023/09/01 07:47:46
"""

import os
import pybullet as p
from Motion_Planning.Robot import Bestman, Pose
from Env import Client
from Visualization import Visualizer
from Motion_Planning.Manipulation import OMPL_Planner
from Motion_Planning.Navigation import *
from Utils import load_config


def main():
    
    pb_client = PbClient(enable_GUI=True)
    pb_client.enable_vertical_view(1.0, [1.7, 3.68, 1.95], -86.4, -52.3)
    pb_visualizer = PbVisualizer(pb_client)
    # logID = pb_client.start_record("example_manipulation") # start recording
    init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
    demo = Bestman(init_pose, pb_client)  # load robot
    demo.get_joint_link_info("arm")  # get info about arm
    init_joint = [0, -1.57, 2.0, -1.57, -1.57, 0]
    demo.move_arm_to_joint_angles(init_joint)  # reset arm joint position

    # load kitchen
    kitchen = Kitchen(pb_client)
    print("object ids in loaded kitchen:\n{}".format(kitchen.object_ids))

    # load OMPL planner
    threshold_distance = 0.1
    ompl = PbOMPL(
        pb_client=pb_client,
        arm_id=demo.arm_id,
        joint_idx=demo.arm_joint_indexs,
        tcp_link=demo.tcp_link,
        obstacles=[],
        planner="BITstar",
        threshold=threshold_distance,
    )

    # add obstacles
    ompl.add_scene_obstacles(display=True)
    ompl.check_obstacles()

    # open fridge
    kitchen.open_it("elementE", 1)

    # load bowl
    bowl_position = [3.8, 2.4, 0.95]
    bowl_id = pb_client.load_object(
        "./URDF_models/utensil_bowl_blue/model.urdf",
        bowl_position,
        [0.0, 0.0, 0.0],
        1.0,
        "bowl",
        tag_obstacle_navigate=False,
    )
    pb_client.run(100)
    _, _, min_z, _, _, max_z = pb_client.get_bounding_box(bowl_id)
    bowl_position[2] = max_z + demo.tcp_height  # consider tcp's height
    print("bowl position:{}".format(bowl_position[2]))

    # navigate to standing position
    standing_position = [3.1, 2.4, 0]  # TODO: how to automatically compute it
    standing_orientation = [0.0, 0.0, 0.0]
    demo.navigate_base(Pose(standing_position, standing_orientation))

    # set target object for grasping
    ompl.set_target(bowl_id)
    target_orientation = [0.0, math.pi / 2.0, 0.0]  # vertical
    goal = demo.cartesian_to_joints(position=bowl_position, orientation=target_orientation)
    print("-" * 20 + "\n" + "Goal configuration:{}".format(goal))

    # reach target object
    result = ompl.reach_object(
        start=demo.get_arm_joint_angle(),
        goal=goal,
        end_effector_link_index=demo.end_effector_index,
    )

    # disconnect pybullet
    pb_client.wait(10)
    pb_client.disconnect_pybullet()
