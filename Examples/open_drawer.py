"""
@Description :   load a kitchen scenario
@Author      :   Yan Ding 
@Time        :   2023/08/30 20:43:44
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
        "../Asset/Kitchen_models/models_yan/elementA/urdf/kitchen_part_right_gen_convex.urdf",
        [4, 2, 1.477],
        [0, 0, math.pi],
        1.0,
        "elementA",
        True
    )
    
    # Init robot
    bestman = Bestman_sim_ur5e_vacuum_long(client, visualizer, cfg)
    
    # Init visualizer
    visualizer.change_robot_color(bestman.get_base_id(), bestman.get_arm_id(), False)

    # Draw fridge aabb link
    # visualizer.draw_aabb_link('elementA', 36)
    visualizer.draw_aabb_link('elementA', 36)
    # visualizer.draw_aabb_link(3, bestman.get_tcp_link())
    # visualizer.draw_aabb_link(3, bestman.get_end_effector_link())
    
    # Init planner
    ompl_planner = OMPL_Planner(
        bestman,
        cfg.Planner
    )
    # get target object bounds
    min_x, min_y, min_z, max_x, max_y, max_z = client.get_link_bounding_box('elementA', 36)
    
    # set target object Pose
    goal_pose = Pose([min_x + 0.01, (min_y + max_y) / 2, (min_z + max_z) / 2], [0.0, 0.0, 0.0])
    
    # get goal angle
    # goal = bestman.cartesian_to_joints(goal_pose)
    goal = ompl_planner.set_target_pose(goal_pose)
    
    start = bestman.get_current_joint_values()
    path = ompl_planner.plan(start, goal)
    
    # Reach fridge door
    bestman.execute_trajectory(path, True)
    
    # bestman.sim_test_active_gripper('fridge', 2, 1)
    
    # link_state = p.getLinkState(drawer_id, 36)
    # vec_inv, quat_inv = p.invertTransform(link_state[0], link_state[1])
    
    # current_pose = client.get_object_link_pose(bestman.get_arm_id(), bestman.get_tcp_link())
    # transform_start_to_link = p.multiplyTransforms(vec_inv, quat_inv, current_pose.position, p.getQuaternionFromEuler(current_pose.orientation))
    
    # constraint_id = p.createConstraint(
    #         parentBodyUniqueId=drawer_id,
    #         parentLinkIndex=36,
    #         childBodyUniqueId=bestman.get_arm_id(),
    #         childLinkIndex=bestman.get_tcp_link(),
    #         jointType=p.JOINT_POINT2POINT,
    #         jointAxis=[0, 0, 0],
    #         parentFramePosition=transform_start_to_link[0],
    #         parentFrameOrientation=transform_start_to_link[1],
    #         childFramePosition=[0, 0, 0],
    #     )
    # p.changeConstraint(constraint_id, maxForce=2000)
    bestman.sim_active_gripper_movable('elementA', 36, 1)
    
    # direction_vec = np.array([1, 0, 0])
    # position_end = current_pose.position - direction_vec * 0.5
    # bestman.move_end_effector_to_goal_pose(Pose(position_end, goal_pose.orientation))
    
    # rotation_matrix = np.array(p.getMatrixFromQuaternion(quat_inv)).reshape(3, 3)
    # world_surface_normal = rotation_matrix @ np.array([0, 0, 1])
    # print(world_surface_normal)
    # normal_end =  np.array(link_state[0]) + 2 * world_surface_normal
    # p.addUserDebugLine(link_state[0], normal_end, lineColorRGB=[1, 0, 0], lineWidth=2)

    init_pose = bestman.get_current_end_effector_pose()
    # end = np.array(start) - np.array([2, 0, 0])
    # p.addUserDebugLine(start, end, lineColorRGB=[1, 0, 0], lineWidth=2)
    
    rotated_poses = [pull_out(init_pose, i, 0.004) for i in range(0, 50)]
    
    visualizer.remove_all_line()
    
    for i in range(len(rotated_poses)):
        current_pose = rotated_poses[i]
        current_position = current_pose.position    
        start = bestman.get_current_joint_values()
        goal = ompl_planner.set_target_pose(current_pose)
        path = ompl_planner.plan(start, goal)
        # bestman.move_end_effector_to_goal_pose(current_pose)
        bestman.execute_trajectory(path)
        if i != 0:
            p.addUserDebugLine(last_pos, current_position, lineColorRGB=[1, 0, 0], lineWidth=2)
        last_pos = current_position
    
    # Disconnect pybullet
    client.wait(10)
    
    # End record
    visualizer.end_record()
    client.disconnect()
    
if __name__=='__main__':
    
    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    # get current file name
    filename = os.path.splitext(os.path.basename(__file__))[0]

    main(filename)