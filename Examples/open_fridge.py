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

def rotate_point_3d_around_axis(init_position, initial_quaternion, rotate_axis, theta):
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
    
    x, y, z = init_position
    a, b, c = rotate_axis
    
    # 平移点，使旋转轴的原点成为全局原点
    x_prime = x - a
    y_prime = y - b
    z_prime = z - c

    # 应用绕 z 轴的旋转变换
    x_double_prime = x_prime * math.cos(theta) - y_prime * math.sin(theta)
    y_double_prime = x_prime * math.sin(theta) + y_prime * math.cos(theta)
    z_double_prime = z_prime  # z 坐标保持不变

    # 将点平移回原来的坐标系
    x_final = x_double_prime + a
    y_final = y_double_prime + b
    z_final = z_double_prime + c

    axis_quaternion = p.getQuaternionFromAxisAngle(rotate_axis, theta)
    initial_quaternion  = p.getQuaternionFromEuler(initial_quaternion)
    rotated_quaternion = p.multiplyTransforms([0, 0, 0], initial_quaternion, [0, 0, 0], axis_quaternion)[1]
    rotated_euler_angles = p.getEulerFromQuaternion(rotated_quaternion)
    
    return Pose([x_final, y_final, z_final], rotated_euler_angles)

def main(filename):
    
    # Load config
    config_path = '../Config/open_fridge.yaml'
    cfg = load_config(config_path)
    print(cfg)

    # Initial client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)
    visualizer.draw_axes()
    
    # Start recording
    visualizer.start_record(filename)

    # Load fridge
    fridge_id = client.load_object(
        "../Asset/Kitchen_models/models/Fridge/10144/mobility.urdf",
        [4.1, 5.42, 1.055],
        [0, 0, 0],
        1.1,
        "fridge",
        True
    )
    
    # Init robot
    bestman = Bestman_sim_ur5e_vacuum_long(client, visualizer, cfg)
    
    # Init visualizer
    visualizer.change_robot_color(bestman.get_base_id(), bestman.get_arm_id(), False)

    # Draw fridge aabb link
    visualizer.draw_aabb_link('fridge', 2)
    # visualizer.draw_aabb_link(3, bestman.get_tcp_link())
    # visualizer.draw_aabb_link(3, bestman.get_end_effector_link())
    
    # Init planner
    ompl_planner = OMPL_Planner(
        bestman,
        cfg.Planner
    )
    # get target object bounds
    min_x, min_y, min_z, max_x, max_y, max_z = client.get_link_bounding_box('fridge', 2)
    
    # set target object Pose
    goal_pose = Pose([min_x - bestman.get_tcp_link_height() - 0.035, (min_y + max_y) / 2, (min_z + max_z) / 2], [0.0, 0.0, 0.0])
    
    # get goal angle
    # goal = bestman.cartesian_to_joints(goal_pose)
    goal = ompl_planner.set_target_pose(goal_pose)
    
    start = bestman.get_current_joint_values()
    path = ompl_planner.plan(start, goal)
    
    # Reach fridge door
    bestman.execute_trajectory(path, True)
    
    # link_state = p.getLinkState(fridge_id, 1)
    # vec_inv, quat_inv = p.invertTransform(link_state[0], link_state[1])
    
    # current_pose = client.get_object_link_pose(bestman.get_arm_id(), bestman.get_tcp_link())
    # transform_start_to_link = p.multiplyTransforms(vec_inv, quat_inv, current_pose.position, p.getQuaternionFromEuler(current_pose.orientation))
    
    # constraint_id = p.createConstraint(
    #         parentBodyUniqueId=fridge_id,
    #         parentLinkIndex=1,
    #         childBodyUniqueId=bestman.get_arm_id(),
    #         childLinkIndex=bestman.get_tcp_link(),
    #         jointType=p.JOINT_POINT2POINT,
    #         jointAxis=[0, 0, 0],
    #         parentFramePosition=transform_start_to_link[0],
    #         parentFrameOrientation=transform_start_to_link[1],
    #         childFramePosition=[0, 0, 0],
    #     )
    # p.changeConstraint(constraint_id, maxForce=2000)
    bestman.sim_active_gripper_movable('fridge', 1, 1)
    
    # direction_vec = np.array([1, 0, 0])
    # position_end = current_pose.position - direction_vec * 0.5
    # bestman.move_end_effector_to_goal_pose(Pose(position_end, goal_pose.orientation))
    
    # rotation_matrix = np.array(p.getMatrixFromQuaternion(quat_inv)).reshape(3, 3)
    # world_surface_normal = rotation_matrix @ np.array([0, 0, 1])
    # print(world_surface_normal)
    # normal_end =  np.array(link_state[0]) + 2 * world_surface_normal
    # p.addUserDebugLine(link_state[0], normal_end, lineColorRGB=[1, 0, 0], lineWidth=2)

    init_pose = bestman.get_current_end_effector_pose()
    init_position = init_pose.position
    init_quaternion = init_pose.orientation
    # end = np.array(start) - np.array([2, 0, 0])
    # p.addUserDebugLine(start, end, lineColorRGB=[1, 0, 0], lineWidth=2)
    rotate_axis = p.getLinkState(fridge_id, 1)[4]
    
    heta_values = [math.radians(deg) for deg in range(0, 31)]
    rotated_poses = [rotate_point_3d_around_axis(init_position, init_quaternion, rotate_axis, theta) for theta in heta_values]

    visualizer.remove_all_line()

    for i in range(len(rotated_poses)):
        current_pose = rotated_poses[i]
        current_position = current_pose.position
        bestman.move_end_effector_to_goal_pose(current_pose)
        if i != 0:
            p.addUserDebugLine(last_pos, current_position, lineColorRGB=[1, 0, 0], lineWidth=2)
        last_pos = current_position
    
    # Disconnect pybullet
    client.wait(5)
    
    # End record
    visualizer.end_record()
    client.disconnect()
    
if __name__=='__main__':
    
    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    # get current file name
    filename = os.path.splitext(os.path.basename(__file__))[0]

    main(filename)