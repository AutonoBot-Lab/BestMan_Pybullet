import os
import pickle
import numpy as np
import torch
import torchvision.transforms as transforms
from torch import nn
import torch.nn.functional as F
from utils_control import Bestman, Pose
from utils_envs import Kitchen
import math
import random
import pybullet as p


# ----------------------------------------------------------------
# Generate random positions for objects within the specified radius and get images
# ----------------------------------------------------------------
# load robot
init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose)
demo.enable_vertical_view(4.0, [1.0, 1.0, 0])

# reset arm joint position
pose1 = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(pose1)

# ----------------------------------------------------------------
# Load plan and table
# ----------------------------------------------------------------
# load table
table_id = demo.load_object(
    "./URDF_models/furniture_table_rectangle_high/table.urdf",
    [1.0, 1.0, 0.0],
    [0.0, 0.0, 0.0],
    1.0, 
    "table",
)
[_, _, _, _, _, table_height] = demo.get_bounding_box(table_id)
table_height += 0.08


# ----------------------------------------------------------------
# Objects
# ----------------------------------------------------------------
object_names = ["blue_cup", "blue_plate", "plastic_apple", "plastic_peach"]
models = {"blue_cup": "./URDF_models/blue_cup/model.urdf",
        "blue_plate": "./URDF_models/blue_plate/model.urdf",
        "plastic_apple": "./URDF_models/plastic_apple/model.urdf",
        "plastic_peach": "./URDF_models/plastic_peach/model.urdf"}

# ----------------------------------------------------------------
# Generate random positions for objects within table
# ----------------------------------------------------------------
n_objects = 10 # number of objects on table
object_ids = []
for _ in range(n_objects):
    object_name = random.choice(object_names) # Choose a random object
    collision_flag = True
    max_attempts = 200
    for attempt in range(max_attempts):
        table_min_x, table_max_x, table_min_y, table_max_y = 0.249, 1.751, 0.499, 1.501
        position_other = demo.generate_point_within_area_with_fixed_z(table_min_x, table_min_y, table_max_x, table_max_x, table_height)
        orientation_other = p.getQuaternionFromEuler([0, 0, np.random.uniform(0, 2 * np.pi)])
        # orientation_other = p.getQuaternionFromEuler([0, 0, 0])
        object_id = p.loadURDF(models[object_name], position_other, orientation_other)
        # simulate(1)
        demo.draw_aabb(object_id)
        print('-' * 20 + '\n' + 'object_id:{} is waitting for to be added.'.format(object_id))

        # ----------------------------------------------------------------
        # Ensure that the position is free of other objects
        # ----------------------------------------------------------------
        object_boundary = demo.get_bounding_box(object_id)
        collision_flag = any(
                demo.check_collision_xy(object_boundary, demo.get_bounding_box(obstacle))
                for obstacle in object_ids
            )
        if not collision_flag:
            # No collision detected, add the object and exit the loop
            object_ids.append(object_id)
            print('object_id:{} has been added.'.format(object_id))
            print("object_name:{} position:{}".format(object_name, position_other))
            break
        else:
            # Collision detected, remove the object and try again
            p.removeBody(object_id)
            demo.run(2)
            print('-' * 20 + '\n' + 'object_id:{} has been removed.'.format(object_id))
    
    if collision_flag and attempt == max_attempts - 1:
        print("-" * 20 + "\n" + "Failed to place objects after maximum attempts!")

# ----------------------------------------------------------------
# Set camera parameters
# ----------------------------------------------------------------
tablePos, _ = p.getBasePositionAndOrientation(table_id) # table center
cameraPos = [tablePos[0], tablePos[1], table_height + 0.7]
cameraUp = [0, 1, 0]  # bird-view
demo.run(2)
print("-" * 20 + "\n" + "Image A, where its center is {}".format(tablePos))
depth_image_A = demo.get_depth_image(tablePos, cameraPos, cameraUp, enable_show=True) # Get depth image A

demo.wait(100)
# ----------------------------------------------------------------
# get depth_image_B, with a given object name
# ----------------------------------------------------------------


# ----------------------------------------------------------------
# predict buffer area
# ----------------------------------------------------------------