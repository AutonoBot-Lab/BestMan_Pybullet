from utils_control import Bestman, Pose
from utils_envs import Kitchen
import math

# load robot
init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose)
demo.enable_vertical_view(4.0, [1.0, 1.0, 0])

# reset arm joint position
pose1 = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(pose1)

# load table
table_id = demo.load_object(
    "./URDF_models/furniture_table_rectangle_high/table.urdf",
    [1.0, 1.0, 0.0],
    [0.0, 0.0, 0.0],
    1.0,
    "table",
)

# load bowl
bowl_id = demo.load_object(
    "./URDF_models/utensil_bowl_blue/model.urdf",
    [0.6, 0.6, 0.85],
    [0.0, 0.0, 0.0],
    1.0,
    "bowl",
)

# get bounding box of objects
demo.get_bounding_box(table_id)
demo.get_bounding_box(bowl_id)

# get info about arm
demo.set_visual_shape()
demo.get_base_joint_info()
demo.get_arm_joint_info()
# demo.get_link_names()

# manipulate ur5e
demo.pick_place(
    4, [0.9, 0.8, 0.85], [0.0, math.pi / 2.0, 0.0]
)  # vcertical downward grip
demo.wait(20)

# print end effector info
demo.get_end_effector_info()

# check position
demo.get_bounding_box(bowl_id)
