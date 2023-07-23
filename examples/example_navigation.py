from utils_control import Bestman, Pose
from utils_envs import Kitchen
import math

# load robot
init_pose = Pose([1.0, 0.0, 0.0], [0.0, 0.0, math.pi / 2])
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
    "bowl"
)

# load chair
chair_id = demo.load_object(
    "./URDF_models/furniture_chair/model.urdf",
    [-0.3, 0.8, 0.1],
    [math.pi / 2.0 * 3, 0.0, math.pi / 2.0],
    1.5, 
    "chair"
)

# print("obstacles in the environment: {}".format(demo.obstacle_navigation_ids))
# demo.draw_aabb(table_id)
# demo.wait(10)

# get bounding box of objects
aabb_table = demo.get_bounding_box(table_id)
# print('-' * 20 + '\n' + 'aabb_table:{}'.format(aabb_table))

# get info about base
demo.get_base_joint_info()

target_position = [2.0, 1.0, 0]
# plot line connecting init and goal positions
demo.draw_line([1.0, 0.0, 0.0], target_position)

# navigate segbot
demo.navigate_base(Pose(target_position, [0.0, 0.0, math.pi/2.0]))

# check result
demo.get_base_joint_info()
