from utils_control import Bestman, Pose
from utils_envs import Kitchen
import math
import time

# load robot
init_pose = Pose([1.0, -1.0, 0.0], [0.0, 0.0, math.pi / 2])
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

# # load bowl
# bowl_id = demo.load_object(
#     "./URDF_models/utensil_bowl_red/model.urdf", [0.6, 0.6, 0.85], [0.0, 0.0, 0.0], 1.0, "bowl"
# )

object_ids = demo.set_envs()
print(demo.obstacle_navigation_ids)

# # navigate and manipulate robot
demo.forward()
demo.forward()
demo.forward()
demo.forward()
demo.grasp(bowl_id)
