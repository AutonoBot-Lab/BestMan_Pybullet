from utils_control import Bestman, Pose
from utils_envs import Kitchen
import math

# load robot
init_pose = Pose([1.0, 0.0, 0.0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose)
demo.enable_vertical_view(4.0, [1.0, 1.0, 0])

# load table
table_id = demo.load_object(
    "./URDF_models/furniture_table_rectangle_high/table.urdf",
    [1.0, 1.0, 0.0],
    [0.0, 0.0, 0.0],
    1.0,
    "table",
)

# get bounding box of objects
demo.get_bounding_box(table_id)

# get info about base
demo.get_base_joint_info()

# plot line connecting init and goal positions
demo.draw_line([1.0, 0.0, 0.0], [4.5, 3.2, 0])

# navigate segbot
demo.navigate_base(Pose([4.5, 3.2, 0], [0.0, 0.0, math.pi/2.0]))

# check result
demo.get_base_joint_info()
