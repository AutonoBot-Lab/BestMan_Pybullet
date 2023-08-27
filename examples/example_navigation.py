from utils.utils_Bestman import Bestman, Pose
from utils.utils_PbClient import PbClient
from utils.utils_PbVisualizer import PbVisualizer
from utils.utils_Kitchen_object import Kitchen
import math

# load cleint
pb_client = PbClient(enable_GUI=True)
pb_client.enable_vertical_view(4.0, [1.0, 1.0, 0])

# # load visualizer
# pb_visualizer = PbVisualizer(pb_client)

# start recording
logID = pb_client.start_record("example_navigation")

# load table, bowl, and chair
table_id = pb_client.load_object(
    "./URDF_models/furniture_table_rectangle_high/table.urdf",
    [1.0, 1.0, 0.0],
    [0.0, 0.0, 0.0],
    1.0,
    "table",
)

bowl_id = pb_client.load_object(
    "./URDF_models/utensil_bowl_blue/model.urdf", 
    [0.6, 0.6, 0.85], 
    [0.0, 0.0, 0.0], 
    1.0,
    "bowl"
)

chair_id = pb_client.load_object(
    "./URDF_models/furniture_chair/model.urdf",
    [-0.3, 0.8, 0.1],
    [math.pi / 2.0 * 3, 0.0, math.pi / 2.0],
    1.5, 
    "chair"
)

# obstacles in the navigation
print("obstacles in the environment: {}".format(pb_client.obstacle_navigation_ids))

# get bounding box of objects
aabb_table = pb_client.get_bounding_box(table_id)
print('-' * 20 + '\n' + 'aabb_table:{}'.format(aabb_table))
pb_visualizer.draw_aabb(table_id)

# load robot
init_pose = Pose([1.0, 0.0, 0.0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose, pb_client)

# reset arm joint position
pose1 = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(pose1)

# get info about base
demo.get_base_joint_info()

# plot line connecting init and goal positions
target_position = [2.0, 1.0, 0]
pb_visualizer.draw_line([1.0, 0.0, 0.0], target_position)

# navigate segbot
demo.navigate_base(Pose(target_position, [0.0, 0.0, math.pi/2.0]))

# check result
demo.get_base_joint_info()

# # end recording
# pb_client.end_record(logID)

pb_client.wait(20)

pb_client.disconnect_pybullet()