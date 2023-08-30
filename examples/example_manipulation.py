from utils.utils_Bestman import Bestman, Pose
from utils.utils_PbClient import PbClient
from utils.utils_PbVisualizer import PbVisualizer
from utils.utils_Kitchen_object import Kitchen
import math

# load cleint
pb_client = PbClient(enable_GUI=True)
pb_client.enable_vertical_view(4.0, [1.0, 1.0, 0])

# load visualizer
pb_visualizer = PbVisualizer(pb_client)

# start recording
# logID = pb_client.start_record("example_manipulation")

# load table and bowl
table_id = pb_client.load_object(
    "./URDF_models/furniture_table_rectangle_high/table.urdf",
    [1.0, 1.0, 0.0],
    [0.0, 0.0, 0.0],
    1.0, 
    "table",
)

bowl_id = pb_client.load_object(
    "./URDF_models/utensil_bowl_blue/model.urdf", [0.6, 0.6, 0.85], [0.0, 0.0, 0.0], 1.0, "bowl"
)

# get bounding box of objects
pb_client.get_bounding_box(table_id)
pb_client.get_bounding_box(bowl_id)

# load robot
init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose, pb_client)

# get info about arm
demo.get_base_joint_info()
demo.get_arm_joint_info()
demo.get_link_names()

# reset arm joint position
pose1 = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(pose1)

# manipulate ur5e - option 1
# demo.move_end_effector_to_goal_position(Pose([0.9, 0.7, 0.85], [0.0, math.pi, 0.0]))

# manipulate ur5e - option 2
# demo.grasp(bowl_id)  # vcertical downward grip

# # manipulate ur5e - option 3
demo.pick_place(bowl_id, [0.9, 0.7, 0.85], [0.0, math.pi / 2.0, 0.0]) # ur5e_vacuum.urdf  # vcertical downward grip
# demo.pick_place(bowl_id, [0.9, 0.7, 0.85], [0.0, math.pi / 2.0, 0.0]) # ur5e.urdf  # vcertical downward grip

# end recording
# pb_client.end_record(logID)

pb_client.wait(20)

pb_client.disconnect_pybullet()