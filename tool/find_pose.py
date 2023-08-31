from utils.utils_Bestman import Bestman, Pose
from utils.utils_PbClient import PbClient
from utils.utils_PbVisualizer import PbVisualizer
from utils.utils_Kitchen_object import Kitchen
from utils.utils_PbOMPL import PbOMPL
import math
import pybullet as p

# This script demonstrates retrieving an object from inside a drawer using a vacuum tool.

# load cleint
pb_client = PbClient(enable_GUI=True, enable_Debug=True)
pb_client.enable_vertical_view(1.0, [1.7, 3.68, 1.95], -86.4, -52.3)

# load visualizer
pb_visualizer = PbVisualizer(pb_client)

# load kitchen
kitchen_id = Kitchen(pb_client)

# load robot
init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose, pb_client)

# # start recording
# logID = pb_client.start_record("example_manipuation_with_gripper")

# reset arm joint position
pose1 = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(pose1)

# get arm information
print("-" * 20 + "\n" + "joint_indexs: {}; end_effector_index: {}".format(demo.joint_indexs, demo.end_effector_index))

# load OMPL planner
threshold_distance = 0.05
ompl = PbOMPL(
    pb_client=pb_client,
    arm_id=demo.arm_id,
    joint_idx=demo.joint_indexs,
    tcp_link=demo.tcp_link,
    obstacles=[],
    planner="RRTConnect",
    threshold=threshold_distance,
)

# add obstacles
ompl.add_scene_obstacles(display=True)
ompl.check_obstacles()

# open the container
container_id = 1
kitchen_id.open_drawer("elementD", container_id)

# load bowl
bowl_position = [3.6, 2.4, 0.9]  # hard code
bowl_id = pb_client.load_object("./URDF_models/utensil_bowl_blue/model.urdf", bowl_position, [0.0, 0.0, 0.0], 1.0, "bowl")
pb_client.run(100) # wait for a few seconds
_, _, min_z, _, _, max_z = pb_client.get_bounding_box(bowl_id)
bowl_position[2] = max_z + demo.tcp_height # consider the height of the
print("-" * 20 + "\n" + 'min_z: {} max_z: {}'.format(min_z, max_z))

pb_client.run_slider_and_update_position(100000, "Position", -10, 10, 1, bowl_id)

pb_client.run(100)
pb_client.disconnect_pybullet()