from utils.utils_Bestman import Bestman, Pose
from utils.utils_PbClient import PbClient
from utils.utils_PbVisualizer import PbVisualizer
from utils.utils_Kitchen_object import Kitchen
from utils.utils_PbOMPL import PbOMPL
import math
import pybullet as p

# This script demonstrates retrieving an object from inside a drawer using a vacuum tool.

# load cleint
pb_client = PbClient(enable_GUI=True)
pb_client.enable_vertical_view(1.0, [1.7, 3.68, 1.95], -86.4, -52.3)

# load visualizer
pb_visualizer = PbVisualizer(pb_client)

# load kitchen
kitchen_id = Kitchen(pb_client)

# load robot
init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose, pb_client)

# reset arm joint position
pose1 = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(pose1)

# get arm information
print("-" * 20 + "\n" + "joint_indexs: {}; end_effector_index: {}".format(demo.joint_indexs, demo.end_effector_index))

# load OMPL planner
threshold_distance = 0.1
ompl = PbOMPL(
    robot_id=demo.arm_id,
    joint_idx=demo.joint_indexs,
    obstacles=[],
    planner="RRTConnect",
    threshold=threshold_distance,
)

# add obstacles
ompl.add_scene_obstacles(display=True)
ompl.check_obstacles()

# open the drawer
drawer_id = 5
kitchen_id.open_drawer("elementA", drawer_id)

# load bowl
bowl_position = [3.6, 2.4, 0.9]  # hard code
bowl_id = pb_client.load_object("./URDF_models/utensil_bowl_blue/model.urdf", bowl_position, [0.0, 0.0, 0.0], 1.0, "bowl")
pb_client.run(100) # wait for a few seconds
_, _, _, _, _, max_z = pb_client.get_bounding_box(bowl_id)
bowl_position[2] = max_z
print("-" * 20 + "\n" + 'bowl position is {}'.format(bowl_position))

# navigate to standing position
standing_position = [3.1, 2.4, 0]  # hard code
demo.navigate_base(Pose(standing_position, [0.0, 0.0, 0.0]))

# get end effector information
ee_position, ee_orientation = demo.get_end_effector_info()
start = demo.cartesian_to_joints(position=ee_position, orientation=p.getEulerFromQuaternion(ee_orientation))

# set target object for grasping
ompl.set_target(bowl_id)
target_orientation = [0.0, math.pi / 2.0, 0.0] # vertical
goal = demo.cartesian_to_joints(position=bowl_position, orientation=target_orientation)
print("-" * 20 + "\n" + "intial configuration:{}".format(start) + "goal configuration:{}".format(goal))

sys.exit()

# start grasping
ompl.grasp_object(start=start, goal=goal, end_effector_link_index=demo.end_effector_index)

# wait a few seconds
pb_client.wait(20)

# disconnect pybullet
pb_client.disconnect_pybullet()