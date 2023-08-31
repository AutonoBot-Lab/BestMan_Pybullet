import math
import sys
import os
import pybullet as p

"""
Get the utils module path
"""
# customized package
current_path = os.path.abspath(__file__)
utils_path = os.path.dirname(os.path.dirname(current_path)) + '/utils'
if os.path.basename(utils_path) != 'utils':
    raise ValueError('Not add the path of folder "utils", please check again!')
sys.path.append(utils_path)
from utils_Bestman import Bestman, Pose
from utils_PbClient import PbClient
from utils_PbVisualizer import PbVisualizer
from utils_Kitchen_object import Kitchen

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

# # start recording
# logID = pb_client.start_record("example_manipuation_with_gripper")

# reset arm joint position
pose1 = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(pose1)

# get arm information
print("-" * 20 + "\n" + "joint_indexs: {}; end_effector_index: {}".format(demo.joint_indexs, demo.end_effector_index))

# open fridge
container_id = 1
kitchen_id.open_it("elementD", container_id)
pb_client.run(100) # wait for a few seconds

# open fridge
container_id = 1
kitchen_id.open_it("elementE", container_id)
pb_client.run(100) # wait for a few seconds

# load bowl
# bowl_position = [3.89, 6.52, 1.58]  # hard code
bowl_position = [3.89, 2.95, 1.05]  # hard code
bowl_id = pb_client.load_object("./URDF_models/utensil_bowl_blue/model.urdf", bowl_position, [0.0, 0.0, 0.0], 1.0, "bowl")
pb_client.run(1000) # wait for a few seconds

_, _, min_z, _, _, max_z = pb_client.get_bounding_box(bowl_id)
bowl_position[2] = max_z + demo.tcp_height # consider the height of the
print("-" * 20 + "\n" + 'min_z: {} max_z: {}'.format(min_z, max_z))


pb_client.wait(10)
pb_client.disconnect_pybullet()