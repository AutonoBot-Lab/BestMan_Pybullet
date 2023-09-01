"""
@Description :   This script shows how to grasp a bowl from a fridge
@Author      :   Yan Ding 
@Time        :   2023/09/01 07:47:46
"""

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
from utils_PbOMPL import PbOMPL

# load kitchen from three scenarios
index = 0
if index == 0:
    from utils_Kitchen_object import Kitchen
elif index == 1:
    from utils_Kitchen_object2 import Kitchen
elif index == 2:
    from utils_Kitchen_scene import Kitchen
else:
    assert False, "index should be 0, 1 or 2"

pb_client = PbClient(enable_GUI=True)
pb_client.enable_vertical_view(1.0, [1.7, 3.68, 1.95], -86.4, -52.3)
pb_visualizer = PbVisualizer(pb_client)
# logID = pb_client.start_record("example_manipulation") # start recording
init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose, pb_client)  # load robot
demo.get_joint_link_info("arm")  # get info about arm
init_joint = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(init_joint)  # reset arm joint position

# load kitchen
kitchen = Kitchen(pb_client)
print("object ids in loaded kitchen:\n{}".format(kitchen.object_ids))

# open fridge
kitchen.open_it("elementE", 1)

# load bowl
bowl_position = [3.89, 6.52, 1.58]  # TODO: object goes flying
bowl_id = pb_client.load_object("./URDF_models/utensil_bowl_blue/model.urdf", bowl_position, [0.0, 0.0, 0.0], 1.0, "bowl")
pb_client.run(100)
_, _, min_z, _, _, max_z = pb_client.get_bounding_box(bowl_id)
bowl_position[2] = max_z + demo.tcp_height # consider tcp's height

# disconnect pybullet
pb_client.wait(10)
pb_client.disconnect_pybullet()