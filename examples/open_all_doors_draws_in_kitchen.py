"""
@Description :   This script opens all doors and drawers in the kitchen
@Author      :   Yan Ding 
@Time        :   2023/09/01 08:12:35
"""

import math
import sys
import os

"""
Get the utils module path
"""
# customized package
current_path = os.path.abspath(__file__)
utils_path = os.path.dirname(os.path.dirname(current_path)) + "/utils"
if os.path.basename(utils_path) != "utils":
    raise ValueError('Not add the path of folder "utils", please check again!')
sys.path.append(utils_path)
from utils_Bestman import Bestman, Pose
from utils_PbClient import PbClient
from utils_PbVisualizer import PbVisualizer
from utils_PbOMPL import PbOMPL

# load kitchen from three scenarios
index = 0
if index == 0:
    from utils_Kitchen_v0 import Kitchen
elif index == 1:
    from utils_Kitchen_v1 import Kitchen
else:
    assert False, "index should be 0 or 1"


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

# open a drawer in element A
for i in range(10):
    drawer_id = i+1
    kitchen.open_it('elementA', drawer_id)

# close a drawer in element A
for i in range(10):
    drawer_id = i+1
    kitchen.close_it('elementA', drawer_id)

# open a drawer in element C
for i in range(3):
    drawer_id = i+1
    kitchen.open_it('elementC', drawer_id)

# close a drawer in element C
for i in [2, 1, 0]:
    drawer_id = i+1
    kitchen.close_it('elementC', drawer_id)

# open a drawer in element D
for i in range(1):
    drawer_id = i+1
    kitchen.open_it('elementD', drawer_id)

# close a drawer in element D
for i in range(1):
    drawer_id = i+1
    kitchen.close_it('elementD', drawer_id)

# open a drawer in element E
for i in range(2):
    drawer_id = i+1
    kitchen.open_it('elementE', drawer_id)

# close a drawer in element E
for i in range(2):
    drawer_id = i+1
    kitchen.close_it('elementE', drawer_id)


# disconnect pybullet
pb_client.wait(5)
pb_client.disconnect_pybullet()