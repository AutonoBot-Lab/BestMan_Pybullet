"""
@Description :   # Find a target pose by manually adjusting the position of an object
@Author      :   Yan Ding 
@Time        :   2023/08/31 16:02:08
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
index = 1
if index == 0:
    from utils_Kitchen_v0 import Kitchen
elif index == 1:
    from utils_Kitchen_v1 import Kitchen
elif index == 2:
    from utils_Kitchen_v2 import Kitchen
else:
    assert False, "index should be 0, 1, and 2"

pb_client = PbClient(enable_GUI=True, enable_Debug=True)
pb_client.enable_vertical_view(2.2, [1.9, 7.35, 1.54], yaw=88.8, pitch=-31.5)
pb_visualizer = PbVisualizer(pb_client)
# logID = pb_client.start_record("example_manipulation") # start recording
init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose, pb_client)  # load robot
demo.get_joint_link_info("arm")  # get info about arm
init_joint = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(init_joint)  # reset arm joint position

# load kitchen
kitchen = Kitchen(pb_client)

# open the frige's door
kitchen.open_it(pb_client.fridge_id, 1, open_angle=math.pi / 2)

# load bowl
bowl_position = [0, 0, 0]
bowl_id = pb_client.load_object(
    "./URDF_models/utensil_bowl_blue/model.urdf",
    bowl_position,
    [0.0, 0.0, 0.0],
    1.0,
    "bowl",
)
pb_client.run_slider_and_update_position(100000, "Position", -10, 10, 1, bowl_id)

pb_client.wait(5)
pb_client.disconnect_pybullet()
