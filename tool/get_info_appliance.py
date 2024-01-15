"""
@Description :   
@Author      :   Yan Ding 
@Time        :   2023/09/01 15:26:35
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
index = 2
if index == 0:
    from utils_Kitchen_v0 import Kitchen
elif index == 1:
    from utils_Kitchen_v1 import Kitchen
elif index == 2:
    from utils_Kitchen_v2 import Kitchen
else:
    assert False, "index should be 0, 1, and 2"


pb_client = PbClient(enable_GUI=True, enable_Debug=True)
pb_client.enable_vertical_view(4.0, [1.0, 1.0, 0])
pb_visualizer = PbVisualizer(pb_client)
# logID = pb_client.start_record("example_manipulation") # start recording
init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose, pb_client)  # load robot
demo.get_joint_link_info("arm")  # get info about arm
init_joint = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(init_joint)  # reset arm joint position

# load fridge
# pb_client.load_object(
#     model_path="./Kitchen_models/models/Fridge/10144/mobility.urdf",
#     object_position=[0.5, 4.9, 1.055],
#     object_orientation=[0, 0, math.pi],
#     scale=1.0,
#     obj_name='fridge',
#     fixed_base=True,
# )

# pb_client.get_appliance_joint_info(pb_client.fridge_id)
# pb_client.change_appliance_joint(pb_client.fridge_id, 1, math.pi / 2.0)
# pb_client.run(100)

# load microwave
# pb_client.load_object(
#     model_path="./Kitchen_models/models/Microwave/7128/mobility.urdf",
#     object_position=[0.4, 6.4, 1.019],
#     object_orientation=[0, 0, math.pi],
#     scale=1.0,
#     obj_name='microwave',
#     fixed_base=True,
# )

# pb_client.get_appliance_joint_info(pb_client.microwave_id)
# pb_client.change_appliance_joint(pb_client.microwave_id, 1, math.pi / 2.0)
# pb_client.run(100)

# load dishwasher
pb_client.load_object(
    model_path="./Kitchen_models/models/Dishwasher/2085/mobility.urdf",
    object_position=[0.7, 6.4, 1.366],
    object_orientation=[0, 0, math.pi],
    scale=1.0,
    obj_name="dishwasher",
    fixed_base=True,
)

pb_client.get_appliance_joint_info(pb_client.dishwasher_id)
pb_client.change_appliance_joint(pb_client.dishwasher_id, 1, math.pi)
pb_client.run(1000)


# disconnect pybullet
pb_client.wait(10)
pb_client.disconnect_pybullet()
