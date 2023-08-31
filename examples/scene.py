"""
@Description :   load a kitchen scenario
@Author      :   Yan Ding 
@Time        :   2023/08/30 20:43:44
"""

import math
import sys
import os

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

# load client
pb_client = PbClient(enable_GUI=True, enable_Debug=True)
pb_client.enable_vertical_view(3.6, [0.02, 6.52, 1.02], pitch=-17.11, yaw=88.79)

# load visualizer
pb_visualizer = PbVisualizer(pb_client)

# load robot
init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose, pb_client)

# reset arm joint position
pose1 = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(pose1)

# load kitchen
kitchen = Kitchen(pb_client, lisdf_id=0)

# wait a few seconds
pb_client.run(1000)
pb_client.wait(100)

# disconnect pybullet
pb_client.disconnect_pybullet()