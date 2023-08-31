"""
@Description :   load a kitchen scenario
@Author      :   Yan Ding 
@Time        :   2023/08/30 20:43:44
"""

"""
Get the utils module path
"""
import sys
import os

current_path = os.path.abspath(__file__)
parent_path = os.path.dirname(current_path)
sys.path.append(parent_path)

from utils.utils_Bestman import Bestman, Pose
from utils.utils_PbClient import PbClient
from utils.utils_PbVisualizer import PbVisualizer
from utils.utils_Kitchen_scene import Kitchen
import math

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