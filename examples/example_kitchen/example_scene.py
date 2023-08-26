from utils.utils_Bestman import Bestman, Pose
from utils.utils_PbClient import PbClient
from utils.utils_PbVisualizer import PbVisualizer
from utils.utils_Kitchen_scene import Kitchen
import math
import tkinter as tk
import pybullet as p

# load cleint
pb_client = PbClient(enable_GUI=True, enable_Debug=True)
pb_client.enable_vertical_view(1.0, [1.7, 3.68, 1.95], -86.4, -52.3)

# load visualizer
pb_visualizer = PbVisualizer(pb_client)

# load robot
init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose, pb_client)

# reset arm joint position
pose1 = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(pose1)

# load kitchen
kitchen = Kitchen(pb_client, './Kitchen/scenes/kitchen_basics.lisdf')

# wait a few seconds
pb_client.wait(100)

# disconnect pybullet
pb_client.disconnect_pybullet()