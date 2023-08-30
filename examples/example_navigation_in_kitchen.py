from utils.utils_Bestman import Bestman, Pose
from utils.utils_PbClient import PbClient
from utils.utils_PbVisualizer import PbVisualizer
# from utils.utils_Kitchen_object import Kitchen
from utils.utils_Kitchen_scene import Kitchen
import math
import pybullet as p

"""
This code is to test how to navigate when a fridge's door is open.
"""

# load cleint
pb_client = PbClient(enable_GUI=True, enable_Debug=True)
# pb_client.enable_vertical_view(1.0, [1.7, 3.68, 1.95], -86.4, -52.3)

# load visualizer
pb_visualizer = PbVisualizer(pb_client)

# load kitchen
kitchen = Kitchen(pb_client, './Kitchen/scenes/kitchen_basics.lisdf')
# kitchen = Kitchen(pb_client, './Kitchen/scenes/kitchen_counter.lisdf')
# kitchen = Kitchen(pb_client, './Kitchen/scenes/kitchen_lunch.lisdf')
pb_client.run(100)

# load robot
init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose, pb_client)

# reset arm joint position
pose1 = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(pose1)

# get arm information
print("-" * 20 + "\n" + "joint_indexs: {}; end_effector_index: {}".format(demo.joint_indexs, demo.end_effector_index))

# open fridge's door
# container_id = 1
# kitchen_id.open_drawer("elementE", container_id)
# pb_client.run(100) # wait for a few seconds

# consider how to navigte

# step1: test bounding box
