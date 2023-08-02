from utils_control import Bestman, Pose
from utils_envs import Kitchen
import math

# load robot
init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose)
demo.enable_vertical_view(4, [1, 1, 0])

# reset arm joint position
pose1 = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(pose1)

# load kitchen
kitchen_id = Kitchen()

# open drawer
for i in range(10):
    drawer_id = i+1
    kitchen_id.open_drawer(drawer_id)
    demo.wait(0.1)

#close all drawers
for i in range(10):
    drawer_id = i+1
    kitchen_id.close_drawer(drawer_id)
    demo.wait(0.1)