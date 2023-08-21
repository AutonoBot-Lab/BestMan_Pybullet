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
kitchen = Kitchen()

# open a drawer in element A
for i in range(10):
    drawer_id = i+1
    kitchen.open_drawer('elementA', drawer_id)

# close a drawer in element A
for i in range(10):
    drawer_id = i+1
    kitchen.close_drawer('elementA', drawer_id)

# open a drawer in element C
for i in range(3):
    drawer_id = i+1
    kitchen.open_drawer('elementC', drawer_id)

# close a drawer in element C
for i in range(3):
    drawer_id = i+1
    kitchen.close_drawer('elementC', drawer_id)

# open a drawer in element D
for i in range(1):
    drawer_id = i+1
    kitchen.open_drawer('elementD', drawer_id)

# close a drawer in element D
for i in range(1):
    drawer_id = i+1
    kitchen.close_drawer('elementD', drawer_id)

# open a drawer in element D
for i in range(2):
    drawer_id = i+1
    kitchen.open_drawer('elementE', drawer_id)

# close a drawer in element D
for i in range(2):
    drawer_id = i+1
    kitchen.close_drawer('elementE', drawer_id)

demo.wait(10)