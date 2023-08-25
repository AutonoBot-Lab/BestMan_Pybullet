from utils.utils_Bestman import Bestman, Pose
from utils.utils_PbClient import PbClient
from utils.utils_PbVisualizer import PbVisualizer
from utils.utils_Kitchen import Kitchen
import math

# load cleint
pb_client = PbClient(enable_GUI=True)
pb_client.enable_vertical_view(4.0, [1.0, 1.0, 0])

# load visualizer
pb_visualizer = PbVisualizer(pb_client)

# load robot
init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose, pb_client)

# reset arm joint position
pose1 = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(pose1)

# load kitchen
kitchen = Kitchen(pb_client)

# start recording
logID = pb_client.start_record("test")

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

# open a drawer in element E
for i in range(2):
    drawer_id = i+1
    kitchen.open_drawer('elementE', drawer_id)

# close a drawer in element E
for i in range(2):
    drawer_id = i+1
    kitchen.close_drawer('elementE', drawer_id)

# end recording
pb_client.end_record(logID)

# disconnect pybullet
pb_client.disconnect_pybullet()