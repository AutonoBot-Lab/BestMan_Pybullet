from utils_Bestman import Bestman, Pose
from utils_PbClient import PbClient
from utils_PbVisualizer import PbVisualizer
from utils_Kitchen import Kitchen
import math
import time

# load cleint
pb_client = PbClient(enable_GUI=True)
pb_client.enable_vertical_view(4.0, [1.0, 1.0, 0])

# load visualizer
pb_visualizer = PbVisualizer(pb_client)

# load table
table_id = pb_client.load_object(
    "./URDF_models/furniture_table_rectangle_high/table.urdf",
    [1.0, 1.0, 0.0],
    [0.0, 0.0, 0.0],
    1.0,
    "table",
)

# load robot
init_pose = Pose([1.0, -1.0, 0.0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose, pb_client)

# reset arm joint position
pose1 = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(pose1)

object_ids = demo.set_envs()
table_id = object_ids["table"]
bowl_id = object_ids["bowl"]
chair_ids = object_ids["chairs"]

pb_client.run(240)

# # navigate and manipulate robot
demo.forward()
demo.forward()
demo.forward()
demo.forward()
demo.grasp(bowl_id)

pb_client.wait(20)
pb_client.disconnect_pybullet()
