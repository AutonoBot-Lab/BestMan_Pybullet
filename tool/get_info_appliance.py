from utils.utils_Bestman import Bestman, Pose
from utils.utils_PbClient import PbClient
from utils.utils_PbVisualizer import PbVisualizer
from utils.utils_Kitchen_object import Kitchen
import math

# load cleint
pb_client = PbClient(enable_GUI=True)
pb_client.enable_vertical_view(1.0, [1.7, 3.68, 1.95], -86.4, -52.3)

# load visualizer
pb_visualizer = PbVisualizer(pb_client)

# load robot
init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose, pb_client)

# reset arm joint position
pose1 = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(pose1)

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
    object_position=[0.7, 6.4, 0.366],
    object_orientation=[0, 0, math.pi],
    scale=1.0,
    obj_name='dishwasher',
    fixed_base=True,
)

pb_client.get_appliance_joint_info(pb_client.dishwasher_id)
pb_client.change_appliance_joint(pb_client.dishwasher_id, 1, math.pi)
pb_client.run(1000)

# wait a few seconds
pb_client.wait(10)

# disconnect pybullet
pb_client.disconnect_pybullet()