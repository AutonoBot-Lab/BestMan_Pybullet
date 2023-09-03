"""
@Description :   This script answers a question that could the robot detect the opened door in the navigation.
@Author      :   Yan Ding 
@Time        :   2023/08/31 03:01:50
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
index = 0
if index == 0:
    from utils_Kitchen_v0 import Kitchen
elif index == 1:
    from utils_Kitchen_v1 import Kitchen
else:
    assert False, "index should be 0 or 1"

pb_client = PbClient(enable_GUI=True)
pb_client.enable_vertical_view(1.0, [1.7, 3.68, 1.95], -86.4, -52.3)
pb_visualizer = PbVisualizer(pb_client)
# logID = pb_client.start_record("example_navigation_with_opened_fridge") # start recording
init_pose = Pose([3.29, 3.86, 0], [0.0, 0.0, 0.0])
demo = Bestman(init_pose, pb_client)  # load robot
demo.get_joint_link_info("arm")  # get info about arm
init_joint = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(init_joint)  # reset arm joint position

# load kitchen
kitchen = Kitchen(pb_client)
print("object ids in loaded kitchen:\n{}".format(kitchen.object_ids))

# load OMPL planner
threshold_distance = 0.1
ompl = PbOMPL(
    pb_client=pb_client,
    arm_id=demo.arm_id,
    joint_idx=demo.arm_joint_indexs,
    tcp_link=demo.tcp_link,
    obstacles=[],
    planner="RRTConnect",
    threshold=threshold_distance,
)

# add obstacles
ompl.add_scene_obstacles(display=True)
ompl.check_obstacles()

# load bowl
bowl_position = [4.15, 4.3, 1.0]  # TODO: object goes flying
bowl_id = pb_client.load_object("./URDF_models/utensil_bowl_blue/model.urdf", bowl_position, [0.0, 0.0, 0.0], 1.0, "bowl", fixed_base=False)
pb_client.run(1000)
_, _, min_z, _, _, max_z = pb_client.get_bounding_box(bowl_id)
bowl_position[2] = max_z + demo.tcp_height * 2 # consider tcp's height
print("bowl position:{}".format(bowl_position))

# set target object for grasping
ompl.set_target(bowl_id)
target_orientation = [0.0, math.pi / 2.0, 0.0] # vertical
goal = demo.cartesian_to_joints(position=bowl_position, orientation=target_orientation)
print("-" * 20 + "\n" + "Goal configuration:{}".format(goal))

# reach target object
pb_visualizer.change_arm_color(demo.arm_id, light_color=True)
result, trajectory = ompl.reach_object(start=demo.get_arm_joint_angle(), goal=goal, end_effector_link_index=demo.end_effector_index)
pb_visualizer.change_arm_color(demo.arm_id, light_color=False)
# print('result:{}, trajectory:{}'.format(result, trajectory))
pb_client.wait(5)

# perform action
demo.excite_trajectory(trajectory)

# grasp object
demo.active_gripper(bowl_id, 1)

# disconnect pybullet
pb_client.wait(5)
pb_client.disconnect_pybullet()