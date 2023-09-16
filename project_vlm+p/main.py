"""
@Description :   An example of VLM+P
@Author      :   Yan Ding 
@Time        :   2023/09/03 21:54:15
"""

import math
import sys
import os
import pybullet as p
import matplotlib.pyplot as plt

"""
Get the utils module path
"""
# customized package
current_path = os.path.abspath(__file__)
utils_path = os.path.dirname(os.path.dirname(current_path)) + '/utils'
if os.path.basename(utils_path) != 'utils':
    raise ValueError('Not add the path of folder "utils", please check again!')
sys.path.append(utils_path)
tool_path = os.path.dirname(os.path.dirname(current_path)) + '/tool'
if os.path.basename(tool_path) != 'tool':
    raise ValueError('Not add the path of folder "tool", please check again!')
sys.path.append(tool_path)
from utils_Bestman import Bestman, Pose
from utils_PbClient import PbClient
from utils_PbVisualizer import PbVisualizer
from utils_PbOMPL import PbOMPL
from utils_camera import CameraInfo

# ----------------------------------------------------------------
# step 0: prepare environment # DONE
# ----------------------------------------------------------------

# load kitchen from three scenarios
index = 0
if index == 0:
    from utils_Kitchen_v0 import Kitchen
elif index == 1:
    from utils_Kitchen_v1 import Kitchen
else:
    assert False, "index should be 0 or 1"

pb_client = PbClient(enable_GUI=True, enable_capture=True)
pb_client.enable_vertical_view(1.0, [1.7, 3.68, 1.95], -86.4, -52.3)
pb_visualizer = PbVisualizer(pb_client)
# logID = pb_client.start_record("example_manipulation") # start recording
# init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
init_pose = Pose([1.5, 0.5, 0], [0.0, 0.0, 0.0])
demo = Bestman(init_pose, pb_client)  # load robot
demo.get_joint_link_info("arm")  # get info about arm
init_joint = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(init_joint)  # reset arm joint position

# load kitchen
kitchen = Kitchen(pb_client)
print("object ids in loaded kitchen:\n{}".format(kitchen.object_ids))
print('-'*20 + '\n' + 'Preparation has been completed!')

# ----------------------------------------------------------------
# step 1: create a scenario  # DONE
# ----------------------------------------------------------------
banana_id = pb_client.load_object(
    "./URDF_models/plastic_banana/model.urdf",
    [3.8, 4.2, 1.1],
    [0.0, 0.0, math.pi/2],
    1.0,
    "banana",
)
cup_id = pb_client.load_object(
    "./URDF_models/orange_cup/model.urdf",
    [3.8, 2.3, 1.1],
    [0.0, 0.0, 0.0],
    1.0,
    "cup",
)
apple_id = pb_client.load_object(
    "./URDF_models/plastic_apple/model.urdf",
    [3.8, 0.7, 1.1],
    [0.0, 0.0, 0.0],
    1.0,
    "apple",
)
pb_client.run(100) # let objects fall down
print('-'*20 + '\n' + 'A scenario has been created!')

# ----------------------------------------------------------------
# step 2: create a service request # DONE
# ----------------------------------------------------------------
service_request = "put all fruit into the fridge"
print('-'*20 + '\n' + 'A service request have been obtained!')

# ----------------------------------------------------------------
# step 3: take screenshots # DONE
# ----------------------------------------------------------------
camera = CameraInfo(pb_client)
rgb, depth, seg = camera.get_image_from_camera(demo.base_id, "view_1", enable_save=True)
# plt.imshow(rgb)
# plt.show()

temp_pose = init_pose

temp_pose_position = temp_pose.position
temp_pose_orientation = temp_pose.orientation
new_position = [temp_pose_position[0], temp_pose_position[1] + 1, temp_pose_position[2]]
next_waypoint = Pose(new_position, temp_pose_orientation)
demo.move_base_to_next_waypoint(next_waypoint)
demo.rotate_base(0)
rgb, depth, seg = camera.get_image_from_camera(demo.base_id, "view_2", enable_save=True)
# plt.imshow(rgb)
# plt.show()

temp_pose_position = next_waypoint.position
temp_pose_orientation = next_waypoint.orientation
new_position = [temp_pose_position[0], temp_pose_position[1] + 1, temp_pose_position[2]]
next_waypoint = Pose(new_position, temp_pose_orientation)
demo.move_base_to_next_waypoint(next_waypoint)
demo.rotate_base(0)
rgb, depth, seg = camera.get_image_from_camera(demo.base_id, "view_3", enable_save=True)
# plt.imshow(rgb)
# plt.show()

temp_pose_position = next_waypoint.position
temp_pose_orientation = next_waypoint.orientation
new_position = [temp_pose_position[0], temp_pose_position[1] + 1, temp_pose_position[2]]
next_waypoint = Pose(new_position, temp_pose_orientation)
demo.move_base_to_next_waypoint(next_waypoint)
demo.rotate_base(0)
rgb, depth, seg = camera.get_image_from_camera(demo.base_id, "view_4", enable_save=True)
# plt.imshow(rgb)
# plt.show()

temp_pose_position = next_waypoint.position
temp_pose_orientation = next_waypoint.orientation
new_position = [temp_pose_position[0], temp_pose_position[1] + 1, temp_pose_position[2]]
next_waypoint = Pose(new_position, temp_pose_orientation)
demo.move_base_to_next_waypoint(next_waypoint)
demo.rotate_base(0)
rgb, depth, seg = camera.get_image_from_camera(demo.base_id, "view_5", enable_save=True)
# plt.imshow(rgb)
# plt.show()

temp_pose_position = next_waypoint.position
temp_pose_orientation = next_waypoint.orientation
new_position = [temp_pose_position[0], temp_pose_position[1] + 1, temp_pose_position[2]]
next_waypoint = Pose(new_position, temp_pose_orientation)
demo.move_base_to_next_waypoint(next_waypoint)
demo.rotate_base(0)
rgb, depth, seg = camera.get_image_from_camera(demo.base_id, "view_6", enable_save=True)
# plt.imshow(rgb)
# plt.show()

print('-'*20 + '\n' + 'Images have been captured!')

# ----------------------------------------------------------------
# step 4: query ViLD and QianWen and get intial state
# ----------------------------------------------------------------

"""
In main_step4.ipynb, how to use VLM: QianWen & how to use LLM: OpenAI
"""

# ----------------------------------------------------------------
# step 5: create pddl problem
# ----------------------------------------------------------------

# ----------------------------------------------------------------
# step 6: generate task plan
# ----------------------------------------------------------------

# ----------------------------------------------------------------
# step 7: execute task plan
# ----------------------------------------------------------------

# disconnect from server
pb_client.wait(5)
pb_client.disconnect_pybullet()