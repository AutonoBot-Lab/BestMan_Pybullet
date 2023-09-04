"""
@Description :   An example of VLM+P
@Author      :   Yan Ding 
@Time        :   2023/09/03 21:54:15
"""

import math
import sys
import os
import pybullet as p

"""
Get the utils module path
"""
# customized package
current_path = os.path.abspath(__file__)
utils_path = os.path.dirname(os.path.dirname(current_path)) + '/utils'
if os.path.basename(utils_path) != 'utils':
    raise ValueError('Not add the path of folder "utils", please check again!')
sys.path.append(utils_path)
from utils_Bestman import Bestman, Pose
from utils_PbClient import PbClient
from utils_PbVisualizer import PbVisualizer
from utils_PbOMPL import PbOMPL

# ----------------------------------------------------------------
# step 0: prepare
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
init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose, pb_client)  # load robot
demo.get_joint_link_info("arm")  # get info about arm
init_joint = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(init_joint)  # reset arm joint position

# load kitchen
kitchen = Kitchen(pb_client)
print("object ids in loaded kitchen:\n{}".format(kitchen.object_ids))
print('-'*20 + '\n' + 'Preparation has been completed!')

# ----------------------------------------------------------------
# step 1: create a scenario
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
# step 2: create a service request
# ----------------------------------------------------------------
service_request = "put all fruit into the fridge"
print('-'*20 + '\n' + 'A service request have been obtained!')

# ----------------------------------------------------------------
# step 3: take screenshot: front and top
# ----------------------------------------------------------------
# TODO: the issue is the fruit image is too small to be seen
# TODO: the solution is using multiple cameras

# camera 1: fridge
pb_client.enable_vertical_view(1.2, [3.76, 4.76, 1.22], yaw=270, pitch=-22.40) # top view
pb_client.run(10)
pb_visualizer.capture_screen('fridge', enable_Debug=False)

# camera 2: table
pb_client.enable_vertical_view(1.0, [4.31, 4.31, 0.91], yaw=270, pitch=-41.2) # top view
pb_client.run(10)
pb_visualizer.capture_screen('table', enable_Debug=False)

# camera 3: microwave_dishwasher_leftdrawers
pb_client.enable_vertical_view(1.4, [4.32, 2.82, 0.51], yaw=270, pitch=-16.80) # top view
pb_client.run(10)
pb_visualizer.capture_screen('microwave_dishwasher_leftdrawers', enable_Debug=False)

# camera 4: stove_countertop
pb_client.enable_vertical_view(1.4, [4.31, 1.37, 0.50], yaw=270, pitch=-48.00) # top view
pb_client.run(10)
pb_visualizer.capture_screen('stove_countertop', enable_Debug=False)

# camera 5: topcabinet
pb_client.enable_vertical_view(1.6, [4.32, 1.73, 1.92], yaw=270, pitch=-5.67) # top view
pb_client.run(10)
pb_visualizer.capture_screen('topcabinet', enable_Debug=False)

# camera 6: bottomdrawers
pb_client.enable_vertical_view(1.4, [4.32, 1.59, 0.39], yaw=270, pitch=-5.67) # top view
pb_client.run(10)
pb_visualizer.capture_screen('bottomdrawers', enable_Debug=False)

print('-'*20 + '\n' + 'Images have been captured!')

# ----------------------------------------------------------------
# step 4: query vlm and get intial state
# ----------------------------------------------------------------

# ----------------------------------------------------------------
# step 5: create pddl problem
# ----------------------------------------------------------------

# ----------------------------------------------------------------
# step 6: generate task plan
# ----------------------------------------------------------------

# ----------------------------------------------------------------
# step 7: execute task plan
# ----------------------------------------------------------------

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

# disconnect from server
pb_client.wait(500)
pb_client.disconnect_pybullet()