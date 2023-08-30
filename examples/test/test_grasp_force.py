import os
import time
import pdb
import math
from collections import namedtuple
from attrdict import AttrDict
import pybullet as p
import pybullet_data
from utils.utils_Bestman import Bestman, Pose
from utils.utils_PbClient import PbClient
from utils.utils_PbVisualizer import PbVisualizer
from utils.utils_Kitchen_object import Kitchen
from utils.utils_UR5_2F85 import UR5_2F85
from utils.utils_robot_elephant import UR5Robotiq85

# This script separates the Robotiq 2F85 gripper from the robotic arm. 
# This separation facilitates testing of grasp positions and forces.

# load cleint
pb_client = PbClient(enable_GUI=True, enable_Debug=True)
pb_client.enable_vertical_view(1.0, [1.7, 3.68, 1.95], -86.4, -52.3)

# load visualizer
pb_visualizer = PbVisualizer(pb_client)

# load kitchen
kitchen = Kitchen(pb_client)

# load visualizer
pb_visualizer.set_elementB_visual_color

# load object
bowl_id = pb_client.load_object(
    "./URDF_models/utensil_bowl_blue/model.urdf", [0.6, 0.6, 0.85], [0.0, 0.0, 0.0], 1.0, "bowl"
)

robot = UR5Robotiq85((0, 0.5, 0), (0, 0, 0))