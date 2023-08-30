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

# This script serves the purpose of determining the grasp position through debugging lines.
# It can be replaced with the `test_grasp_force` script for achieving the same result.

# load cleint
pb_client = PbClient(enable_GUI=True, enable_Debug=True)
pb_client.enable_vertical_view(1.0, [1.7, 3.68, 1.95], -86.4, -52.3)

# load visualizer
pb_visualizer = PbVisualizer(pb_client)

# load kitchen
kitchen = Kitchen(pb_client)

pb_visualizer.set_elementB_visual_color

grasp_position = [3.95, 3.1, 1.6]  # hard code
grasp_position2 = [3.85, 3.1, 1.6]  # hard code

# debug point show grasp point
debug_point_id2 = p.addUserDebugLine(
    lineFromXYZ=grasp_position,
    lineToXYZ=grasp_position2,
    lifeTime=0,  # always display
    lineWidth=5,
    lineColorRGB=[1, 0, 0],
)

pb_client.wait(120)