import math
import time
import numpy as np

import pybullet as p

from utils import Bestman, Pose, Kitchen, PbOMPL

# This script demonstrates the functionality of the PbOMPL class for motion planning
# using the PyBullet simulation environment. It initializes a simulated environment
# with a robotic arm and a kitchen setup, showcasing operations related to motion planning
# and obstacle management.

# Initialize the simulation
# load robot
init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose)
demo.enable_vertical_view(4, [1, 1, 0])

# reset arm joint position
pose1 = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(pose1)

# load kitchen
kitchen_id = Kitchen()

joint_idx = [0, 1, 2, 3, 4, 5]
# Create an instance of the PbOMPL class
ompl = PbOMPL(
    robot_id=demo.arm_id,
    joint_idx=joint_idx,
    obstacles=[],
    planner="RRTConnect",
    threshold=0.1,
)

ompl.get_scene_items()

# Add additional obstacles
obstacle1 = 1
obstacle2 = 2
ompl.add_obstacles(obstacle1)
ompl.add_obstacles(obstacle2)

# Print the obstacles
print("Obstacles after adding:")
ompl.check_obstacles()

# Remove an obstacle
ompl.remove_obstacles(obstacle1)

# Print the obstacles after removal
print("Obstacles after removal:")
ompl.check_obstacles()

# Set new obstacles
new_obstacles = [0, 1, 2]
ompl.set_obstacles(new_obstacles)

# Print the updated obstacles
print("Obstacles after setting new ones:")
ompl.check_obstacles()

# Set new obstacles
new_obstacles = []
ompl.set_obstacles(new_obstacles)

# Print the updated obstacles
print("Obstacles after setting new ones:")
ompl.check_obstacles()

ompl.add_scene_obstacles()
# Print the updated obstacles
print("Add scene obstacles:")
ompl.check_obstacles()

ompl.store_obstacles()
print(ompl.obstacles)

# Output
# Item Name: plane, ID: 0
# Item Name: segbot, ID: 1
# Item Name: ur5e, ID: 2
# Item Name: kitchen_part_right, ID: 3
# Obstacles after adding:
#          Obstacle Name: segbot, ID: 1
#          Obstacle Name: ur5e, ID: 2
# Obstacles after removal:
#          Obstacle Name: ur5e, ID: 2
# Obstacles after setting new ones:
#          Obstacle Name: plane, ID: 0
#          Obstacle Name: segbot, ID: 1
#          Obstacle Name: ur5e, ID: 2
# Obstacles after setting new ones:
# Obstacle list is empty
# Add scene obstacles:
#          Obstacle Name: plane, ID: 0
#          Obstacle Name: segbot, ID: 1
#          Obstacle Name: kitchen_part_right, ID: 3
# [0, 1, 3]
