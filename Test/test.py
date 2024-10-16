import time

import pybullet as p
import pybullet_data

# Connect to PyBullet, use GUI for visualization
physicsClient = p.connect(p.GUI)

# Set additional search path to find PyBullet's default URDFs
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load a URDF file (in this example, a plane and a robot arm)
plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF(
    "/home/yk/GitHub_Code/BestMan_Pybullet/Asset/Robot/mobile_manipulator/base/ranger_miniv3/urdf/ranger_mini_v3.urdf",
    basePosition=[0, 0, 0.4],
)

# Set gravity in the simulation
g = -9.8
p.setGravity(0, 0, g)

# Run the simulation for a few seconds
for _ in range(10000):
    p.stepSimulation()
    time.sleep(1 / 240.0)  # Sleep to match simulation step time

# Disconnect from the simulation
p.disconnect()
