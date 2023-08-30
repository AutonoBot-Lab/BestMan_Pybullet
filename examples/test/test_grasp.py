import pybullet as p
import pybullet_data
import time
import math

def main():
    # Start simulation
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    # Load plane and robot
    p.loadURDF("plane.urdf")
    robot = p.loadURDF("./URDF_robot/model_elephant/urdf/ur5_robotiq_85.urdf")
    
    # Get joint indices
    joint_indices = [i for i in range(p.getNumJoints(robot))]
    
    # Get the debug parameter ids
    joint_positions = [p.addUserDebugParameter("joint_{}".format(i), -math.pi, math.pi, 0) for i in joint_indices]
    gripper_open_id = p.addUserDebugParameter("gripper_open", 0, 1, 1)
    
    while True:
        # Read the debug parameters
        target_joint_positions = [p.readUserDebugParameter(pos) for pos in joint_positions]
        gripper_open = p.readUserDebugParameter(gripper_open_id)
        
        # Move the robot
        p.setJointMotorControlArray(robot, joint_indices, p.POSITION_CONTROL, targetPositions=target_joint_positions)

        # Move the gripper
        gripper_joint_indices = [joint_indices[-2], joint_indices[-1]]
        gripper_open_length = gripper_open * 0.02
        p.setJointMotorControlArray(robot, gripper_joint_indices, p.POSITION_CONTROL, targetPositions=[gripper_open_length, -gripper_open_length])

        p.stepSimulation()
        time.sleep(0.01)

if __name__ == '__main__':
    main()
