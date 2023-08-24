#!/usr/bin/python3
import sys
import os
import logging
import numpy as np
import pybullet as p

#Note about the 2F-85:  
#The chassis and fingers are made out of anodized aluminum 6061 T6. Mass density = 0.0027 Kg/cm^3
#The finger bars are made out of stainless steel 17-4 ph. Mass density = 0.0028 Kg/cm^3
#The silicon of the finger tips is 60A

class Robotiq2F85:
    def __init__(self, simulatorConnectionID, robotUID=None, startingPosition=[0,0,0.001], startingOrientationRAD=[0,0,0]):

        self._cid = simulatorConnectionID

        #If no robot unique id (robotUID) is supplied, we assume that we want to instantiate a new 2F85
        if robotUID == None:
            startPos         = startingPosition
            startOrientation = p.getQuaternionFromEuler(startingOrientationRAD)

            dirname  = os.path.dirname(__file__)
            filename = os.path.join(dirname, '../urdf/robotiq_2f85.urdf')
            self.gripper     = [p.loadURDF(filename, startPos, startOrientation, flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)]
        else:
            self.gripper = [robotUID]

        #Maximum closing of the gripper
        self.MAX_CLOSING = np.deg2rad(45) # 45
        #Maximum force of the gripper
        self.MAX_FORCE   = 10
        #Use a single actuator with gears
        self.USE_SINGLE_ACTUATOR = False

        #We need to add a constraint for each finger of the gripper to close the kinematic loop.
        #The constraint is specified relative to the Center of Mass of the link. The CAD can be used
        #to locate the joint relative to the center of mass.
        right_follower_index   = self.getJointIndexFromName('gripper_right_follower_joint')
        right_springlink_index = self.getJointIndexFromName('gripper_right_spring_link_joint')
        c = p.createConstraint(parentBodyUniqueId=self.getUID(), parentLinkIndex=right_follower_index, 
            childBodyUniqueId=self.getUID(), childLinkIndex=right_springlink_index, 
            jointType=p.JOINT_POINT2POINT, jointAxis=[1, 0, 0], parentFramePosition=[6/1000,-6.7/1000,0], childFramePosition=[0,28.9/1000,0])
        #Make the constraint very strong so an actuation cannot separate the links
        #For details about ERP: https://raw.githubusercontent.com/bulletphysics/bullet3/master/examples/Utils/b3ERPCFMHelper.hpp
        p.changeConstraint(c, maxForce=9999, erp=0.8)

        left_follower_index   = self.getJointIndexFromName('gripper_left_follower_joint')
        left_springlink_index = self.getJointIndexFromName('gripper_left_spring_link_joint')
        c = p.createConstraint(parentBodyUniqueId=self.getUID(), parentLinkIndex=left_follower_index, 
            childBodyUniqueId=self.getUID(), childLinkIndex=left_springlink_index, 
            jointType=p.JOINT_POINT2POINT, jointAxis=[1, 0, 0], parentFramePosition=[6/1000,-6.7/1000,0], childFramePosition=[0,28.9/1000,0])
        #Make the constraint very strong so an actuation cannot separate the links
        p.changeConstraint(c, maxForce=9999, erp=0.8)

        #There is a pin fixed on the driver link which limits the movement of the coupler link. If we dont use self-collision (for performances and stability purposes)
        #then we need to make sure that the coupler does not move through the pin. We can do that by limiting the rotation of the coupler/driver joint. This constraint
        #is implemented in the URDF but is respected ONLY IF its also implemented here. Try commenting out the two following lines to test it.
        p.changeDynamics(bodyUniqueId=self.getUID(), linkIndex=self.getJointIndexFromName('gripper_right_coupler_joint'), jointLowerLimit=0.13 , jointUpperLimit=0.79)
        p.changeDynamics(bodyUniqueId=self.getUID(), linkIndex=self.getJointIndexFromName('gripper_left_coupler_joint'),  jointLowerLimit=0.13 , jointUpperLimit=0.79)

        #A JOINT_GEAR can be used to actuate the left finger when the right one is actuated but external forcces
        #can break the symmetry so its better to use two separate actuators instead.
        if self.USE_SINGLE_ACTUATOR == True:
            right_driver_joint_index   = self.getJointIndexFromName('gripper_right_driver_joint')
            left_driver_joint_index    = self.getJointIndexFromName('gripper_left_driver_joint')
            #A JOINT_GEAR is unidirectional so two constraints needs to be added to have the real gear behaviour
            c = p.createConstraint(parentBodyUniqueId=self.getUID(), parentLinkIndex=right_driver_joint_index, 
                childBodyUniqueId=self.getUID(), childLinkIndex=left_driver_joint_index,  jointType=p.JOINT_GEAR, 
                jointAxis=[0,0,1], parentFramePosition=[0,0,0], childFramePosition=[0,0,0])
            p.changeConstraint(c, gearRatio=-1, maxForce=self.MAX_FORCE)
            c = p.createConstraint(parentBodyUniqueId=self.getUID(), parentLinkIndex=left_driver_joint_index, 
                childBodyUniqueId=self.getUID(), childLinkIndex=right_driver_joint_index,  jointType=p.JOINT_GEAR, 
                jointAxis=[0,0,1], parentFramePosition=[0,0,0], childFramePosition=[0,0,0])
            p.changeConstraint(c, gearRatio=-1, maxForce=self.MAX_FORCE)

        # WATCH OUT!!! In PyBullet, all revolute and slider joints have active motors by default. This does not work well for an underactuated mechanism.
        # Therefore, we need to disable some of them by setting a maximal force to 0 and allowing a full 360 degres rotation
        p.setJointMotorControl2(bodyIndex=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_right_follower_joint'), controlMode=p.VELOCITY_CONTROL, force=0)
        p.setJointMotorControl2(bodyIndex=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_left_follower_joint'),  controlMode=p.VELOCITY_CONTROL, force=0)
        p.setJointMotorControl2(bodyIndex=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_left_spring_link_joint'),  controlMode=p.VELOCITY_CONTROL, force=0)
        p.setJointMotorControl2(bodyIndex=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_right_spring_link_joint'),  controlMode=p.VELOCITY_CONTROL, force=0)
        p.setJointMotorControl2(bodyIndex=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_right_coupler_joint'),  controlMode=p.VELOCITY_CONTROL, force=0)
        p.setJointMotorControl2(bodyIndex=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_left_coupler_joint'),  controlMode=p.VELOCITY_CONTROL, force=0)
        
        #If the left finger is driven by the right one, the left driver must not be motorized
        if self.USE_SINGLE_ACTUATOR == True:
            p.setJointMotorControl2(bodyIndex=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_left_driver_joint'),  controlMode=p.VELOCITY_CONTROL, force=0)
 
        #The mass and inertia of the links are grossly inaccurate in most (all?) models available online but the impact of these values on the simulation can be very important.
        #The following values were obtained from the CAD supplied by Robotiq using the anodized aluminum 6061 T6 with mass density = 0.0027 Kg/cm^3
        #PyBullet/URDF uses Mass specified in Kg and inertia in Kg/m^2
            #The right inertial parameters are now implemented in the URDF

        #Initialize the state of the gripper to fully open
        p.resetJointState(bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_right_driver_joint'), targetValue=0)
        p.resetJointState(bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_left_driver_joint'),  targetValue=0)

    #Use a single actuator with gears instead of two independant actuators
    def useSingleActuator(self, useSingle):
        self.USE_SINGLE_ACTUATOR = useSingle

    def setMaxForce(self, max_force):
        self.MAX_FORCE = max_force

    def getUID(self):
        return self.gripper[0]

    def initJoints(self, jointsPositionDEG):
        #Initialize the state of the gripper to fully open
        p.resetJointState(bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_right_driver_joint'), targetValue=jointsPositionDEG[0])
        p.resetJointState(bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_left_driver_joint'),  targetValue=jointsPositionDEG[1])

    #Returns the joints positions of the gripper
    def getGripperState(self):
        current_joint_position = [  p.getJointState(bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_left_driver_joint'))[0],
                                    p.getJointState(bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_right_driver_joint'))[0] ]
        return current_joint_position

    def getCouplerState(self):
        current_joint_position = [  p.getJointState(bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_left_coupler_joint'))[0],
                                    p.getJointState(bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_right_coupler_joint'))[0] ]
        return current_joint_position

    #Returns joints position error from goal position
    def getStateError(self, gripper_joint_positions_goal):
        g = gripper_joint_positions_goal
        state = self.getGripperState()
        error = np.linalg.norm([state[0]-g[0]*(self.MAX_CLOSING/100),state[1]-g[1]*(self.MAX_CLOSING/100)])
        return error

    def getMaximumClosing(self):
        #The driver joint is motorized. The value of 0.7 for maximum closing has been found by trial and error
        return self.MAX_CLOSING

    #Uses position control to actuate each driver joint of the gripper
    #Each of left_finger_goal, right_finger_goal is a percentage that specify how close each finger is (100 = Fully closed, 0 = Fully open)
    def setGoal(self, left_finger_goal, right_finger_goal):
        
        right_finger_goal = max(min(right_finger_goal,100),0)
        left_finger_goal  = max(min(left_finger_goal,100),0)

        p.setJointMotorControl2(targetPosition=right_finger_goal*self.MAX_CLOSING/100, bodyIndex=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_right_driver_joint'), controlMode=p.POSITION_CONTROL, maxVelocity=1, force=self.MAX_FORCE)
        p.setJointMotorControl2(targetPosition=left_finger_goal*self.MAX_CLOSING/100,  bodyIndex=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_left_driver_joint'),  controlMode=p.POSITION_CONTROL, maxVelocity=1, force=self.MAX_FORCE)
        if self.USE_SINGLE_ACTUATOR == False:
            p.setJointMotorControl2(targetPosition=left_finger_goal*self.MAX_CLOSING/100,  bodyIndex=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_left_driver_joint'),  controlMode=p.POSITION_CONTROL, maxVelocity=1, force=self.MAX_FORCE)
        
        #Acts as the spring in the gripper that keeps the pads parallel
        p.setJointMotorControl2(targetPosition=self.MAX_CLOSING, bodyIndex=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_right_spring_link_joint'), controlMode=p.POSITION_CONTROL, force=1)
        p.setJointMotorControl2(targetPosition=-1*self.MAX_CLOSING, bodyIndex=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_left_spring_link_joint'),  controlMode=p.POSITION_CONTROL, force=1) 

    #Build a dictionnary that can then be used to efficiently retrieve
    #joints unique IDs from their names.
    def buildJointIndexFromNameDict(self):
        self.jointsNamesDict = {}

        numJoints = p.getNumJoints(self.getUID(), self._cid)

        for i in range(numJoints):
            jointInfo = p.getJointInfo(self.getUID(), i, self._cid)
            strJointName = jointInfo[1].decode('UTF-8')
            self.jointsNamesDict[strJointName] = jointInfo[0]

    #Retrieve the unique ID associated with a joint name.
    #Returns None if the joint name is not found
    def getJointIndexFromName(self, JointName):
        #Verify that the joints dictonary has been populated
        #if its not the case, do it.
        try:
            if len(self.jointsNamesDict) == 0:
                self.buildJointIndexFromNameDict()
        except AttributeError:
            self.buildJointIndexFromNameDict()

        jointId = self.jointsNamesDict.get(JointName)
        if jointId == None:
            logging.warning('Cannot find a joint named: '+JointName)
        return jointId
