"""
@Description :   A few functions used in PID control
@Author      :   Yan Ding 
@Time        :   2023/08/31 02:50:31
"""

import cv2
import pybullet as p
import pybullet_data
import math
import time
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import random
import sys
import os
from matplotlib.colors import LinearSegmentedColormap

"""
PID controller
"""


class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        """
        Initialize the PID controller with the given parameters. It is assumed
        that the setpoint is a function of the PID controller and that the
        parameters are in the form of numpy arrays.
        Args:
            Kp: The parameter matrix for the Poisson process.
            Ki: The parameter matrix for the Integral process.
            Kd: The parameter matrix for the Dirichlet process.
            setpoint: The target value for the PID controller.
        """
        # Initialize the PID controller with the given parameters
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint

        # Initialize the variables used in the PID algorithm
        self.last_error = 0
        self.integral = 0

    def calculate(self, process_value):
        """
        Calculate the PID for a given process value.
        Args: process_value: The value of the process to be controlled.
        Returns: The output of the PID controller.
        """
        # Calculate the error, integral and derivative terms
        error = self.setpoint - process_value
        self.integral += error
        derivative = error - self.last_error
        # Calculate the PID output
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        # Update the variables
        self.last_error = error
        return output

    def set_goal(self, setpoint):
        """
        Set the goal that will be used for this step.
        Args: setpoint: The Goal to be reached.
        """
        self.setpoint = setpoint
