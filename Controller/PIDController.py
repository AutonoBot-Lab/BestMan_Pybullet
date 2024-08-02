#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
# @FileName      : PIDController
# @Time          : 2024-08-01 20:26:50
# @Author        : kui yang
# @Email         : yangkui1127@gmail.com
# @description   : PID Controller
"""

class PIDController:
    """
    A PID (Proportional-Integral-Derivative) controller for computing control output based on
    the difference between a setpoint and a process value.
    """
    def __init__(self, Kp, Ki, Kd, setpoint):
        """
        Initialize the PID controller with the given parameters. It is assumed
        that the setpoint is a function of the PID controller and that the
        parameters are in the form of numpy arrays.

        Args:
            Kp (float or numpy.ndarray): The proportional gain parameter.
            Ki (float or numpy.ndarray): The integral gain parameter.
            Kd (float or numpy.ndarray): The derivative gain parameter.
            setpoint (float): The target value for the PID controller.
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
        Calculate the PID output for a given process value.

        Args:
            process_value (float): The value of the process to be controlled.

        Returns:
            float: The output of the PID controller.
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

        Args:
            setpoint (float): The goal to be reached.
        """
        self.setpoint = setpoint
