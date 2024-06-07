import numpy as np
from scipy.spatial.transform import Rotation as R

class Pose:
    def __init__(self, position, orientation):
        """
        Initialize a new Pose object.

        Parameters:
            position (list): A list of three floats representing the position in 3D space.
            orientation (list or np.ndarray): A list of three floats representing the orientation in 3D space (Euler angles)
                                              or a 3x3 rotation matrix.

        Attributes:
            position (list): A list of three floats representing the position in 3D space.
            x (float): The X coordinate of the position.
            y (float): The Y coordinate of the position.
            z (float): The Z coordinate of the position.
            orientation (list): A list of three floats representing the orientation in 3D space.
            roll (float): The roll component of the orientation.
            pitch (float): The pitch component of the orientation.
            yaw (float): The yaw component of the orientation.
        """
        
        self.position = position
        self.x, self.y, self.z = position
        
        # Automatically detect orientation type and convert
        if isinstance(orientation, np.ndarray) and orientation.shape == (3, 3):
            # If it is a 3x3 numpy ndarray, it is assumed to be a rotation matrix
            r = R.from_matrix(orientation)
            self.orientation = r.as_euler('xyz', degrees=False)
        elif isinstance(orientation, (list, np.ndarray)) and len(orientation) == 3:
            # If it is a list or a one-dimensional numpy ndarray containing three elements, Euler angles are assumed
            self.orientation = orientation
        else:
            raise ValueError("Orientation must be a list of three floats or a 3x3 rotation matrix")
        
        self.roll, self.pitch, self.yaw = self.orientation