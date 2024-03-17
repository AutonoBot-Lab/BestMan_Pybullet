class Pose:
    def __init__(self, position, orientation):
        """
        Initialize a new Pose object.

        Parameters:
            position (list): A list of three floats representing the position in 3D space.
            orientation (list): A list of three floats representing the orientation in 3D space.

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
        
        self.orientation = orientation
        self.roll, self.pitch, self.yaw = orientation