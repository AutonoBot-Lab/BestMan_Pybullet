from dataclasses import dataclass
from typing import Type

import numpy as np
from PIL import Image

@dataclass
class CameraParameters:
    """Used to store the camera's intrinsic parameters, head tilt angle, image data, color data, and depth data.
    """
    
    fx: float                       # The focal length scaling factor in the x direction. It is typically the focal length in pixel units and is used to describe the camera's intrinsic parameters.
    fy: float                       # The focal length scaling factor in the y direction. Similar to fx, but for the y direction.
    cx: float                       # The x-coordinate of the camera's principal point (optical center). It represents the position of the optical axis's intersection on the image plane in the x direction, usually near the center of the image.
    cy: float                       # The y-coordinate of the camera's principal point (optical center). Similar to cx, but for the y direction.
    head_tilt: float                # The tilt angle of the camera head, describing the camera's rotation or inclination.
    image: Type[Image.Image]        # An image object from the PIL library, representing the captured image data.
    colors: np.ndarray              # A NumPy array containing color data of the image, typically used for processing color information within the image.
    depths: np.ndarray              # A NumPy array containing depth data of the image, representing the distance of each pixel point from the camera.