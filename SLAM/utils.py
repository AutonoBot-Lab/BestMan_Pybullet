import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

class AreaBounds:
	"""A class to represent the bounding area for navigation obstacles.

    Attributes:
        x_min (float): Minimum x-coordinate of the bounding area.
        y_min (float): Minimum y-coordinate of the bounding area.
        x_max (float): Maximum x-coordinate of the bounding area.
        y_max (float): Maximum y-coordinate of the bounding area.
    """

	def __init__(self, obstacles_bounds):
		"""
        Initializes the AreaBounds class.

        Args:
            obstacles_bounds (list): A list of obstacle bounds in the format [x_min, y_min, x_max, y_max].
        """
		obstacles_bounds_arr = np.array(obstacles_bounds)
		self.x_min, self.y_min, _, _ = np.min(obstacles_bounds_arr, axis=0)
		_, _, self.x_max, self.y_max = np.max(obstacles_bounds_arr, axis=0)
		self.x_min = min([self.x_min]) -2
		self.y_min = min([self.y_min]) -2
		self.x_max = max([self.x_max]) +2
		self.y_max = max([self.y_max]) +2


def plot_rectangle(x_min, y_min, x_max, y_max):
	"""
    Plots a rectangle on the current matplotlib axis.

    Args:
        x_min (float): Minimum x-coordinate of the rectangle.
        y_min (float): Minimum y-coordinate of the rectangle.
        x_max (float): Maximum x-coordinate of the rectangle.
        y_max (float): Maximum y-coordinate of the rectangle.
    """
	width = x_max - x_min
	height = y_max - y_min
	rect = Rectangle((x_min, y_min), width, height, edgecolor='black', facecolor='black')
	plt.gca().add_patch(rect)