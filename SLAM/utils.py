import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

class AreaBounds:
        
	def __init__(self, obstacles_bounds):
		obstacles_bounds_arr = np.array(obstacles_bounds)
		self.x_min, self.y_min, _, _ = np.min(obstacles_bounds_arr, axis=0)
		_, _, self.x_max, self.y_max = np.max(obstacles_bounds_arr, axis=0)
		self.x_min = min([self.x_min]) -2
		self.y_min = min([self.y_min]) -2
		self.x_max = max([self.x_max]) +2
		self.y_max = max([self.y_max]) +2


def plot_rectangle(x_min, y_min, x_max, y_max):
    width = x_max - x_min
    height = y_max - y_min
    rect = Rectangle((x_min, y_min), width, height, edgecolor='black', facecolor='black')
    plt.gca().add_patch(rect)