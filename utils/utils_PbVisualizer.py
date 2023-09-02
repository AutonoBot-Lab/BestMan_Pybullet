"""
@Description :   A few functions for visualization
@Author      :   Yan Ding 
@Time        :   2023/08/30 22:48:04
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
from matplotlib.colors import LinearSegmentedColormap


"""
Visualization class
"""

# color list
colors = {
    "white": [1, 1, 1, 1],
    "wood_light": [0.87, 0.72, 0.53, 1],
    "wood_heavy": [0.4, 0.26, 0.13, 1],
    "stainless_steel": [0.59, 0.59, 0.59, 1],
    "wood_dark": [0.4, 0.26, 0.13, 1],
    "cream": [1, 0.99, 0.82, 1],
    "grey": [0.86, 0.86, 0.86, 1],
    "blue": [0.53, 0.81, 0.92, 1.0],
}

class PbVisualizer:
    def __init__(self, pb_client):
        self.client_id = pb_client.get_client()
        self.line_visual = None

    # ----------------------------------------------------------------
    # Visualization functions
    # ----------------------------------------------------------------
    def crop_image(self, image, center, size):
        """
        Crop a given image around a specified center point and returns the cropped portion. The resulting cropped image will be a square with the provided size. If the cropping dimensions exceed the original image boundaries, the function ensures it stays within the original image's dimensions to prevent out-of-bounds access.

        Args:
            image (np.array): The original image to be cropped. It is assumed to be a two-dimensional array, representing pixel values.
            center (tuple): A tuple (x, y) specifying the center point around which the image will be cropped.
            size (int): The side length of the resulting square cropped image.

        Returns:
            np.array: A cropped portion of the original image centered around the specified center point and with the given size.
        """
        image_height, image_width = image.shape
        top = max(0, int(center[1] - size / 2))
        bottom = min(image_height, int(center[1] + size / 2))
        left = max(0, int(center[0] - size / 2))
        right = min(image_width, int(center[0] + size / 2))
        
        return image[top:bottom, left:right]

    def get_depth_image(
        self, basePos, cameraPos, cameraUp, enable_show=False, enable_save=False
    ):
        """
        Capture a cropped depth image from the PyBullet simulation. It positions the camera based on specified positions and orientations, captures the scene, extracts the depth information, and then crops the depth image around its center. Depth images are representations where pixel values indicate distances to the objects from the camera's perspective. Such images are valuable in robotics and computer vision applications for understanding the spatial relationships and distances between objects in a scene.
        Args:
            basePos (tuple): The target or base position in the scene towards which the camera is oriented. Typically, this could be the position of an object of interest, like a table.
            cameraPos (tuple): The position coordinates (x, y, z) where the camera is placed in the simulation environment.
            cameraUp (tuple): The upward direction vector of the camera, which determines the camera's orientation. For instance, (0, 1, 0) would point the camera's upward direction along the positive y-axis.

        Returns:
            np.array: A cropped depth image centered around the midpoint of the captured image, providing depth (distance) information from the camera's perspective.
        """
        viewMatrix = p.computeViewMatrix(
            cameraPos, basePos, cameraUp, physicsClientId=self.client_id
        )
        projectionMatrix = p.computeProjectionMatrixFOV(
            fov=60, aspect=1.0, nearVal=0.01, farVal=100.0, physicsClientId=self.client_id
        )

        img_arr = p.getCameraImage(
            640, 480, viewMatrix, projectionMatrix, shadow=1, lightDirection=[1, 1, 1]
        )
        depth_buffer = img_arr[3]  # depth buffer
        near = 0.01  # Near plane distance
        far = 100  # Far plane distance
        depth = far * near / (far - (far - near) * depth_buffer)
        # personalized colormap
        cdict = {
            "red": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
            "green": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
            "blue": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
        }
        custom_cmap = LinearSegmentedColormap("custom_cmap", cdict)
        image_point = [320, 240]

        crop_size = 200
        depth_image = self.crop_image(depth, image_point, crop_size)

        print("depth size:{} * {}".format(len(depth_image), len(depth_image[0])))
        print("image_point:{}".format(image_point))

        if enable_show:
            plt.imshow(depth_image, cmap=custom_cmap)
            plt.colorbar()
            plt.show()

        if enable_save:
            plt.imsave("depth_image.png", depth_image, cmap=custom_cmap)

        return depth_image

    def draw_aabb(self, object_id):
        """
        Draw an Axis-Aligned Bounding Box (AABB) around the specified table object in the simulation. The AABB is a box that covers the entire object based on its maximum and minimum coordinates along each axis. It can be useful for various purposes, such as collision detection, spatial partitioning, and bounding volume hierarchies.

        Args:
            table_id: The unique identifier of the table object in the simulation for which the AABB is to be drawn.
        """
        link_ids = [
            i for i in range(-1, p.getNumJoints(object_id, physicsClientId=self.client_id))
        ]
        for link_id in link_ids:
            aabb = p.getAABB(object_id, link_id)
            aabb_min = aabb[0]
            aabb_max = aabb[1]
            corners = [
                [aabb_min[0], aabb_min[1], aabb_min[2]],  # 0
                [aabb_max[0], aabb_min[1], aabb_min[2]],  # 1
                [aabb_max[0], aabb_max[1], aabb_min[2]],  # 2
                [aabb_min[0], aabb_max[1], aabb_min[2]],  # 3
                [aabb_min[0], aabb_min[1], aabb_max[2]],  # 4
                [aabb_max[0], aabb_min[1], aabb_max[2]],  # 5
                [aabb_max[0], aabb_max[1], aabb_max[2]],  # 6
                [aabb_min[0], aabb_max[1], aabb_max[2]],  # 7
            ]
            lines = [
                (0, 1),
                (1, 2),
                (2, 3),
                (3, 0),  # bottom face
                (4, 5),
                (5, 6),
                (6, 7),
                (7, 4),  # top face
                (0, 4),
                (1, 5),
                (2, 6),
                (3, 7),  # vertical edges
            ]
            color = [1, 0, 0]
            for line in lines:
                p.addUserDebugLine(
                    lineFromXYZ=corners[line[0]],
                    lineToXYZ=corners[line[1]],
                    lineColorRGB=color,
                    lineWidth=2,
                    physicsClientId=self.client_id,
                )

    def draw_aabb_link(self, object_id, link_id=-1):
        """
        Draw an Axis-Aligned Bounding Box (AABB) around the specified object or link in the simulation. The AABB is a box that covers the entire object based on its maximum and minimum coordinates along each axis. It can be useful for various purposes, such as collision detection, spatial partitioning, and bounding volume hierarchies.

        Args:
            object_id: The unique identifier of the object in the simulation for which the AABB is to be drawn.
            link_id: The index of the link for which the AABB is to be drawn. Default is -1, which means the entire object.
        """
        aabb = p.getAABB(object_id, link_id, physicsClientId=self.client_id)
        aabb_min = aabb[0]
        aabb_max = aabb[1]
        corners = [
            [aabb_min[0], aabb_min[1], aabb_min[2]],  # 0
            [aabb_max[0], aabb_min[1], aabb_min[2]],  # 1
            [aabb_max[0], aabb_max[1], aabb_min[2]],  # 2
            [aabb_min[0], aabb_max[1], aabb_min[2]],  # 3
            [aabb_min[0], aabb_min[1], aabb_max[2]],  # 4
            [aabb_max[0], aabb_min[1], aabb_max[2]],  # 5
            [aabb_max[0], aabb_max[1], aabb_max[2]],  # 6
            [aabb_min[0], aabb_max[1], aabb_max[2]],  # 7
        ]
        lines = [
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),  # bottom face
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 4),  # top face
            (0, 4),
            (1, 5),
            (2, 6),
            (3, 7),  # vertical edges
        ]
        color = [1, 0, 0]
        for line in lines:
            p.addUserDebugLine(
                lineFromXYZ=corners[line[0]],
                lineToXYZ=corners[line[1]],
                lineColorRGB=color,
                lineWidth=2,
                physicsClientId=self.client_id,
            )

    def set_robot_visual_color(self, base_id, arm_id):
        """
        # Set the color of base
        """
        white = [1, 1, 1, 1]
        grey = [0.5, 0.5, 0.5, 1]
        for i in range(15):
            p.changeVisualShape(
                objectUniqueId=base_id,
                linkIndex=i,
                rgbaColor=colors["white"],
                physicsClientId=self.client_id,
            )
        p.changeVisualShape(
            objectUniqueId=base_id,
            linkIndex=4,
            rgbaColor=colors["grey"],
            physicsClientId=self.client_id,
        )
        p.changeVisualShape(
            objectUniqueId=base_id,
            linkIndex=5,
            rgbaColor=colors["grey"],
            physicsClientId=self.client_id,
        )
        p.changeVisualShape(
            objectUniqueId=base_id,
            linkIndex=6,
            rgbaColor=colors["grey"],
            physicsClientId=self.client_id,
        )

        """
        Set the color of arm
        """
        white = [1, 1, 1, 1]
        for i in range(15):
            p.changeVisualShape(
                objectUniqueId=arm_id, linkIndex=i, rgbaColor=colors["white"]
            )
        p.changeVisualShape(
            objectUniqueId=arm_id,
            linkIndex=0,
            rgbaColor=colors["blue"],
            physicsClientId=self.client_id,
        )
        p.changeVisualShape(
            objectUniqueId=arm_id,
            linkIndex=3,
            rgbaColor=colors["blue"],
            physicsClientId=self.client_id,
        )
        p.changeVisualShape(
            objectUniqueId=arm_id,
            linkIndex=6,
            rgbaColor=colors["blue"],
            physicsClientId=self.client_id,
        )

    def set_elementA_visual_color(self, elementA_id):
        """
        Set the color of element A (oven, container) in the kitchen.
        """
        for i in [1, 8, 9, 10, 11]: # white
            p.changeVisualShape(
                objectUniqueId=elementA_id,
                linkIndex=i,
                rgbaColor=colors["white"],
                physicsClientId=self.client_id,
            )

        for i in [2]: # wood_light
            p.changeVisualShape(
                objectUniqueId=elementA_id,
                linkIndex=i,
                rgbaColor=colors["wood_light"],
                physicsClientId=self.client_id,
            )

        for i in [3, 4, 5, 6]: # stainless_steel
            p.changeVisualShape(
                objectUniqueId=elementA_id,
                linkIndex=i,
                rgbaColor=colors["stainless_steel"],
                physicsClientId=self.client_id,
            )
        
        for i in [7]: # wood_dark
            p.changeVisualShape(
                objectUniqueId=elementA_id,
                linkIndex=i,
                rgbaColor=colors["wood_dark"],
                physicsClientId=self.client_id,
            )
        
        for i in [30, 33, 34, 42, 43]: # wood_light
            p.changeVisualShape(
                objectUniqueId=elementA_id,
                linkIndex=i,
                rgbaColor=colors["wood_light"],
                physicsClientId=self.client_id,
            )

    def set_elementB_visual_color(self, elementB_id):
        """
        Set the color of element B (counter) in the kitchen.
        """
        # link id:
        for i in [-1]: # wood_light
            p.changeVisualShape(
                objectUniqueId=elementB_id,
                linkIndex=i,
                rgbaColor=colors["wood_light"],
                physicsClientId=self.client_id,
            )

        # link id:
        for i in [0, 1]: # stainless_steel
            p.changeVisualShape(
                objectUniqueId=elementB_id,
                linkIndex=i,
                rgbaColor=colors["stainless_steel"],
                physicsClientId=self.client_id,
            )
        
        # link id:
        for i in [2]: # wood_dark
            p.changeVisualShape(
                objectUniqueId=elementB_id,
                linkIndex=i,
                rgbaColor=colors["wood_dark"],
                physicsClientId=self.client_id,
            )
        
    def set_elementC_visual_color(self, elementC_id):
        """
        Set the color of element C (dishwasher) in the kitchen.
        """
        # link id:
        for i in [0, 1]: # wood_light
            p.changeVisualShape(
                objectUniqueId=elementC_id,
                linkIndex=i,
                rgbaColor=colors["white"],
                physicsClientId=self.client_id,
            )
    
    def set_elementD_visual_color(self, elementD_id):
        """
        Set the color of element D (microwave) in the kitchen.
        """
        # link id:
        for i in [0]: # wood_light
            p.changeVisualShape(
                objectUniqueId=elementD_id,
                linkIndex=i,
                rgbaColor=colors["grey"],
                physicsClientId=self.client_id,
            )
        
        # link id:
        for i in [1]: # wood_light
            p.changeVisualShape(
                objectUniqueId=elementD_id,
                linkIndex=i,
                rgbaColor=colors["stainless_steel"],
                physicsClientId=self.client_id,
            )

    def set_elementE_visual_color(self, elementE_id):
        """
        Set the color of element E (fridge) in the kitchen.
        """
        # link id:
        for i in [0, 1, 2, 3]: # wood_light
            p.changeVisualShape(
                objectUniqueId=elementE_id,
                linkIndex=i,
                rgbaColor=colors["white"],
                physicsClientId=self.client_id,
            )


    def visualize_path(self, path):
        # Reverse the path so that it goes from start to goal
        path = path[::-1]  # This line reverses the path
        cartesian_path = [self.joints_to_cartesian(point) for point in path]
        # print("cartesian_path:{}".format(cartesian_path))
        for i in range(len(cartesian_path) - 1):
            p.addUserDebugLine(
                cartesian_path[i],
                cartesian_path[i + 1],
                lineColorRGB=[1, 0, 0],
                lifeTime=10,
                lineWidth=3,
                physicsClientId=self.client_id,
            )

    def draw_line(self, start_pos, target_pos, color=[1, 0, 0], width=3.0):
        """
        Draw a line on the screen from the specified start position to the target position.
        
        Args:
            start_pos: The starting position of the line as a tuple of (x, y, z) coordinates.
            target_pos: The ending position of the line as a tuple of (x, y, z) coordinates.
            color: A list representing the RGB values of the line's color. Default is red [1, 0, 0].
            width: The width of the line. Default is 3.0.
        """
        if self.line_visual is not None:
            p.removeUserDebugItem(self.line_visual, physicsClientId=self.client_id)
        
        self.line_visual = p.addUserDebugLine(
            start_pos,
            target_pos,
            lineColorRGB=color,
            lineWidth=width,
            physicsClientId=self.client_id,
        )
