"""
@Description :   A few functions for visualization
@Author      :   Yan Ding 
@Time        :   2023/08/30 22:48:04
"""

import cv2
import pybullet as p
import numpy as np
from PIL import Image
from datetime import datetime

"""
Visualization class
"""

# color list
colors = {
    "light_white": [1, 1, 1, 1],
    "wood_light": [0.87, 0.72, 0.53, 1],
    "wood_heavy": [0.4, 0.26, 0.13, 1],
    "stainless_steel": [0.59, 0.59, 0.59, 1],
    "wood_dark": [0.4, 0.26, 0.13, 1],
    "cream": [1, 0.99, 0.82, 1],
    "light_grey": [0.9, 0.9, 0.9, 1],
    "grey": [0.56, 0.56, 0.56, 1],
    "dark_grey": [0.13, 0.13, 0.13, 1],
    "blue": [0.53, 0.81, 0.92, 1.0],
    "light_blue": [0.9, 0.9, 1, 1],
    "light_white": [0.98, 0.98, 0.98, 1.0],
    "white": [1, 1, 1, 1.0],
    "black": [0, 0, 0, 1],
}


class Visualizer:
    def __init__(self, client, visualizer_cfg):
        self.client = client
        self.client_id = client.get_client_id()
        
        # Init camera pose
        self.set_camera_pose(visualizer_cfg.Camera)

    # ----------------------------------------------------------------
    # Scene camera
    # ----------------------------------------------------------------
    
    def set_camera_pose(self, camera_cfg):
        """
        Set the debug visualizer camera pose 

        Args:
            dist (float): The distance of the camera from the target point.
            position (list): A list of three floats representing the target position in 3D space.
            yaw (float, optional): The yaw component of the camera orientation. Defaults to 0.
            pitch (float, optional): The pitch component of the camera orientation. Defaults to -89.9.
        """
        
        p.resetDebugVisualizerCamera(
            cameraDistance=camera_cfg.dist,
            cameraYaw=camera_cfg.yaw,
            cameraPitch=camera_cfg.pitch,
            cameraTargetPosition=camera_cfg.position,
            physicsClientId=self.client_id,
        )
    
    
    def capture_screen(self, filename=None):
        """
            Continuously capture the screens of pybullet GUI and save the images to files.
            The file names will be based on the camera target position, distance, and yaw.

        Parameters:
            width (int): The width of the captured image.
            height (int): The height of the captured image.
        """
        
        # Get GUI information
        (
            width,
            height,
            viewMatrix,
            projectionMatrix,
            _,
            _,
            _,
            _,
            _,
            _,
            _,
            _,
        ) = p.getDebugVisualizerCamera()

        # Capture the screen
        _, _, rgb, _, _ = p.getCameraImage(
                                width=width,
                                height=height,
                                viewMatrix=viewMatrix,
                                projectionMatrix=projectionMatrix,
                                renderer=p.ER_BULLET_HARDWARE_OPENGL
                            )

        # Save the image to the file
        if filename == None:
            current_time = datetime.now().strftime('%Y%m%d_%H%M%S')
            rgb_path = f"../Examples/image/{current_time}.png"
        else:
            rgb_path = f"../Examples/image/{filename}.png"
        rgbImg = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
        cv2.imwrite(rgb_path, rgbImg)
        
        print("-" * 20 + "capture_screen is done!" + "-" * 20)
            
            
    # ----------------------------------------------------------------
    # Axes
    # ----------------------------------------------------------------
    
    def draw_axes(self, length=1.0, lineWidth=2.0, textSize=1.0):
        """
        Draws the x and y axes in the PyBullet environment with text labels.
        
        Parameters:
        - length (float): Length of the axes.
        - lineWidth (float): Width of the axes lines.
        - textSize (float): Size of the text labels.
        """
        origin = [0, 0, 0]  # The start point of the axes

        # Drawing the X-axis (in red)
        p.addUserDebugLine(
            lineFromXYZ=origin,
            lineToXYZ=[length, 0, 0],
            lineColorRGB=[1, 0, 0],
            lineWidth=lineWidth,
            physicsClientId=self.client_id,
        )

        # Drawing the Y-axis (in green)
        p.addUserDebugLine(
            lineFromXYZ=origin,
            lineToXYZ=[0, length, 0],
            lineColorRGB=[0, 1, 0],
            lineWidth=lineWidth,
            physicsClientId=self.client_id,
        )
        
        # Drawing the Z-axis (in blue)
        p.addUserDebugLine(
            lineFromXYZ=origin,
            lineToXYZ=[0, 0, length],
            lineColorRGB=[0, 0, 1],
            lineWidth=lineWidth,
            physicsClientId=self.client_id,
        )

        # Adding text labels
        p.addUserDebugText(
            text="X",
            textPosition=[length + 0.1, 0, 0],
            textColorRGB=[1, 0, 0],
            textSize=textSize,
            physicsClientId=self.client_id,
        )
        
        p.addUserDebugText(
            text="Y",
            textPosition=[0, length + 0.1, 0],
            textColorRGB=[0, 1, 0],
            textSize=textSize,
            physicsClientId=self.client_id,
        )
        
        p.addUserDebugText(
            text="Z",
            textPosition=[0, 0, length + 0.1],
            textColorRGB=[0, 0, 1],
            textSize=textSize,
            physicsClientId=self.client_id,
        )
    
    
    # ----------------------------------------------------------------
    # Video
    # ----------------------------------------------------------------

    def start_record(self, fileName):
        """
        Enable and disable recording
        """
        
        self.logId = p.startStateLogging(
            p.STATE_LOGGING_VIDEO_MP4,
            "../Examples/log/" + fileName + ".mp4",
            physicsClientId=self.client_id
        )
        
        print(
            "-" * 20
            + "\n"
            + "The video can be found in "
            + "Examples/log/"
            + fileName
            + ".mp4"
        )
        
        # return logId

    def end_record(self):
        p.stopStateLogging(self.logId, physicsClientId=self.client_id)
        
        
    # ----------------------------------------------------------------
    # line / aabb
    # ----------------------------------------------------------------

    def draw_line(self, start_pos, target_pos, color=[1, 0, 0], width=3.0):
        """
        Draw a line on the screen from the specified start position to the target position.

        Args:
            start_pos: The starting position of the line as a tuple of (x, y, z) coordinates.
            target_pos: The ending position of the line as a tuple of (x, y, z) coordinates.
            color: A list representing the RGB values of the line's color. Default is red [1, 0, 0].
            width: The width of the line. Default is 3.0.
        """

        self.line_visual = p.addUserDebugLine(
            start_pos,
            target_pos,
            lineColorRGB=color,
            lineWidth=width,
            physicsClientId=self.client_id,
        )
        
    def remove_all_line(self):
        p.removeAllUserDebugItems()

    def draw_aabb(self, object_id):
        """
        Draw an Axis-Aligned Bounding Box (AABB) around the specified table object in the simulation. The AABB is a box that covers the entire object based on its maximum and minimum coordinates along each axis. It can be useful for various purposes, such as collision detection, spatial partitioning, and bounding volume hierarchies.

        Args:
            table_id: The unique identifier of the table object in the simulation for which the AABB is to be drawn.
        """
        link_ids = [
            i
            for i in range(
                -1, p.getNumJoints(object_id, physicsClientId=self.client_id)
            )
        ]
        print(f"test: {link_ids}")
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

    def draw_aabb_link(self, object, link_id=-1):
        """
        Draw an Axis-Aligned Bounding Box (AABB) around the specified object or link in the simulation. The AABB is a box that covers the entire object based on its maximum and minimum coordinates along each axis. It can be useful for various purposes, such as collision detection, spatial partitioning, and bounding volume hierarchies.

        Args:
            object_id: The unique identifier of the object in the simulation for which the AABB is to be drawn.
            link_id: The index of the link for which the AABB is to be drawn. Default is -1, which means the entire object.
        """
        
        if isinstance(object, (int)):   # input type is object id
            object_id = object
        elif isinstance(object, (str)): # input type is object name
            object_id = getattr(self.client, object)
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
            
    def visualize_path(self, path):
        """
        visualize the path of Manipution
        """

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


    # ----------------------------------------------------------------
    # change color
    # ----------------------------------------------------------------
    
    def change_robot_color(self, base_id, arm_id, light_color=True):
        """
        Set the color of robot
        """

        # set the color of base
        base_num_joints = p.getNumJoints(base_id, physicsClientId=self.client_id)
        for i in range(base_num_joints):
            p.changeVisualShape(
                objectUniqueId=base_id,
                linkIndex=i,
                rgbaColor=colors["white"],
                physicsClientId=self.client_id,
            )
        
        # set the color of arm
        arm_num_joints = p.getNumJoints(arm_id, physicsClientId=self.client_id)
        for i in range(arm_num_joints):
            if i % 3 == 0:
                if light_color:
                    p.changeVisualShape(
                        objectUniqueId=arm_id,
                        linkIndex=i,
                        rgbaColor=colors["light_blue"],
                        physicsClientId=self.client_id
                    )
                else:
                    p.changeVisualShape(
                        objectUniqueId=arm_id,
                        linkIndex=i,
                        rgbaColor=colors["blue"],
                        physicsClientId=self.client_id
                    )
            else:
                p.changeVisualShape(
                    objectUniqueId=arm_id, 
                    linkIndex=i, 
                    rgbaColor=colors["light_white"], 
                    physicsClientId=self.client_id
                )

    def set_object_color(self, object_id, color):
        """
        Set the object color of object in the kitchen.
        """
        
        p.changeVisualShape(
            objectUniqueId=object_id,
            linkIndex=-1,
            rgbaColor=colors[color],
            physicsClientId=self.client_id
        )
        
    def set_link_color(self, object_id, link_id, color):
        """
        Set the link color of object in the kitchen.
        """
        
        p.changeVisualShape(
            objectUniqueId=object_id,
            linkIndex=link_id,
            rgbaColor=colors[color],
            physicsClientId=self.client_id
        )
    
    def set_links_color(self, object_id, link_ids, colors):
        """
        Set the links color of object in the kitchen.
        """
        
        for i, color in zip(link_ids, colors):
            p.changeVisualShape(
                objectUniqueId=object_id,
                linkIndex=i,
                rgbaColor=colors[color],
                physicsClientId=self.client_id
            )