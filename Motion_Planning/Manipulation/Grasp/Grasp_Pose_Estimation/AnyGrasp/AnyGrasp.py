import os
import copy
import math
import numpy as np
from PIL import Image

from gsnet import AnyGrasp
from graspnetAPI import GraspGroup

from utils.camera import CameraParameters
from utils.types import Bbox
from utils.utils import (
    # get_3d_points,
    visualize_cloud_geometries,
    sample_points,
    draw_rectangle
)
from RoboticsToolBox import Pose


class AnyGrasp:
    
    def __init__(self, cfgs):
        self.cfgs = cfgs
        self.grasping_model = AnyGrasp(self.cfgs)
        self.grasping_model.load_net()
       
     
    # def receive_input(self, tries):
        
    #     fx, fy, cx, cy = 306, 306, 118, 211
    #     scale = 0.001   # convert the original depth value in the depth image to the actual depth value (usually in meters)
    #     head_tilt = -45
    #     data_dir = "./test_data/"
    #     colors = np.array(Image.open(os.path.join(data_dir, "test_rgb.png")))
    #     image = Image.open(os.path.join(data_dir, "test_rgb.png"))
    #     depths = np.array(Image.open(os.path.join(data_dir, "test_depth.png")))
    #     if tries == 1:
    #         self.action = str(input("Enter action [pick/place]: "))
    #         self.query = str(input("Enter a Object name in the scence: "))
    #     depths = depths * scale

    #     colors = colors / 255.0     # Convert the color values ​​of the image from integer pixel values ​​(0-255) to floating point values ​​(0.0-1.0).
    #     head_tilt = head_tilt / 100
    #     self.cam = CameraParameters(fx, fy, cx, cy, head_tilt, image, colors, depths)
    
    
    def Grasp_Pose_Estimation(
        self,
        cam: CameraParameters,
        points: np.ndarray,
        seg_mask: np.ndarray,
        bbox: Bbox,
        crop_flag: bool = False
    ):
        """Calculate the optimal grasping pose

        Args:
            cam (CameraParameters): Camera internal parameters
            points (np.ndarray): 3D point cloud data
            seg_mask (np.ndarray): Object Mask
            bbox (Bbox): Object bbox
            crop_flag (bool, optional): Crop flag. Defaults to False.

        Returns:
            grasp_pose(Pose): best grasp pose
        """
        
        self.cam = cam
        
        image = copy.deepcopy(self.cam.image)
        img_drw = draw_rectangle(image, bbox)
        projections_file_name = (
            self.cfgs.environment + "/" + self.query + "/grasp_projections.jpg"
        )
        image.save(projections_file_name)
        print(f"Saved projections of grasps at {projections_file_name}")
        
        # extract x, y, z coordinates from 3D point cloud
        points_x, points_y, points_z = points[:, :, 0], points[:, :, 1], points[:, :, 2]

        # Filter the point cloud based on depth, keeping points within the specified depth range
        mask = (points_z > self.cfgs.min_depth) & (points_z < self.cfgs.max_depth)
        points = np.stack([points_x, -points_y, points_z], axis=-1)
        points = points[mask].astype(np.float32)
        colors_m = self.cam.colors[mask].astype(np.float32)

        # If the sampling rate is less than 1, the point cloud is downsampled
        if self.cfgs.sampling_rate < 1:
            points, indices = sample_points(points, self.cfgs.sampling_rate)
            colors_m = colors_m[indices]

        # Grasp prediction, return grasp group and point cloud
        # gg is a list of grasps of type graspgroup in graspnetAPI
        xmin = points[:, 0].min()
        xmax = points[:, 0].max()
        ymin = points[:, 1].min()
        ymax = points[:, 1].max()
        zmin = points[:, 2].min()
        zmax = points[:, 2].max()
        lims = [xmin, xmax, ymin, ymax, zmin, zmax]
        gg, cloud = self.grasping_model.get_grasp(points, colors_m, lims)

        if len(gg) == 0:
            print("No Grasp detected after collision detection!")
            return False, None

        # The grasped groups are subjected to non-maximum suppression (NMS) and sorted by scores.
        gg = gg.nms().sort_by_score()
        filter_gg = GraspGroup()

        # Filtering the grasps by penalising the vertical grasps as they are not robust to calibration errors.
        bbox_x_min, bbox_y_min, bbox_x_max, bbox_y_max = bbox
        W, H = self.cam.image.size
        ref_vec = np.array(
            [0, math.cos(self.cam.head_tilt), -math.sin(self.cam.head_tilt)]
        )
        min_score, max_score = 1, -10

        for g in gg:
            grasp_center = g.translation
            ix, iy = (
                int(((grasp_center[0] * self.cam.fx) / grasp_center[2]) + self.cam.cx),
                int(((-grasp_center[1] * self.cam.fy) / grasp_center[2]) + self.cam.cy),
            )
            if ix < 0:
                ix = 0
            if iy < 0:
                iy = 0
            if ix >= W:
                ix = W - 1
            if iy >= H:
                iy = H - 1
            rotation_matrix = g.rotation_matrix
            cur_vec = rotation_matrix[:, 0]
            angle = math.acos(np.dot(ref_vec, cur_vec) / (np.linalg.norm(cur_vec)))
            if not crop_flag:
                score = g.score - 0.1 * (angle) ** 4
            else:
                score = g.score

            if not crop_flag:
                if seg_mask[iy, ix]:
                    img_drw.ellipse([(ix - 2, iy - 2), (ix + 2, iy + 2)], fill="green")
                    if g.score >= 0.095:
                        g.score = score
                    min_score = min(min_score, g.score)
                    max_score = max(max_score, g.score)
                    filter_gg.add(g)
                else:
                    img_drw.ellipse([(ix - 2, iy - 2), (ix + 2, iy + 2)], fill="red")
            else:
                g.score = score
                filter_gg.add(g)

        if len(filter_gg) == 0:
            print(
                "No grasp poses detected for this object try to move the object a little and try again"
            )
            return False, None
        
        filter_gg = filter_gg.nms().sort_by_score()

        if self.cfgs.debug:
            trans_mat = np.array(
                [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
            )
            cloud.transform(trans_mat)
            grippers = gg.to_open3d_geometry_list()
            filter_grippers = filter_gg.to_open3d_geometry_list()
            for gripper in grippers:
                gripper.transform(trans_mat)
            for gripper in filter_grippers:
                gripper.transform(trans_mat)

            visualize_cloud_geometries(
                cloud,
                grippers,
                visualize=not self.cfgs.headless,
                save_file=f"{self.cfgs.environment}/{self.query}/anygrasp/poses.jpg",
            )
            visualize_cloud_geometries(
                cloud,
                [filter_grippers[0].paint_uniform_color([1.0, 0.0, 0.0])],
                visualize=not self.cfgs.headless,
                save_file=f"{self.cfgs.environment}/{self.query}/anygrasp/best_pose.jpg",
            )

        # if self.cfgs.open_communication:
        #     data_msg = "Now you received the gripper pose, good luck."
        #     self.socket.send_data(
        #         [
        #             filter_gg[0].translation,       # grasp position
        #             filter_gg[0].rotation_matrix,   # grasp orientation
        #             [filter_gg[0].depth, crop_flag, 0],
        #             data_msg,
        #         ]
        #     )
        
        # grasp_result = [
        #     filter_gg[0].translation,       # grasp position
        #     filter_gg[0].rotation_matrix,   # grasp orientation
        # ]
        grasp_pose = Pose(filter_gg[0].translation, filter_gg[0].rotation_matrix)
        return True, grasp_pose