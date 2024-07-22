import os
import argparse
import copy
import math
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
from gsnet import AnyGrasp
from graspnetAPI import GraspGroup
from utils import (
    Bbox,
    get_3d_points,
    visualize_cloud_geometries,
    sample_points,
    draw_rectangle
)
from RoboticsToolBox import Pose
from Visualization import CameraParameters
from Perception.Object_detection import Lang_SAM

class Anygrasp:
    
    def __init__(self, cfgs):
        self.cfgs = cfgs
        self.grasping_model = AnyGrasp(self.cfgs)
        self.grasping_model.load_net()   
    
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

        # gg is a list of grasps of type graspgroup in graspnetAPI
        xmin = points[:, 0].min()
        xmax = points[:, 0].max()
        ymin = points[:, 1].min()
        ymax = points[:, 1].max()
        zmin = points[:, 2].min()
        zmax = points[:, 2].max()
        lims = [xmin, xmax, ymin, ymax, zmin, zmax]
        # Grasp prediction, return grasp group and point cloud
        gg, cloud = self.grasping_model.get_grasp(points, colors_m, lims, apply_object_mask=True, dense_grasp=False, collision_detection=True)

        if len(gg) == 0:
            print("[AnyGrasp] No Grasp detected after collision detection!")
            return False, None

        # The grasped groups are subjected to non-maximum suppression (NMS) and sorted by scores.
        gg = gg.nms().sort_by_score()
        print("[AnyGrasp] Grasp point nums", len(gg))

        filter_gg = GraspGroup()

        # Filtering the grasps by penalising the vertical grasps as they are not robust to calibration errors.
        W, H = self.cam.image.size

        # Reference direction vector, indicating the ideal grasping direction.
        ref_vec = np.array(
            [0, math.cos(self.cam.head_tilt), -math.sin(self.cam.head_tilt)]
        )
        min_score, max_score = 1, -10
        image = copy.deepcopy(self.cam.image)
        img_drw = draw_rectangle(image, bbox)

        for g in gg:

            grasp_center = g.translation
            # Convert the coordinates of a grasp center in three-dimensional space to two-dimensional coordinates on the image plane
            ix, iy = (
                int(((grasp_center[0] * self.cam.fx) / grasp_center[2]) + self.cam.cx),
                int(((-grasp_center[1] * self.cam.fy) / grasp_center[2]) + self.cam.cy),
            )
            if ix < 0:  ix = 0
            if iy < 0:  iy = 0
            if ix >= W: ix = W - 1
            if iy >= H: iy = H - 1
            
            # 3 * 3 rotation matrix
            rotation_matrix = g.rotation_matrix
            # x-axis direction, forward direction
            cur_vec = rotation_matrix[:, 0]
            # The angle between two vectors
            angle = math.acos(np.dot(ref_vec, cur_vec) / (np.linalg.norm(cur_vec)))
            if not crop_flag:
                score = g.score - 0.1 * (angle) ** 4
            else:
                score = g.score

            if not crop_flag:
                # Check if the grasp point is within the object area
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
                "[AnyGrasp] No grasp poses detected for this object try to move the object a little and try again"
            )
            return False, None

        projections_file_name = (os.path.join(self.cfgs.output_dir, "grasp_projections.png"))
        plt.imshow(image)
        plt.axis('off')
        plt.show()
        
        image.save(projections_file_name)
        print(f"[AnyGrasp] Saved projections of grasps at {projections_file_name}")

        filter_gg = filter_gg.nms().sort_by_score()
        print("[AnyGrasp] Filter grasp point nums", len(filter_gg))

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
                save_file=os.path.join(self.cfgs.output_dir, "poses.png"),
            )

            visualize_cloud_geometries(
                cloud,
                [filter_grippers[0].paint_uniform_color([1.0, 0.0, 0.0])],
                visualize=not self.cfgs.headless,
                save_file=os.path.join(self.cfgs.output_dir, "best_pose.png"),
            )
        
        # grasp_result = [
        #     filter_gg[0].translation,       # grasp position
        #     filter_gg[0].rotation_matrix,   # grasp orientation
        # ]
        grasp_pose = Pose(filter_gg[0].translation, filter_gg[0].rotation_matrix)
        return True, grasp_pose


if __name__=='__main__':

    # set work dir to AnyGrasp
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # cfgs
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint_path", default="./checkpoints/checkpoint_detection.tar", help="Model checkpoint path")
    parser.add_argument("--max_gripper_width", type=float, default=0.1, help="Maximum gripper width (<=0.1m)")
    parser.add_argument("--gripper_height", type=float, default=0.03, help="Gripper height")
    parser.add_argument("--top_down_grasp", action="store_true", help="Output top-down grasps")
    parser.add_argument("--debug", action="store_true", help="Enable visualization")
    parser.add_argument("--headless", action="store_true", help="Enable headless mode, don't save visualization result")
    parser.add_argument("--max_depth", type=float, default=1.0, help="Maximum depth of point cloud")
    parser.add_argument("--min_depth", type=float, default=0, help="Maximum depth of point cloud")
    parser.add_argument("--sampling_rate", type=float, default=1.0, help="Sampling rate of points [<= 1]")
    parser.add_argument("--input_dir", default="./test_data/example2", help="Input data dir")
    parser.add_argument("--output_dir", default="./output/example2", help="Output data dir")
    parser.add_argument("--fx", type=float, default=306, help="Focal length in the x direction")
    parser.add_argument("--fy", type=float, default=306, help="Focal length in the y direction")
    parser.add_argument("--cx", type=float, default=118, help="Center of the optical axis in the x-direction")
    parser.add_argument("--cy", type=float, default=211, help="Center of the optical axis in the y-direction")
    parser.add_argument("--scale", type=float, default=0.001, help="Convert the original depth value in the depth image to the actual depth value")
    parser.add_argument("--head_tilt", type=float, default=-0.45, help="Tilt angle for ideal gripping pose")
    cfgs = parser.parse_args()
    cfgs.max_gripper_width = max(0, min(0.2, cfgs.max_gripper_width))

    if not os.path.exists(cfgs.output_dir):
        os.mkdir(cfgs.output_dir)
    
    # camera parameters
    colors = np.array(Image.open(os.path.join(cfgs.input_dir, "color.png")), dtype=np.float32) / 255.0
    image = Image.open(os.path.join(cfgs.input_dir, "color.png"))
    depths = np.array(Image.open(os.path.join(cfgs.input_dir, "depth.png"))) * cfgs.scale
    cam = CameraParameters(cfgs.fx, cfgs.fy, cfgs.cx, cfgs.cy, cfgs.head_tilt, image, colors, depths)
    
    # object detection
    lang_sam = Lang_SAM()
    image = Image.open(os.path.join(cfgs.input_dir, "color.png"))
    query = str(input("Enter a Object name in the image: "))
    seg_mask, bbox = lang_sam.detect_obj(
        image,
        query,
        save_box=False,
        save_mask=False
    )

    # get 3d points
    points = get_3d_points(cam)

    anygrasp = Anygrasp(cfgs)
    best_pose = anygrasp.Grasp_Pose_Estimation(cam, points, seg_mask, bbox)

