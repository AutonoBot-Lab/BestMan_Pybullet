import os
import numpy as np
from PIL import Image

from Visualization import CameraParameters
from Grasp_Pose_Estimation import AnyGrasp
from Perception.Object_detection import Lang_SAM
from utils import get_3d_points

class Grasp:

    def __init__(self, cfgs):
        
        self.cfgs = cfgs
        self.lang_sam = Lang_SAM()
        self.anygrasp = AnyGrasp(self.cfgs)
    
    
    def receive_input(self, tries):
        
        fx, fy, cx, cy = 306, 306, 118, 211
        scale = 0.001   # convert the original depth value in the depth image to the actual depth value (usually in meters)
        head_tilt = -45
        data_dir = "./test_data/"
        colors = np.array(Image.open(os.path.join(data_dir, "test_rgb.png")))
        image = Image.open(os.path.join(data_dir, "test_rgb.png"))
        depths = np.array(Image.open(os.path.join(data_dir, "test_depth.png")))
        # if tries == 1:
        #     self.action = str(input("Enter action [pick/place]: "))
            # self.query = str(input("Enter a Object name in the scence: "))
        depths = depths * scale

        colors = colors / 255.0     # Convert the color values ​​of the image from integer pixel values ​​(0-255) to floating point values ​​(0.0-1.0).
        head_tilt = head_tilt / 100
        self.cam = CameraParameters(fx, fy, cx, cy, head_tilt, image, colors, depths)
    
    
    def grasp(self, object_name):
        """Specify an object to grasp

        Args:
            object_name(int): grasp object name
        
        Returns:
            grasp_pose(Pose): Grasping pose
        """
        
        tries = 1
        retry = True
        while retry and tries <= 11:
            
            self.receive_input(tries)

            # Output Dir
            self.save_dir = os.path.join(self.cfgs.environment, object_name)
            if not os.path.exists(self.save_dir):
                os.makedirs(self.save_dir)

            box_filename = os.path.join(self.save_dir, f"object_box_{tries}.jpg")
            mask_filename = os.path.join(self.save_dir, f"object_mask_{tries}.jpg")
            
            # Object Segmentaion Mask
            seg_mask, bbox = self.lang_sam.detect_obj(
                self.cam.image,
                object_name,
                save_box=True,
                save_mask=True,
                box_filename=box_filename,
                mask_filename=mask_filename,
            )

            if bbox is None:
                print(
                    "Didnt find the Object. Try with another object or tune grasper height and width parameters in demo.py"
                )
                retry = False
                continue
            
            # bbox_x_min, bbox_y_min, bbox_x_max, bbox_y_max = bbox
            
            points = get_3d_points(self.cam)    # Convert depth image to 3D point cloud
            flag, grasp_pose = not self.anygrasp.Grasp_Pose_Estimation(self.cam, points, seg_mask, bbox, (tries == 11))
            retry = not flag
            if retry:
                if tries <= 11:
                    print("Trying Again")
                    tries = tries + 1
                else:
                    print(
                        "Try with another object or tune grasper height and width parameters in demo.py"
                    )
                    retry = False
                    return None
            else:
                return grasp_pose
                    
                