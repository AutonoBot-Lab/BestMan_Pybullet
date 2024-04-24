import numpy as np
from PIL import Image
from typing import List, Type, Tuple
# from image_processors.image_processor import ImageProcessor
from lang_sam import LangSAM
from ..object_detection import object_detection


class LangSAM(object_detection):
    
    def __init__(self):

        self.model = LangSAM()

    def detect_obj(
        self,
        image: Type[Image.Image],
        text: str = None,
        bbox: List[int] = None,
        visualize_box: bool = False,
        box_filename: str = None,
        visualize_mask: bool = False,
        mask_filename: str = None,
    ) -> Tuple[np.ndarray, List[int]]:
        """
            image: An image object on which object detection is performed.
            text: Optional parameter for performing text-related object detection tasks, a object name in the scence, eg. "cup". Defaults to None.
            bbox: Optional parameter specifying an initial bounding box. Defaults to None.
            visualize_box: Optional parameter indicating whether to visualize bounding boxes. Defaults to False.
            box_filename: Optional parameter specifying the filename to save the visualization of bounding boxes. Defaults to None.
            visualize_mask: Optional parameter indicating whether to visualize masks. Defaults to False.
            mask_filename: Optional parameter specifying the filename to save the visualization of masks. Defaults to None.
        """
        
        masks, boxes, phrases, logits = self.model.predict(image, text)
        if len(masks) == 0:
            return masks, None

        seg_mask = np.array(masks[0])
        bbox = np.array(boxes[0], dtype=int)

        if visualize_box:
            self.draw_bounding_box(image, bbox, box_filename)

        if visualize_mask:
            self.draw_mask_on_image(image, seg_mask, mask_filename)

        return seg_mask, bbox
    

if __name__=='__main__':
    
    lang_sam = LangSAM()
    
    image = Image.open(f"../test_image/test_rgb.png")
    query = str(input("Enter a Object name in the image: "))
    box_filename = f"../output/box.jpg"
    mask_filename = f"../output/mask.jpg"
    
    # Object Segmentaion Mask
    seg_mask, bbox = lang_sam.detect_obj(
        image,
        query,
        visualize_box=True,
        visualize_mask=True,
        box_filename=box_filename,
        mask_filename=mask_filename
    )