import os
os.environ["HF_ENDPOINT"] = "https://hf-mirror.com"
import cv2
import copy
import numpy as np
from PIL import Image, ImageDraw
from typing import List, Type, Tuple
from lang_sam import LangSAM
from .utils import draw_rectangle


class Lang_SAM():
    
    def __init__(self):
        
        self.model = LangSAM()

    def detect_obj(
        self,
        image: Type[Image.Image],
        text: str = None,
        bbox: List[int] = None,
        save_box: bool = False,
        box_filename: str = None,
        save_mask: bool = False,
        mask_filename: str = None,
    ) -> Tuple[np.ndarray, List[int]]:
        """
        Args:
            image: An image object on which object detection is performed.
            text: Optional parameter for performing text-related object detection tasks, a object name in the scence, eg. "cup". Defaults to None.
            bbox: Optional parameter specifying an initial bounding box. Defaults to None.
            save_box: Optional parameter indicating whether to save bounding boxes. Defaults to False.
            box_filename: Optional parameter specifying the filename to save the visualization of bounding boxes. Defaults to None.
            save_mask: Optional parameter indicating whether to save masks. Defaults to False.
            mask_filename: Optional parameter specifying the filename to save the visualization of masks. Defaults to None.
            
        Returns:
            seg_mask: the segmentation mask of the detected object in the input image.
            bbox: the bounding box coordinates of the detected object in the input image
        """
        
        masks, boxes, phrases, logits = self.model.predict(image, text)
        if len(masks) == 0:
            return masks, None

        seg_mask = np.array(masks[0])
        bbox = np.array(boxes[0], dtype=int)

        if save_box:
            self.draw_bounding_box(image, bbox, box_filename)

        if save_mask:
            self.draw_mask_on_image(image, seg_mask, mask_filename)

        return seg_mask, bbox
    
    def draw_bounding_box(
        self, image: Type[Image.Image], bbox: List[int], save_file: str = None
    ) -> None:
        new_image = copy.deepcopy(image)
        draw_rectangle(new_image, bbox)

        if save_file is not None:
            new_image.save(save_file)
    
    def draw_bounding_boxes(
        self,
        image: Type[Image.Image],
        bboxes: List[int],
        scores: List[int],
        max_box_ind: int = -1,
        save_file: str = None,
    ) -> None:
        if max_box_ind != -1:
            max_score = np.max(scores.detach().numpy())
            max_ind = np.argmax(scores.detach().numpy())
        max_box = bboxes.detach().numpy()[max_ind].astype(int)

        new_image = copy.deepcopy(image)
        img_drw = ImageDraw.Draw(new_image)
        img_drw.rectangle(
            [(max_box[0], max_box[1]), (max_box[2], max_box[3])], outline="green"
        )
        img_drw.text(
            (max_box[0], max_box[1]), str(round(max_score.item(), 3)), fill="green"
        )

        for box, score, label in zip(bboxes, scores):
            box = [int(i) for i in box.tolist()]
            if score == max_score:
                img_drw.rectangle([(box[0], box[1]), (box[2], box[3])], outline="red")
                img_drw.text(
                    (box[0], box[1]), str(round(max_score.item(), 3)), fill="red"
                )
            else:
                img_drw.rectangle([(box[0], box[1]), (box[2], box[3])], outline="white")
        new_image.save(save_file)
        print(f"Saved Detection boxes at {save_file}")

    def draw_mask_on_image(
        self, image: Type[Image.Image], seg_mask: np.ndarray, save_file: str = None
    ) -> None:
        image = np.array(image)
        image[seg_mask] = image[seg_mask] * 0.2

        # overlay mask
        highlighted_color = [179, 210, 255]
        overlay_mask = np.zeros_like(image)
        overlay_mask[seg_mask] = highlighted_color

        # placing mask over image
        alpha = 0.6
        highlighted_image = cv2.addWeighted(overlay_mask, alpha, image, 1, 0)
        highlighted_image = cv2.cvtColor(highlighted_image, cv2.COLOR_RGB2BGR)

        cv2.imwrite(save_file, highlighted_image)
        print(f"Saved Segmentation Mask at {save_file}")

if __name__=='__main__':
    
    # set work dir to Lang-SAM
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    lang_sam = Lang_SAM()
    
    image = Image.open(f"./test_image/test_rgb.jpg")
    query = str(input("Enter a Object name in the image: "))
    box_filename = f"./output/object_box.jpg"
    mask_filename = f"./output/object_mask.jpg"
    
    # Object Segmentaion Mask
    seg_mask, bbox = lang_sam.detect_obj(
        image,
        query,
        save_box=True,
        save_mask=True,
        box_filename=box_filename,
        mask_filename=mask_filename
    )