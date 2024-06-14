import cv2
import numpy as np

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

def label_target_on_rgb_by_segmentation(
        rgb_img,
        seg_img,
        height=224,
        width=224,
        target_object_id=4,
        file_output_path="./outputs/target_rectangle.png",
    ):
        """
        Draws a rectangular box around the target object in the RGB image based on segmentation.

        Args:
            rgb_img (np.ndarray): The RGB image.
            seg_img (np.ndarray): The segmentation image.
            target_object_id (int): The ID of the target object in the segmentation image.
            file_output_path (str, optional): The path to save the output image. Default is './outputs/target_rectangle.png'.

        Returns:
            np.ndarray: The RGB image with a rectangular box drawn around the target object.
        """

        # get the target object's bounding box
        target_object_mask = np.where(seg_img == target_object_id, 255, 0).astype(
            np.uint8
        )
        contours, _ = cv2.findContours(
            target_object_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )
        # if the target object is found, draw a rectangular box around it
        if len(contours) != 0:
            x, y, w, h = cv2.boundingRect(contours[0])
            # draw a rectangular box around the target object
            cv2.rectangle(rgb_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # transfer the image to RGB from BGR
        rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2RGB)
        # save the image
        cv2.imwrite(file_output_path, rgb_img)
        return rgb_img