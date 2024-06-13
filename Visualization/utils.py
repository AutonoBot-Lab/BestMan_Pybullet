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