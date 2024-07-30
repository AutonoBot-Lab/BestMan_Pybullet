from PIL import ImageDraw

def draw_rectangle(image, bbox, width=5):
    """
    Draw a bounding box on an image.
    
    Args:
        image (Image.Image): The image on which to draw the bounding box.
        bbox (list[int]): The bounding box coordinates in the format [x1, y1, x2, y2].
        width (int, optional): The width of the bounding box outline. Defaults to 5.
    
    Returns:
        ImageDraw.Draw: The ImageDraw object with the bounding box drawn.
    """
    img_drw = ImageDraw.Draw(image)
    x1, y1, x2, y2 = bbox[0], bbox[1], bbox[2], bbox[3]

    width_increase = 5
    for _ in range(width_increase):
        img_drw.rectangle([(x1, y1), (x2, y2)], outline="green")

        x1 -= 1
        y1 -= 1
        x2 += 1
        y2 += 1
    
    return img_drw