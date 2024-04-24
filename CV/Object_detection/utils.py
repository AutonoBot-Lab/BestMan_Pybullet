from PIL import ImageDraw

def draw_rectangle(image, bbox, width=5):
    """
    draw a bounding box on an image
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