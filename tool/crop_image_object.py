import cv2
import os
from PIL import Image

def capture_and_enlarge(image_path, roi):
    """
    对图片进行截图并放大到原始大小
    
    参数:
    - image_path: 图片路径
    - roi: 感兴趣的区域，格式为 (x1, y1, x2, y2)

    返回:
    - enlarged_image: 放大后的图片
    """
    image = cv2.imread(image_path)
    
    # 获取原始图片的宽和高
    original_height, original_width = image.shape[:2]
    # print(f"original_height:{original_height} original_width:{original_width}")
    
    # 截图
    x1, y1, x2, y2 = roi
    cropped_image = image[y1:y2, x1:x2]
    
    # 放大到原始大小
    enlarged_image = cv2.resize(cropped_image, (original_width, original_height))
    
    return enlarged_image

# 输入和输出文件夹
input_folder = '/home/yan/BestMan/BestMan_Pybullet/image/input_jpg/point3'
output_folder = '/home/yan/BestMan/BestMan_Pybullet/image/input_jpg/point3_microwave'
if not os.path.exists(output_folder):
    os.makedirs(output_folder)  # 如果输出文件夹不存在，就创建它

"""
point1_type1: table
"""
x1=18.227890014648438
y1=1302.2623291015625
x2=3743.08642578125
y2=3024.0
expand_ratio = 0.2

"""
point1_type1: object on table
"""
x1=2129.544921875
y1=1834.2882080078125
x2=2676.904541015625
y2=2078.733154296875
expand_ratio = 1

"""
point1_type2: table
"""
x1=0.0
y1=996.259521484375
x2=3578.68017578125
y2=3024.0
expand_ratio = 0.2

"""
point1_type2: object
"""
x1=2300
y1=1357.404052734375
x2=1452.4952392578125
y2=1535.7783203125
expand_ratio = 5

"""
point2_cabinet:
"""
x1=2176.248046875
y1=83.08363342285156
x2=2875.90380859375
y2=1473.243896484375
expand_ratio = 0.2

"""
point2_microwave:
"""
x1=1293
y1=1584
x2=2855
y2=3125
expand_ratio = 0.2

"""
point2: microwave (open) + cabinet (closed)
"""

"""
point2: microwave (open) + cabinet (open)
"""

x1 = int(x1)
y1 = int(y1)
x2 = int(x2)
y2 = int(y2)

# 扩大bounding box的大小
box_width = x2 - x1
box_height = y2 - y1

width = 4032
height = 3024

# 首先扩大 bounding box
x1_expanded = max(0, x1 - int(expand_ratio * box_width))
y1_expanded = max(0, y1 - int(expand_ratio * box_height))
x2_expanded = min(width, x2 + int(expand_ratio * box_width))
y2_expanded = min(height, y2 + int(expand_ratio * box_height))

expanded_box_width = x2_expanded - x1_expanded
expanded_box_height = y2_expanded - y1_expanded

# 检查扩大后的bounding box宽高比
ratio = expanded_box_width / expanded_box_height

# 如果当前的宽高比不等于 1.333，那么进行调整
target_ratio = 4/3

if ratio < target_ratio:
    # 增加宽度
    difference = expanded_box_height * target_ratio - expanded_box_width
    x1_expanded = max(0, x1_expanded - int(difference / 2))
    x2_expanded = min(width, x2_expanded + int(difference / 2))
else:
    # 增加高度
    difference = expanded_box_width / target_ratio - expanded_box_height
    y1_expanded = max(0, y1_expanded - int(difference / 2))
    y2_expanded = min(height, y2_expanded + int(difference / 2))

roi = (x1_expanded, y1_expanded, x2_expanded, y2_expanded)
print(f'ratio: {target_ratio} x: {roi[2]-roi[0]} y: {roi[3]-roi[1]}')
# sys.exit()

# 遍历输入文件夹中的所有图片
for image_file in os.listdir(input_folder):
    if image_file.lower().endswith(('.png', '.jpg', '.jpeg')):
        input_image_path = os.path.join(input_folder, image_file)
        output_image_path = os.path.join(output_folder, image_file)
        
        # 裁剪图片
        result_image = capture_and_enlarge(input_image_path, roi)
        
        # 保存裁剪后的图片到新的文件夹
        cv2.imwrite(output_image_path, result_image)