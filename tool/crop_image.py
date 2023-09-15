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
    
    # 截图
    x1, y1, x2, y2 = roi
    cropped_image = image[y1:y2, x1:x2]
    
    # 放大到原始大小
    enlarged_image = cv2.resize(cropped_image, (original_width, original_height))
    
    return enlarged_image

# 输入和输出文件夹
input_folder = '/home/yan/BestMan/BestMan_Pybullet/image/input_jpg/input_jpg_point2'
output_folder = '/home/yan/BestMan/BestMan_Pybullet/image/input_jpg/input_jpg_point2_crop2'
if not os.path.exists(output_folder):
    os.makedirs(output_folder)  # 如果输出文件夹不存在，就创建它

# roi = (1200, 900, 3024, 2268) # input_jpg_point1_crop1
# roi = (900, 1125, 3000, 2700) # input_jpg_point2_crop1
roi = (2000, 0, 3820, 1400) # input_jpg_point2_crop2
# roi = (1000, 100, 2700, 1375) # input_jpg_point3_crop1
# roi = (2000, 1700, 3200, 2600) # input_jpg_point3_crop2
# roi = (400, 1150, 2300, 2575) # input_jpg_point3_crop2

ratio = abs(roi[0] - roi[2]) / abs(roi[1] - roi[3]) 
print(f'ratio:{ratio} x:{abs(roi[0] - roi[2])} y:{abs(roi[1] - roi[3])}')
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