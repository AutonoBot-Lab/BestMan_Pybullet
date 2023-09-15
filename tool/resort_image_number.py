# -*- coding: utf-8 -*-

"""
@Description :   convert heic to jpg
@Author      :   Yan Ding 
@Time        :   2023/09/11 22:56:52
"""

import os

def batch_rename(directory, prefix="view_5_fridge_"):
    # 获取文件夹中所有文件的列表
    files = [f for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f))]
    # 为了确保文件的排序，我们可以按文件名对文件进行排序
    files.sort()
    
    count = 1
    for file in files:
        # 获取文件的扩展名
        file_extension = os.path.splitext(file)[1]
        # 构造新的文件名
        new_name = f"{prefix}{count}{file_extension}"
        # 重命名文件
        os.rename(os.path.join(directory, file), os.path.join(directory, new_name))
        count += 1

    print(f"Renamed {count - 1} files successfully!")

# 使用方法
directory_path = "/home/yan/BestMan/BestMan_Pybullet/image/image_dataset/view_5_fridge"  # 将此路径替换为你的文件夹路径
batch_rename(directory_path)