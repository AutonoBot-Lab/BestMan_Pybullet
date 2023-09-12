"""
@Description :   convert heic to jpg
@Author      :   Yan Ding 
@Time        :   2023/09/11 22:56:52
"""

import os
import re

def rename_view_images(directory):
    # 查找以 "view" 开头的所有文件
    view_files = [f for f in os.listdir(directory) if re.match(r'view_.*', f)]
    view_files.sort()  # 如果需要按照特定顺序排序，可以在此处调整

    for idx, view_file in enumerate(view_files, start=1):
        old_path = os.path.join(directory, view_file)
        new_path = os.path.join(directory, f'view_{idx}.jpg')  # 假设都是 .jpg 格式，如果有其他格式，请进行调整
        os.rename(old_path, new_path)

if __name__ == "__main__":
    directory = "/home/yan/Downloads/input/case6"  # 替换为你的文件夹路径
    rename_view_images(directory)