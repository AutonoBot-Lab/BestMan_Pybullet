"""
@Description :   convert heic to jpg
@Author      :   Yan Ding 
@Time        :   2023/09/11 22:56:52
"""

import os
import subprocess

def convert_heic_to_jpg(directory):
    heic_files = [f for f in os.listdir(directory) if f.lower().endswith('.heic')]
    heic_files.sort()  # 如果需要按照特定顺序排序，可以在此处调整

    for idx, heic_file in enumerate(heic_files, start=1):
        input_path = os.path.join(directory, heic_file)
        output_path = os.path.join(directory, f'view_{idx:02}.jpg')
        
        subprocess.run(["heif-convert", input_path, output_path], check=True)

if __name__ == "__main__":
    directory = "/home/yan/BestMan/BestMan_Pybullet/image/input_heic"  # 替换为你的文件夹路径
    convert_heic_to_jpg(directory)
