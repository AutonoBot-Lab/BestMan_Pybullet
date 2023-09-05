"""
@Description :   Extract object names
@Author      :   Yan Ding 
@Time        :   2023/09/05 02:21:54
"""

import os

def list_subdirectories(path):
    """列出给定路径下的所有子文件夹"""
    with open('object_names.txt', 'a') as f:
        # os.walk返回的是一个三元组 (dirpath, dirnames, filenames)
        for dirpath, dirnames, filenames in os.walk(path):
            # 只需要从根目录取子文件夹即可，所以使用break提前跳出循环
            for dirname in dirnames:
                f.write(dirname + '\n')
            break  # 保证只列出根目录下的子文件夹，而不是递归列出所有子文件夹

def filter_duplicates(filename):
    """读取文件，过滤掉重复行，并重新保存"""
    with open(filename, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    # 使用set来过滤重复行
    unique_lines = set(lines)

    with open(filename, 'w', encoding='utf-8') as f:
        for line in sorted(unique_lines):  # sorted用于对结果排序，可以根据需要删除
            f.write(line)

if __name__ == "__main__":
    directory_path = '/home/yan/BestMan/BestMan_Pybullet/Kitchen_models/models'  # 这里修改为你的文件夹A的实际路径
    list_subdirectories(directory_path)
    filter_duplicates('object_names.txt')
