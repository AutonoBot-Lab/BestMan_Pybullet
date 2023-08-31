# -*- coding: utf-8 -*-
import ast

def write_functions_to_txt(input_file, output_file):
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
        for line in infile:
            if line.strip().startswith('def '):
                outfile.write(line)

# 使用示例
input_file = './utils/utils_Bestman.py'  # 输入你的python文件名
output_file = 'functions.txt'
write_functions_to_txt(input_file, output_file)