#!/bin/bash

# 获取当前脚本的绝对路径
script_dir=$(dirname "$(readlink -f "$0")")

# 获取项目绝对路径
project_dir=$(dirname "$script_dir")

# 设置python项目搜索路径
echo "export PYTHONPATH=\$PYTHONPATH:$project_dir" >> ~/.bashrc

# 配置生效
source ~/.bashrc