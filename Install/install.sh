#!/bin/bash

# Add this project to the python search path
script_dir=$(dirname "$(readlink -f "$0")")
project_dir=$(dirname "$script_dir")
echo "export PYTHONPATH=\$PYTHONPATH:$project_dir" >> ~/.bashrc
source ~/.bashrc

# Install shared file
sudo apt install libgl1-mesa-glx
sudo apt install libglib2.0-0

# Configure x11 forwarding service
echo "export DISPLAY=host.docker.internal:0" >> ~/.bashrc

# Create conda enviroment
conda env create -f requirements.yaml
pip install ompl-1.6.0-cp38-cp38-manylinux_2_28_x86_64.whl