#!/bin/bash

# Add this project to the python search path
script_dir=$(dirname "$(readlink -f "$0")")
project_dir=$(dirname "$script_dir")
echo "export PYTHONPATH=\$PYTHONPATH:$project_dir" >> ~/.bashrc

# Install shared file
sudo apt update && sudo apt install libgl1-mesa-glx libglib2.0-0

# Configure x11 forwarding service
echo "export DISPLAY=host.docker.internal:0" >> ~/.bashrc

source ~/.bashrc
