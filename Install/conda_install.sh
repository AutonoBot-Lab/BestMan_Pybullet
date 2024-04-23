#!/bin/bash

# Add this project to the python search path
script_dir=$(dirname "$(readlink -f "$0")")
project_dir=$(dirname "$script_dir")
if [ -z "$PYTHONPATH" ] || ! echo "$PYTHONPATH" | grep -q "$project_dir"; then
    # If PYTHONPATH doesn't exist, set it to project directory
    echo "export PYTHONPATH=\$PYTHONPATH:$project_dir" >> ~/.bashrc
fi

# Install shared file
# sudo apt update && sudo apt install libgl1-mesa-glx libglib2.0-0

# Configure x11 forwarding service
# echo "export DISPLAY=host.docker.internal:0" >> ~/.bashrc

source ~/.bashrc
