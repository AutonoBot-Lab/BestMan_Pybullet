#!/bin/bash

# Add this project to the python search path
script_dir=$(dirname "$(readlink -f "$0")")
project_dir=$(dirname "$script_dir")
if [ -z "$PYTHONPATH" ] || ! echo "$PYTHONPATH" | grep -q "$project_dir"; then
    # If PYTHONPATH doesn't exist, set it to project directory
    echo "export PYTHONPATH=\$PYTHONPATH:$project_dir" >> ~/.bashrc
fi

# Install shared file
sudo apt update && sudo apt install -y libgl1-mesa-glx libglib2.0-0
sudo mkdir /usr/lib/dri
cd /usr/lib/dri
sudo ln -s /lib/x86_64-linux-gnu/dri/swrast_dri.so swrast_dri.so