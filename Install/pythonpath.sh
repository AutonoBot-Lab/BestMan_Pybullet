#!/bin/bash

# Add this project to the python search path
Install_dir=$(dirname "$(readlink -f "$0")")
project_dir=$(dirname "$Install_dir")
if [ -z "$PYTHONPATH" ] || ! echo "$PYTHONPATH" | grep -q "$project_dir"; then
    PYTHONPATH="\$PYTHONPATH:$project_dir"
fi

# Add AnyGrasp .so files to the python search path
Anygrasp_dir="$project_dir/Motion_Planning/Manipulation/Grasp/Grasp_Pose_Estimation/AnyGrasp"
if ! echo "$PYTHONPATH" | grep -q "$Anygrasp_dir"; then
    PYTHONPATH="$PYTHONPATH:$Anygrasp_dir"
fi

echo "export PYTHONPATH=$PYTHONPATH" >> ~/.bashrc

