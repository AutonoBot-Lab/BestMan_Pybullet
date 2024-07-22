#!/bin/bash

# Add this project to the python search path
Install_dir=$(dirname "$(readlink -f "$0")")
project_dir=$(dirname "$Install_dir")
if [ -z "$PYTHONPATH" ] || ! echo "$PYTHONPATH" | grep -q "$project_dir"; then
    PYTHONPATH="\$PYTHONPATH:$project_dir"
fi

echo "export PYTHONPATH=$PYTHONPATH" >> ~/.bashrc

