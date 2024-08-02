#!/bin/bash
#
# Description: 
# This is a shell script for installing ROS1 Noetic and its dependencies in Ubuntu 20.04 in China.
# It uses mirrors from Tsinghua University to speed up the download.
# The script also sets up the sudo privileges and modifies the sources.list file to update the repositories accordingly.
# It installs ROS1 Noetic and its dependencies, initializes rosdep, and verifies the installation by running roscore.
# It logs the installation progress and redirects the output to the console and logs files.
#
# Version: 1.3
# Date: 2023-06-12
# Author: Herman Ye @Realman Robotics
# License: Apache License 2.0
# Warning: This script is ONLY for ROS1 Noetic in ubuntu 20.04
# set -x
set -e



# UBUNTU CONFIGURATION BEGINS HERE

# Check if script is run as root (sudo)
if [ "$(id -u)" != "0" ]; then
    echo "This script must be run with sudo privileges. for example: sudo bash ros1_noetic_install.sh"
    read -p "Press any key to exit..."
    exit 1
fi
# Get script directory
SCRIPT_DIR=$(dirname "$0")
# Get the username of the non-root user
USERNAME=$SUDO_USER
echo "Current user is: $USERNAME"
# Save logs to files
LOG_FILE="${SCRIPT_DIR}/ros1_noetic_install.log"
ERR_FILE="${SCRIPT_DIR}/ros1_noetic_install.err"
rm -f ${LOG_FILE}
rm -f ${ERR_FILE}

# Redirect output to console and log files
exec 1> >(tee -a ${LOG_FILE} )
exec 2> >(tee -a ${ERR_FILE} >&2)

# Output log info to console
echo "ROS1 Noetic installation started!"  
echo "Installation logs will be saved to ${LOG_FILE}"
echo "Installation errors will be saved to ${ERR_FILE}"

# No Password sudo config
sudo sed -i 's/^%sudo.*/%sudo ALL=(ALL) NOPASSWD:ALL/g' /etc/sudoers

# Get architecture of the system
if [ $(uname -m) = "x86_64" ]; then
  MIRROR="https://mirrors.tuna.tsinghua.edu.cn/ubuntu/"
else
  MIRROR="https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/"
fi
echo "Current system architecture is: $(uname -m)"
echo "Current mirror is: $MIRROR"

# Backup original software sources
sudo cp /etc/apt/sources.list /etc/apt/sources.list.backup

# Clear original software sources
sudo echo "" > /etc/apt/sources.list

# Replace software sources
echo "deb $MIRROR focal main restricted universe multiverse" >> /etc/apt/sources.list
echo "deb $MIRROR focal-updates main restricted universe multiverse" >> /etc/apt/sources.list
echo "deb $MIRROR focal-backports main restricted universe multiverse" >> /etc/apt/sources.list

if [ $(uname -m) = "x86_64" ]; then
  echo "deb http://security.ubuntu.com/ubuntu/ focal-security main restricted universe multiverse" >> /etc/apt/sources.list
else
  echo "deb http://ports.ubuntu.com/ubuntu-ports/ focal-security main restricted universe multiverse" >> /etc/apt/sources.list
fi

# System update
sudo apt update
sudo apt upgrade -y

# Install pip
sudo apt install pip -y # If you haven't already installed pip

# Install gnome-terminal
sudo apt install gnome-terminal -y # If you haven't already installed gnome-terminal

# Set default pip source
pip config set global.index-url http://pypi.tuna.tsinghua.edu.cn/simple
pip config set global.trusted-host pypi.tuna.tsinghua.edu.cn

# ROS1 NOETIC INSTALLATION BEGINS HERE

# Configure your Ubuntu repositories
sudo add-apt-repository restricted
sudo add-apt-repository universe
sudo add-apt-repository multiverse

# Setup your sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Add the ROS key
ros_key="${SCRIPT_DIR}/ros.key"
rm -f "${ros_key}"
wget http://packages.ros.org/ros.key
sudo apt-key add ros.key

# Update the system packages index to the latest version
sudo apt update

# Install Curl
sudo apt install curl -y # If you haven't already installed curl

# Install ROS1 Noetic
sudo apt install ros-noetic-desktop-full -y

# Install dependencies
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y

# Environment setup
if ! grep -q "source /opt/ros/noetic/setup.bash" /home/$USERNAME/.bashrc; then

    echo "# ROS1 Noetic Environment Setting" | sudo tee -a /home/$USERNAME/.bashrc
    echo "source /opt/ros/noetic/setup.bash" | sudo tee -a /home/$USERNAME/.bashrc
    echo "ROS1 Noetic environment setup added to /home/$USERNAME/.bashrc"
else
    echo "ROS1 Noetic environment is already set in /home/$USERNAME/.bashrc"
fi
source /home/$USERNAME/.bashrc

# Initialize rosdepc by fishros under BSD License
# https://pypi.org/project/rosdepc/#files
sudo pip install rosdep
sudo pip install rosdepc
# sudo pip install -i https://pypi.tuna.tsinghua.edu.cn/simple -U rosdep
# Init & update rosdep 
sudo rosdepc init > /dev/null
#sudo rosdep fix-permissions
# su -l $USERNAME -c 'rosdepc update' > /dev/null
echo "rosdepc init completed!"

# System update again
sudo apt update
sudo apt dist-upgrade -y

# Verifying ROS1 installation
clear
# Define the variables to be printed
TEXT1="ROS1 Noetic installation completed!"
TEXT2="Please open a new terminal and run roscore to verify the installation:"
TEXT3="roscore"

# Define the colors
RED='\033[0;31m'
BLUE='\033[0;34m'
GREEN='\033[1;32m'
NC='\033[0m'

# Calculate the center of the terminal window
TERMINAL_WIDTH=$(tput cols)
TEXT1_PADDING=$((($TERMINAL_WIDTH-${#TEXT1})/2))
TEXT2_PADDING=$((($TERMINAL_WIDTH-${#TEXT2})/2))
TEXT3_PADDING=$((($TERMINAL_WIDTH-${#TEXT3})/2))

# Print the text in the center of the screen in the desired colors
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo -e "${GREEN}$(printf '%*s' $TEXT1_PADDING)${TEXT1} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT2} ${NC}"
echo -e "${RED}$(printf '%*s' $TEXT3_PADDING)${TEXT3} ${NC}"
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
