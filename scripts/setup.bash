#!/bin/bash
set -e

# Dependencies
sudo apt update
sudo apt-get install ros-humble-rttest ros-humble-rclcpp-action ros-humble-gazebo-dev ros-humble-gazebo-msgs ros-humble-gazebo-plugins ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs ros-humble-joint-state-publisher-gui ros-humble-xacro

# Submodules
git submodule update --init --recursive 

# Build
(cd ../../

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash)

# Install IVGC World Models
bash marvin_simulation/world/ivgc_models/install_models.sh

echo "Setup successful!"
