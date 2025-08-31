#!/bin/bash
set -e

# Dependencies
sudo apt update
sudo apt-get install ros-humble-rttest ros-humble-rclcpp-action ros-humble-gazebo-dev ros-humble-gazebo-msgs ros-humble-gazebo-plugins ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs ros-humble-joint-state-publisher-gui ros-humble-xacro

# Enforce Correct Directory
(cd "$(dirname "${BASH_SOURCE[0]}")/../"

# Submodules
git submodule update --init --recursive 

# Build
(cd ../../

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash)

# Install IGVC World Models
bash simulation_common/world/igvc_models/install_models.sh)

echo "Setup successful!"
