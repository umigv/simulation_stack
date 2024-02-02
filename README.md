# Simulation Stack
This package allows you to simulate Marvin (UMARV's 2022-2024 robot) using RViz and Gazebo

## URDF basics
Unified Robotics Description Format (URDF) is an XML specification used to model robots. It is made up of ```links``` (bodies with kinematic and dynamic specifications) and ```joints``` (connection between links). You can imagine the model structure to be a [tree](https://en.wikipedia.org/wiki/Tree_(data_structure)) with links as nodes and joints as edges. The root, or fixed frame (in our case ```base_link```, specified in the rviz configs) is the core of the model. All other links are define relative to the fixed frame. You can read more about the URDF format [here](https://wiki.ros.org/urdf/XML) and [here](https://navigation.ros.org/setup_guides/urdf/setup_urdf.html#urdf-and-the-robot-state-publisher). I also recommend watching [this video](https://youtu.be/CwdbsvcpOHM?si=mOkKDYqQnHFhNE2T) as it is a good introduction to URDF.

## Package Dependencies
- [Velodyne Simulator](https://github.com/ToyotaResearchInstitute/velodyne_simulator), which simulates our LiDAR.
- [pointcloud_to_laserscan](https://github.com/ros-perception/pointcloud_to_laserscan), which helps convert our LiDAR's point cloud data to laser scan which can then be used for SLAM.
- If you are not using the official UMARV environment, you should have ROS2 installed by following the tutorial [here](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)

## Requirements
### Docker image setup (umarv environment)
**WARNING - this step recreates your docker container, which resets everything except files within the ```ws``` folder. Make sure your files are within ```ws``` or backed up.**
1. Open file explorer, find docker-compose.yml within your wsl folder
2. Open the file, uncomment line 16. This prevents errors when running Gazebo
3. Connect to the docker container as usual

## Installation Guide

### Install Simulation Stack
1. ```cd``` into ```ws/src```
2. ```git clone https://github.com/umigv/simulation_stack.git```
3. ```cd simulation_stack```
4. ```./scripts/SetupSimulation.bash```

## Testing the Project
1. ```cd``` into ```ws```
2. ```source /opt/ros/humble/setup.bash```
3. ```colcon build```
4. ```source install/setup.bash```
5. Running ```ros2 launch marvin_simulation display.launch.py``` should show open RViz with the robot model
6. Alternatively, running ```ros2 launch marvin_simulation simulation.launch.py``` should open up the robot model with Gazebo and RViz

## Sample Image
![image](https://github.com/umigv/nav_stack/assets/71594512/cde0a60f-b5a3-47b7-b05a-c7afba1f751d)
![image](https://github.com/umigv/nav_stack/assets/71594512/0ef3b50e-5b1a-42f2-a5a8-bbf3d5d2e234)

## Possible Issues
### ros2: command not found
Run ```source /opt/ros/humble/setup.bash```

### symbol lookup error: ... undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE
This error is caused by the VSCode snap package setting GTK_PATH, which is then used by the VSCode integrated terminal. There are two solutions:
- run ``` unset GTK_PATH ```
- Build and run the project using an external terminal

### GTK+ module libcanberra-gtk-module.so cannot be loaded  
This error is caused by [snap variables leaking into terminal variables](https://github.com/microsoft/vscode/issues/179086). There are two solutions:
- (If linux) Redownload VSCode using the debian package [here](https://code.visualstudio.com/download)
- Build and run the project using an external terminal

## Credits
Jason Ning and Kari Naga on the sensors team, who created the original URDF files and the Gazebo World in the [marvin](https://github.com/umigv/marvin/tree/main/urdf) repository.  
Ethan Hardy for testing the package and creating the installation script
