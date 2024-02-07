# Simulation Stack
This package allows you to simulate Marvin (ARV's 2022-2024 robot) using RViz and Gazebo

## Package Dependencies
- [Velodyne Simulator](https://github.com/umigv/velodyne_simulator), which simulates our LiDAR.
- [pointcloud_to_laserscan](https://github.com/ros-perception/pointcloud_to_laserscan), which converts our LiDAR's point cloud to laser scan
- Packages installed with apt-get can be found in [setup.bash](https://github.com/umigv/simulation_stack/blob/main/scripts/setup.bash)
- If you are not using the official UMARV environment, you should have ROS2 humble installed through the tutorial [here](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)

## Requirements
### Docker Image Setup (ARV Environment)
**WARNING - this step recreates your docker container, which resets everything except files within the ```ws``` folder. Make sure your files are within ```ws``` or backed up.**
1. Open file explorer, find docker-compose.yml within your wsl folder
2. Open the file, uncomment line 16. This prevents errors when running Gazebo
3. Connect to the docker container as usual

### Performance


## Installation
1. ```cd``` into ```ws/src```
2. ```git clone https://github.com/umigv/simulation_stack.git```
3. ```cd simulation_stack```
4. ```./scripts/setup.bash```

## Running the Stack
### Building
1. ```cd``` into ```ws```
2. ```source /opt/ros/humble/setup.bash```
3. ```colcon build```
4. ```source install/setup.bash```

### Launching
1. Running ```ros2 launch marvin_simulation display.launch.py``` should show open RViz with the robot model
2. Alternatively, running ```ros2 launch marvin_simulation simulation.launch.py``` should open up the robot model with Gazebo and RViz

## Interfacing with the Stack
### Topics

### Launch File Parameters

### Simulation Stack as a Subpackage

## Sample Image
![image](https://github.com/umigv/simulation_stack/assets/71594512/d06b174b-d1e1-4ed9-87ef-9c0c3b0abce3)
![image](https://github.com/umigv/simulation_stack/assets/71594512/9130685b-c081-4591-942f-6b38e1be852f)

## Possible Issues

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
