# Simulation Stack
This package allows you to simulate Marvin (ARV's 2022-2024 robot) using RViz and Gazebo


## Package Dependencies
- [Velodyne Simulator](https://github.com/umigv/velodyne_simulator), which simulates our LiDAR.
- [pointcloud_to_laserscan](https://github.com/ros-perception/pointcloud_to_laserscan), which converts our LiDAR's point cloud to laser scan
- Packages installed with apt-get can be found in [setup.bash](https://github.com/umigv/simulation_stack/blob/main/scripts/setup.bash)
- If you are not using the [official ARV environment](https://github.com/umigv/environment), you should have ROS2 humble installed through the tutorial [here](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)


## Requirements
### Docker Image Setup (ARV Environment)
**WARNING - this step recreates your docker container, which resets everything except files within the ```ws``` folder. Make sure your files are within ```ws``` or backed up.**
1. Open file explorer, find docker-compose.yml within your wsl folder
2. Open the file, uncomment line 16. This prevents errors when running Gazebo
3. Connect to the docker container as usual

### Performance
The simulation is fairly computation intensive so a GPU is recommended. Note that the ARV docker image is not able to access the computer's GPU so performance can still be low. Running Linux natively (ex. through dual booting) is recommended for optimal performance.


## Installation
1. ```cd``` into ```ws/src```
2. ```git clone https://github.com/umigv/simulation_stack.git```
3. ```cd simulation_stack```
4. ```./scripts/setup.bash```


## Running the Stack
### Building
1. ```cd``` into ```ws```
2. ```source /opt/ros/humble/setup.bash```
3. ```colcon build --symlink-install```
4. ```source install/setup.bash```

### Launching
Running ```ros2 launch marvin_simulation display.launch.py``` will open RViz and display the robot model. This launch file is made to quickly check if the robot model is correctly formatted and used internally when developing the stack.

Running ```ros2 launch marvin_simulation simulation.launch.py``` will spawn the robot in Gazebo and visualize sensor outputs in RViz. Any project interfacing with the simulation should use this launch file.


## Interfacing with the Stack
### Topics
#### Robot
```/cmd_vel``` (```geometry_msgs/msg/Twist```) - target velocity for the robot to move in  
```/odom``` (```nav_msgs/msg/Odometry```) - an estimation of the robot's position and velocity  

#### LiDAR
```/velodyne_points``` (```sensor_msgs/msg/PointCloud2```) - pointcloud output of the LiDAR  
```/scan``` (```sensor_msgs/msg/LaserScan```) - converted laser scan of the LiDAR pointcloud on the x-y plane  

#### Camera
```/zed/camera_info``` (```sensor_msgs/msg/CameraInfo```) - Information about the camera  
```/zed/depth/camera_info``` (```sensor_msgs/msg/CameraInfo```) - Information about the depth camera  
```/zed/depth/image_raw``` (```sensor_msgs/msg/Image```) - Raw depth image  
```/zed/image_raw``` (```sensor_msgs/msg/Image```) - Raw image  
```/zed/points``` (```sensor_msgs/msg/PointCloud2```) - Point cloud of the camera's depth data (warning: visualizing this in RViz is VERY computationally intensive)  

#### IMU
```/imu_controller/out``` (```sensor_msgs/msg/Imu```) - IMU data  

### Launch File Arguments
#### display.launch.py
```joint_gui``` (default: ```True```) - whether to enable joint_state_publisher_gui

#### simulation.launch.py
```headless``` (default: ```False```) - whether to enable RViz. If you have other code that runs their own instance of RViz (ex. Nav2), you should set headless to True  
```world``` (default: ```empty```) - Name of the Gazebo world file in the world directory

### Worlds
```empty``` - empty worlds  
```basic_obstacles``` - small world with basic obstacles like traffic cones and shelves  
```igvc``` - standard IGVC world with lane lines and ramps  
```igvc_flat``` - flat IGVC world with no ramps  
```igvc_wall``` - standard IGVC world with walls representing lane lines  
```igvc_flat_wall``` - flat IGVC world with walls and no ramps  

### Simulation Stack as a Subpackage
Since multiple subteams may use this stack as a dependency, **do not include this stack directly in another subteam stack either as a package or git submodule**. Having multiple instances in the same package will result in conflicts. Instead, develop with simulation without adding simulation to your repostitory, and include this project as a submodule in the main stack.

## Sample Image
![image](https://github.com/umigv/simulation_stack/assets/71594512/d06b174b-d1e1-4ed9-87ef-9c0c3b0abce3)
![image](https://github.com/umigv/simulation_stack/assets/71594512/9130685b-c081-4591-942f-6b38e1be852f)

## Learning more
Visit the [wiki](https://github.com/umigv/simulation_stack/wiki) to learn about how to set up robot models for simulation

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

[UTRA ART](https://github.com/UTRA-ART) for their [full IVGC course world](https://github.com/UTRA-ART/Caffeine/tree/master/worlds/). Any file or folder containing IGVC is under the original [Apache 2.0 license](https://github.com/umigv/simulation_stack/blob/igvc_course/marvin_simulation/world/LICENSE)
