## Marvin
ARV's robot used in IGVC 2024

## Topics
### Robot
```/cmd_vel``` (```geometry_msgs/msg/Twist```) - target velocity for the robot to move in  
```/odom``` (```nav_msgs/msg/Odometry```) - an estimation of the robot's position and velocity  

### LiDAR
```/velodyne_points``` (```sensor_msgs/msg/PointCloud2```) - pointcloud output of the LiDAR  
```/scan``` (```sensor_msgs/msg/LaserScan```) - converted laser scan of the LiDAR pointcloud on the x-y plane  

### Camera
```/zed/camera_info``` (```sensor_msgs/msg/CameraInfo```) - Information about the camera  
```/zed/depth/camera_info``` (```sensor_msgs/msg/CameraInfo```) - Information about the depth camera  
```/zed/depth/image_raw``` (```sensor_msgs/msg/Image```) - Raw depth image  
```/zed/image_raw``` (```sensor_msgs/msg/Image```) - Raw image  
```/zed/points``` (```sensor_msgs/msg/PointCloud2```) - Point cloud of the camera's depth data (warning: visualizing this in RViz is VERY computationally intensive)  

### IMU
```/imu_controller/out``` (```sensor_msgs/msg/Imu```) - IMU data  