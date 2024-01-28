import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition


from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Constants
    pkg_name = 'marvin_simulation'
    model = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'marvin.xacro') 
    world = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'course.world')
    launch = os.path.join(get_package_share_directory(pkg_name), 'launch')
    robot_description = xacro.process_file(model).toxml()

    # Robot
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}]
    )

    # Gazebo
    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world,
        ],
        output='screen',
        cwd=[launch]
    )

    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        cwd=[launch]
    )

    spawn_robot = Node(package='gazebo_ros', 
                       executable='spawn_entity.py',
                       arguments=['-topic', 'robot_description', '-entity', 'marvin'],
                       output='screen'
    )

    # Pointcloud to laserscan
    pointcloud_to_laserscan = Node(package='pointcloud_to_laserscan', 
                                   executable='pointcloud_to_laserscan_node', 
                                   ros_arguments=[
                                       '-p', 'target_frame:=base_footprint',
                                       '-p', 'range_min:=0.9',
                                       '-p', 'range_max:=100.0',
                                       '-p', 'scan_time:=0.05',
                                       '-p', 'angle_increment:=0.00335'
                                   ],
                                   remappings=[
                                        ('/cloud_in', '/velodyne_points'),
                                        ('/scan', '/laser_scan'),
                                   ]
    )

    #RViz
    rviz_config_path = os.path.join(get_package_share_directory(pkg_name), 'rviz/simulation.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_server, 
        gazebo_client,
        spawn_robot,
        pointcloud_to_laserscan,
        rviz_node,
    ])