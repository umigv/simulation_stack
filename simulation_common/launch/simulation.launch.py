from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import UnlessCondition
import os
import re

def get_latest_model_name():
    pattern = re.compile(r"^simulation_(\d+)_([a-zA-Z0-9_-]+)$")
    packages = get_packages_with_prefixes("simulation").keys()

    latest_model_package = None

    for package in packages:
        match = pattern.match(package)

        if match is None:
            continue

        if latest_model_package is None:
            latest_model_package = package
            continue
    
        latest_version = int(pattern.match(latest_model_package).group(1))
        current_version = int(pattern.match(package).group(1))
        if current_version > latest_version:
            latest_model_package = package

    return pattern.match(latest_model_package).group(2)

def get_package_of_model(model):
    pattern = re.compile(r"^simulation_(\d+)_" + model + "$")
    packages = get_packages_with_prefixes().keys()

    matching_packages = [package for package in packages if pattern.match(package) is not None]

    if len(matching_packages) == 0:
        return None
    
    return matching_packages[0]

def generate_launch_description():
    # Constants
    package_directory = get_package_share_directory('simulation_marvin')
    world_directory = get_package_share_directory('simulation_common')
    cwd = os.path.join(package_directory, 'launch')
    #model = os.path.join(package_directory, 'urdf', 'marvin.xacro')
    rviz_config = os.path.join(package_directory, 'rviz', 'simulation.rviz')
    default_world_name = 'empty'

    # Arguments
    world_launch_arg = DeclareLaunchArgument(
        name='world',
        default_value=default_world_name,
        description='Name of world file in the world directory'
    )

    headless_launch_arg = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Show RViz and Gazebo'
    )

    model_launch_arg = DeclareLaunchArgument(
        name = 'model',
        default_value = get_latest_model_name(),
        description = 'name of the robot model'
    )

    # Robot node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        # parameters=[{'robot_description': Command(['xacro ', model]), #TODO: USE LAUNCH ARG
        #              'use_sim_time': True}] 
    )

    # Gazebo node
    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            [world_directory, '/world/', LaunchConfiguration('world'), '.world']],
        output='screen',
        cwd=[cwd]
    )

    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        cwd=[cwd]
    )

    spawn_robot = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', 
                    '-entity', 'marvin'],
        output='screen'
    )

    # Pointcloud to laserscan node
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan', 
        executable='pointcloud_to_laserscan_node', 
        ros_arguments=[
            '-p', 'target_frame:=base_footprint',
            '-p', 'range_min:=0.9',
            '-p', 'range_max:=100.0',
            '-p', 'scan_time:=0.05',
            '-p', 'angle_increment:=0.00335',
            '-p', 'min_height:=0.01',
            ],
        remappings=[('/cloud_in', '/velodyne_points')]
    )

    # RViz node
    rviz_node = Node(
        condition=UnlessCondition(LaunchConfiguration('headless')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    # Launch Description
    return LaunchDescription([
        world_launch_arg,
        headless_launch_arg,
        model_launch_arg,
        robot_state_publisher_node,
        gazebo_server, 
        gazebo_client,
        spawn_robot,
        pointcloud_to_laserscan_node,
        rviz_node
    ])
