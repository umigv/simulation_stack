from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import UnlessCondition
import os

def generate_launch_description():
    # Constants
    package_directory = get_package_share_directory('marvin_simulation')
    cwd = os.path.join(package_directory, 'launch')
    rviz_config = os.path.join(package_directory, 'rviz', 'simulation.rviz')
    default_model = os.path.join(package_directory, 'urdf', 'marvin.xacro')
    default_world = os.path.join(package_directory, 'worlds', 'course.world')
    headless = LaunchConfiguration('headless')

    # Arguments
    world_launch_arg = DeclareLaunchArgument(
        name='world',
        default_value=default_world,
        description='Absolute path to world file'
    )

    model_launch_arg = DeclareLaunchArgument(
        name='model', 
        default_value=default_model,
        description='Absolute path to robot urdf file'
    )

    headless_arg = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Show RViz and Gazebo'
    )

    # Robot node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')]),
                     'use_sim_time': True}]
    )

    # Gazebo node
    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            LaunchConfiguration('world')],
        output='screen',
        cwd=[cwd]
    )

    gazebo_client = ExecuteProcess(
        condition=UnlessCondition(headless),
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
            '-p', 'angle_increment:=0.00335'],
        remappings=[
            ('/cloud_in', '/velodyne_points'),
            ('/scan', '/laser_scan')]
    )

    # RViz node
    rviz_node = Node(
        condition=UnlessCondition(headless),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    # Launch Description
    return LaunchDescription([
        world_launch_arg,
        model_launch_arg,
        headless_arg,
        robot_state_publisher_node,
        gazebo_server, 
        gazebo_client,
        spawn_robot,
        pointcloud_to_laserscan_node,
        rviz_node,
    ])