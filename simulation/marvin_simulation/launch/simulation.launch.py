import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Constants
    package_name = 'marvin_simulation'
    package_directory = get_package_share_directory(package_name)
    launch_path = os.path.join(package_directory, 'launch')
    rviz_config = os.path.join(package_directory, 'rviz/simulation.rviz')
    model = xacro.process_file(os.path.join(package_directory, 'urdf', 'marvin.xacro')).toxml()

    # Arguments
    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(package_directory, 'worlds', 'course.world'),
        description='Absolute path to world file'
    )

    world = LaunchConfiguration('world')

    # Robot node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': model,
                     'use_sim_time': True}]
    )

    # Gazebo node
    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world,
        ],
        output='screen',
        cwd=[launch_path]
    )

    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        cwd=[launch_path]
    )

    spawn_robot = Node(package='gazebo_ros', 
                       executable='spawn_entity.py',
                       arguments=['-topic', 'robot_description', 
                                  '-entity', 'marvin'],
                       output='screen'
    )

    # Pointcloud to laserscan node
    pointcloud_to_laserscan = Node(package='pointcloud_to_laserscan', 
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

    #RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    # Launch Description
    return LaunchDescription([
        world_launch_arg,
        robot_state_publisher_node,
        gazebo_server, 
        gazebo_client,
        spawn_robot,
        pointcloud_to_laserscan,
        rviz_node,
    ])