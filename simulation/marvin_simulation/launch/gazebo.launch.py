import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'marvin_simulation'
    model_subpath = 'urdf/marvin.xacro'
    world_subpath = 'worlds/course.world'
    launch_dir = os.path.join(get_package_share_directory(pkg_name), 'launch')

    world = os.path.join(get_package_share_directory(pkg_name), world_subpath)

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), model_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )

    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            '-s',
            'libgazebo_ros_init.so',
            '-s',
            'libgazebo_ros_factory.so',
            world,
        ],
        output='screen',
        cwd=[launch_dir]
    )

    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        cwd=[launch_dir]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'marvin'],
                    output='screen')

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
                                   ])
    
    # Run the node
    return LaunchDescription([
        gazebo_server, 
        gazebo_client,
        node_robot_state_publisher,
        spawn_entity,
        pointcloud_to_laserscan
    ])