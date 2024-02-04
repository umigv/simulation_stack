from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

#nav2_bringup/rviz/nav2_default_view.rviz'
#nav2_bringup/params/nav2_params.yaml

def generate_launch_description():
    # Nav2 Node
    planner_directory = get_package_share_directory("sam_bot_description")
    nav2_params_file = os.path.join(planner_directory, 'config', 'custom_fn_params', 'nav2_params.yaml')

    # RViz node\
    nav2_directory = get_package_share_directory("nav2_bringup")
    rviz_config = os.path.join(nav2_directory, 'rviz', 'nav2_default_view.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_directory, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': 'True',
            'use_sim_time': 'True',
            'params_file': nav2_params_file,
            'map': "",
        }.items(),
    )

    return LaunchDescription([
        rviz_node,
        bringup_cmd,
    ])