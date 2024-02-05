from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
import os

def generate_launch_description():
    # Constants
    package_directory = get_package_share_directory('marvin_simulation')
    model = os.path.join(package_directory, 'urdf', 'marvin.xacro')
    rviz_config = os.path.join(package_directory, 'rviz', 'display.rviz')

    # Arguments
    enable_joint_publisher_launch_arg = DeclareLaunchArgument(
        name='joint_gui', 
        default_value='True',
        description='Flag to enable joint_state_publisher_gui'
    )

    # Robot Node
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', model])}]
    )

    # Wheel Node
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('joint_gui'))
    )

    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('joint_gui'))
    )

    # RViz Node
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    #Launch Description
    return launch.LaunchDescription([
        enable_joint_publisher_launch_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])