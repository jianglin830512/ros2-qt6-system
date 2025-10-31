
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    package_share_dir = get_package_share_directory('record_node')

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_share_dir, 'config', 'record_node_config.yaml'),
        description='Path to the ROS2 config file (e.g., topics).'
    )

    record_node =  Node(
        package='record_node',
        executable='record_node_executable', 
        name='record_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file')
        ]
    )
    
    return LaunchDescription([
        declare_config_file_cmd,
        record_node
    ])