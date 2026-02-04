import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    package_share_dir = get_package_share_directory('hardware_node')

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_share_dir, 'config', 'hardware_node_config.yaml'),
        description='Path to the ROS2 config file (e.g., topics).'
    )
    
    # 保留有用的日志打印，这不会导致错误
    log_resolved_paths = LogInfo(
        msg=[
            ' [HARDWARE_NODE LAUNCH] Attempting to load parameter files: ',
            '\n\t- Config File: ', LaunchConfiguration('config_file')
        ]
    )

    hardware_node =  Node(
        package='hardware_node',
        executable='hardware_node_executable', 
        name='hardware_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file')
        ]
    )
    
    return LaunchDescription([
        declare_config_file_cmd,
        log_resolved_paths,
        hardware_node
    ])