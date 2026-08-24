import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    package_share_dir = get_package_share_directory('control_node')

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_share_dir, 'config', 'control_node_config.yaml'),
        description='Path to the ROS2 config file (e.g., topics).'
    )
    
    # 仅保留 config_file 的日志打印
    log_resolved_paths = LogInfo(
        msg=[
            ' [CONTROL_NODE LAUNCH] Attempting to load parameter files: ',
            '\n\t- Config File: ', LaunchConfiguration('config_file')
        ]
    )

    control_node = Node(
        package='control_node',
        executable='control_node_executable', 
        name='control_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file')  # 只加载这一个 config_file
        ]
    )
    
    return LaunchDescription([
        declare_config_file_cmd,
        log_resolved_paths,
        control_node
    ])