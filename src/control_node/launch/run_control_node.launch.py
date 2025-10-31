# control_node/launch/run_control_node.launch.py (最终、干净的版本)

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
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(package_share_dir, 'config', 'settings_params.yaml'),
        description='Path to the ROS2 parameters file (e.g., settings).'
    )
    
    # 保留有用的日志打印，这不会导致错误
    log_resolved_paths = LogInfo(
        msg=[
            ' [CONTROL_NODE LAUNCH] Attempting to load parameter files: ',
            '\n\t- Config File: ', LaunchConfiguration('config_file'),
            '\n\t- Params File: ', LaunchConfiguration('params_file')
        ]
    )

    control_node =  Node(
        package='control_node',
        executable='control_node_executable', 
        name='control_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            LaunchConfiguration('params_file')
        ]
    )
    
    return LaunchDescription([
        declare_config_file_cmd,
        declare_params_file_cmd,
        log_resolved_paths,
        control_node
    ])