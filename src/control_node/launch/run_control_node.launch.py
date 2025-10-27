import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # 获取包的 'share' 目录路径，用于定位配置文件
    package_share_dir = get_package_share_directory('control_node')

    # 1. 使用 DeclareLaunchArgument 来定位 .yaml 文件，并将它赋值给 params_file
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(package_share_dir, 'config', 'control_node_config.yaml'),
        description='Path to the ROS2 parameters file to use.'
    )
    
    control_node =  Node(
        package='control_node',
        executable='control_node_executable', 
        name='control_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]  # 加载参数文件
    )
    
    return LaunchDescription([
        declare_params_file_cmd,
        control_node
    ])
