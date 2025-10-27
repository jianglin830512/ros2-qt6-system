import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, FindExecutable # <-- 1. 导入 FindExecutable

def generate_launch_description():
    # 获取包的 'share' 目录路径，用于定位配置文件
    package_share_dir = get_package_share_directory('qt_node')
    
    # 声明参数文件路径的启动参数
    params_file = LaunchConfiguration('params_file')

    # 1. 使用 DeclareLaunchArgument 来定位 .yaml 文件，并将它赋值给 params_file
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(package_share_dir, 'config', 'qt_node_config.yaml'),
        description='Path to the ROS2 parameters file to use.'
    )
    
    # 2. 使用 FindExecutable 来定位 .exe 文件
    executable_path = FindExecutable(name='qt_node_executable')
    
    qt_node = ExecuteProcess(
        cmd=[
            executable_path,  # <-- 2. exe 文件
            '--ros-args',
            '--params-file',
            params_file       # <-- 1. yaml 文件
        ],
        output='screen',
        )

    return LaunchDescription([
        declare_params_file_cmd,
        qt_node
    ])
