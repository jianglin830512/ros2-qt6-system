import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取配置文件的路径
    config_dir = os.path.join(get_package_share_directory('global_node'), 'config')
    config_file = os.path.join(config_dir, 'global_node_config.yaml')

    global_node = Node(
        package='global_node',
        executable='global_node_executable',
        name='global_node',
        # 将其置于 root namespace，保证跨命名空间可访问
        namespace='',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        global_node
    ])