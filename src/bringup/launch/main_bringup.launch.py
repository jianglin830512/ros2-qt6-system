import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, Shutdown
from launch.substitutions import FindExecutable
from launch_ros.actions import Node  # <--- 导入 Node

def generate_launch_description():
    # --- 1. 定位所有需要的包和文件 ---
    bringup_pkg_share_dir = get_package_share_directory('bringup') 
    qt_node_share_dir = get_package_share_directory('qt_node')
    control_node_share_dir = get_package_share_directory('control_node')

    # 定义【覆盖】参数文件的路径 (由 bringup 包管理)
    override_params_file = os.path.join(bringup_pkg_share_dir, 'config', 'system_params.yaml')
    
    # 定义 QT 节点的【默认】参数文件的路径
    default_qt_params_file = os.path.join(qt_node_share_dir, 'config', 'qt_node_config.yaml')
    
    # 定义 Control 节点的【默认】参数文件的路径 <--- 新增
    default_control_params_file = os.path.join(control_node_share_dir, 'config', 'control_node_config.yaml')

    # --- 2. 直接定义 control_node 的启动动作 (不再使用 IncludeLaunchDescription) ---
    launch_control_node = Node(
        package='control_node',
        executable='control_node_executable',
        name='control_node',
        output='screen',
        parameters=[
            default_control_params_file,  # 首先加载默认参数
            override_params_file          # 然后用系统参数覆盖
        ]
    )

    # --- 3. 直接定义 qt_node 的启动动作 (保持不变) ---
    qt_executable_path = FindExecutable(name='qt_node_executable')
    
    qt_node_process = ExecuteProcess(
        cmd=[
            qt_executable_path,
            '--ros-args',
            '--params-file',
            default_qt_params_file,
            '--params-file',
            override_params_file
        ],
        output='screen',
        on_exit=Shutdown()
    )

    # --- 4. 创建并返回总的启动描述 ---
    return LaunchDescription([
        launch_control_node,
        qt_node_process
    ])