import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, Shutdown, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart
from launch.substitutions import FindExecutable
from launch_ros.actions import Node

def generate_launch_description():
    """
    这是系统的主启动文件，负责编排和配置所有核心节点。
    - 它会按顺序启动 record_node, control_node, 和 qt_node。
    - 它使用一个位于 bringup/config/ 目录下的全局参数文件
      来覆盖各个节点的默认设置，实现集中式配置管理。
    """
    
    # --- 1. 定位所有需要的包和配置文件路径 ---

    # 获取各个功能包的 'share' 目录路径
    bringup_pkg_share_dir = get_package_share_directory('bringup') 
    qt_node_pkg_share_dir = get_package_share_directory('qt_node')
    control_node_pkg_share_dir = get_package_share_directory('control_node')
    record_node_pkg_share_dir = get_package_share_directory('record_node') # 新增 record_node 路径

    # 定义由 bringup 包管理的【全局覆盖】参数文件
    system_override_file = os.path.join(bringup_pkg_share_dir, 'config', 'system_params.yaml')
    
    # 定义各个节点的【默认】参数文件路径
    default_qt_params_file = os.path.join(qt_node_pkg_share_dir, 'config', 'qt_node_config.yaml')
    # 假设 control_node 的默认配置文件名和位置如下，请根据实际情况修改
    default_control_params_file = os.path.join(control_node_pkg_share_dir, 'config', 'control_node_config.yaml')
    # 新增 record_node 的默认配置文件路径
    default_record_params_file = os.path.join(record_node_pkg_share_dir, 'config', 'record_node_config.yaml')


    # --- 2. 定义各个节点的启动动作 ---
    
    # 定义 record_node 启动动作
    # 我们直接定义Node，而不是IncludeLaunchDescription，以便于事件处理器引用
    record_node = Node(
        package='record_node',
        executable='record_node_executable', 
        name='record_node',
        output='screen',
        parameters=[
            # 参数加载顺序：
            # 1. 首先加载自己的默认参数文件
            default_record_params_file,
            # 2. 然后加载全局覆盖文件。同名参数，后加载的会生效。
            system_override_file
        ]
    )

    # 定义 control_node 启动动作
    # 同样直接定义Node，以便于事件处理器引用
    # 注意：您需要将 'control_node_executable' 替换为 control_node 的实际可执行文件名
    control_node = Node(
        package='control_node',
        executable='control_node_executable', # <-- 请确认 control_node 的可执行文件名
        name='control_node',
        output='screen',
        parameters=[
            default_control_params_file,
            system_override_file
        ]
    )

    # 定义 qt_node 启动动作
    # ExecuteProcess 保持不变
    launch_qt_node = ExecuteProcess(
        cmd=[
            FindExecutable(name='qt_node_executable'),
            '--ros-args',
            '--params-file', default_qt_params_file,
            '--params-file', system_override_file
        ],
        output='screen',
        on_exit=Shutdown()
    )

    # --- 3. 使用事件处理器编排启动顺序 ---

    # 创建一个事件处理器：当 record_node 启动后，才开始启动 control_node
    handler_launch_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=record_node, # 监视 record_node 的启动事件
            on_start=[
                LogInfo(msg='record_node started. Launching control_node...'),
                control_node # record_node启动后，执行此动作
            ]
        )
    )

    # 创建第二个事件处理器：当 control_node 启动后，才开始启动 qt_node
    handler_launch_qt_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node, # 监视 control_node 的启动事件
            on_start=[
                LogInfo(msg='control_node started. Launching qt_node...'),
                launch_qt_node # control_node启动后，执行此动作
            ]
        )
    )

    # --- 4. 组装并返回最终的启动描述 ---
    # 启动描述中首先放入第一个需要启动的节点，以及所有的事件处理器。
    # 事件处理器会根据依赖关系，在正确的时机启动后续的节点。
    return LaunchDescription([
        LogInfo(msg='Launching record_node...'),
        record_node, # 1. 首先启动 record_node
        handler_launch_control_node, # 2. 然后注册事件，等待 record_node 启动后启动 control_node
        handler_launch_qt_node       # 3. 最后注册事件，等待 control_node 启动后启动 qt_node
    ])