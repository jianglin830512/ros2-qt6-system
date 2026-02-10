import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, Shutdown, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart
from launch.substitutions import FindExecutable
from launch_ros.actions import Node

def generate_launch_description():
    """
    系统主启动文件。
    启动顺序: record_node -> hardware_node -> control_node -> qt_node
    """
    
    # --- 1. 定位所有需要的包和配置文件路径 ---

    # 获取各个功能包的 'share' 目录路径
    bringup_pkg_share_dir = get_package_share_directory('bringup') 
    qt_node_pkg_share_dir = get_package_share_directory('qt_node')
    control_node_pkg_share_dir = get_package_share_directory('control_node')
    record_node_pkg_share_dir = get_package_share_directory('record_node')
    hardware_node_pkg_share_dir = get_package_share_directory('hardware_node') # 新增 hardware_node 路径

    # 定义全局覆盖参数文件
    system_override_file = os.path.join(bringup_pkg_share_dir, 'config', 'system_params.yaml')
    
    # 定义各个节点的【默认】参数文件路径
    default_qt_params_file = os.path.join(qt_node_pkg_share_dir, 'config', 'qt_node_config.yaml')
    default_control_params_file = os.path.join(control_node_pkg_share_dir, 'config', 'control_node_config.yaml')
    default_record_params_file = os.path.join(record_node_pkg_share_dir, 'config', 'record_node_config.yaml')
    # 新增 hardware_node 默认参数文件路径 (请确认文件名是否正确)
    default_hardware_params_file = os.path.join(hardware_node_pkg_share_dir, 'config', 'hardware_node_config.yaml')


    # --- 2. 定义各个节点的启动动作 ---
    
    # [节点 1] record_node
    record_node = Node(
        package='record_node',
        executable='record_node_executable', 
        name='record_node',
        output='screen',
        parameters=[default_record_params_file, system_override_file]
    )

    # [节点 2] hardware_node (新增)
    # 警告：请确认 executable='hardware_node_executable' 是否与 CMakeLists.txt 中定义的目标名称一致
    hardware_node = Node(
        package='hardware_node',
        executable='hardware_node_executable', 
        name='hardware_node',
        output='screen',
        parameters=[default_hardware_params_file, system_override_file]
    )

    # [节点 3] control_node
    control_node = Node(
        package='control_node',
        executable='control_node_executable',
        name='control_node',
        output='screen',
        parameters=[default_control_params_file, system_override_file]
    )

    # [节点 4] qt_node (GUI)
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

    # --- 3. 使用事件处理器编排启动顺序 (链式启动) ---

    # 链条环节 1: record_node 启动后 -> 启动 hardware_node
    handler_launch_hardware = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=record_node, # 监视 record_node
            on_start=[
                LogInfo(msg='record_node started. Launching hardware_node...'),
                hardware_node
            ]
        )
    )

    # 链条环节 2: hardware_node 启动后 -> 启动 control_node
    handler_launch_control = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=hardware_node, # 监视 hardware_node
            on_start=[
                LogInfo(msg='hardware_node started. Launching control_node...'),
                control_node
            ]
        )
    )

    # 链条环节 3: control_node 启动后 -> 启动 qt_node
    handler_launch_qt = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node, # 监视 control_node
            on_start=[
                LogInfo(msg='control_node started. Launching qt_node...'),
                launch_qt_node
            ]
        )
    )

    # --- 4. 组装并返回最终的启动描述 ---
    # 我们只需要将链条的第一个节点 (record_node) 和所有的事件处理器放入描述中。
    # 处理器会自动接管后续的流程。
    return LaunchDescription([
        LogInfo(msg='System Bringup Started. Launching sequence: Record -> Hardware -> Control -> Qt'),
        
        # 1. 启动第一个节点
        record_node,
        
        # 2. 注册所有事件处理器 (顺序无关，因为它们是基于事件触发的，但逻辑上如下)
        handler_launch_hardware, # 等待 record -> 启动 hardware
        handler_launch_control,  # 等待 hardware -> 启动 control
        handler_launch_qt        # 等待 control -> 启动 qt
    ])