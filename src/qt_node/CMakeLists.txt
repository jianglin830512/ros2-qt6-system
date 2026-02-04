cmake_minimum_required(VERSION 3.16)
project(qt_node LANGUAGES CXX)

# --- 标准设置 ---
set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- 查找 ament 和 ROS 2 包 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_interfaces REQUIRED)

# --- 查找Qt6包 ---
find_package(Qt6 6.8 REQUIRED COMPONENTS
    Core
    Gui
    Qml
    Quick
    Charts
    QuickControls2
    Widgets
    CoreTools  # 不是开发用的，而是工具包。
)

qt_policy(SET QTP0001 NEW)
qt_policy(SET QTP0004 NEW)

# 诊断信息，确认找到正确的Qt版本
message(STATUS "CMake found Qt version: ${Qt6_VERSION} at ${Qt6_DIR}")

# --- 定义可执行文件 ---
add_executable(qt_node_executable
    src/main.cpp
)

target_include_directories(qt_node_executable
    PUBLIC
        # 1. 你自己包的头文件
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>

        # ${rclcpp_INCLUDE_DIRS} # rclcpp只需要targit_link_libraries就够了

        # 2. std_msgs 的头文件
        ${std_msgs_INCLUDE_DIRS}

        # 3. ros2_interfaces 的头文件
        ${ros2_interfaces_INCLUDE_DIRS} # ros2_interfaces 仅包含头文件，所以不需要链接库
)

# --- 添加QML模块 ---
qt_add_qml_module(qt_node_executable
    URI qt_node
    VERSION 1.0
    RESOURCE_PREFIX /
    QML_FILES
        qml/Main.qml
        qml/MainForm.ui.qml              # 主窗口的纯UI布局
        qml/Theme.qml
        qml/components/StyledButton.qml            # 带有逻辑的自定义按钮
        qml/components/StyledButtonForm.ui.qml
        qml/components/ModeControl.qml
        qml/components/ModeControlForm.ui.qml
        qml/components/LoopStatus.qml
        qml/components/LoopStatusForm.ui.qml
        qml/components/CircuitStatus.qml           # 回路状态
        qml/components/CircuitStatusForm.ui.qml
        qml/components/RegulatorControl.qml        # 调压器控制
        qml/components/RegulatorControlForm.ui.qml
        qml/components/ActionButtonForm.ui.qml
        qml/components/ToggleActionButton.qml
        qml/components/ContinuousActionButton.qml
        qml/components/ValueAndUnit.ui.qml
        qml/components/SettingsGroup.qml
        qml/components/SettingInput.qml
        qml/components/SettingSwitch.qml
        qml/components/MessagePopup.qml
        qml/components/StartStopPanel.qml
        qml/components/SettingTextArea.qml
        qml/pages/CircuitSettingsPageForm.ui.qml
        qml/pages/CircuitSettingsPage.qml
        qml/pages/HistoryPage.qml
        qml/pages/SystemSettingsPage.qml
        qml/pages/SystemSettingsPageForm.ui.qml
        qml/pages/TemperatureMonitorPage.qml
        qml/pages/TemperatureMonitorPageForm.ui.qml
        qml/pages/TestDataPage.qml
    SOURCES
        include/qt_node/qt_node_constants.hpp # hpp only
        include/qt_node/qt_ros_node.hpp
        include/qt_node/ros_proxy.hpp
        include/qt_node/data_types/data_types.hpp # hpp only
        include/qt_node/data_types/sample_settings_data.hpp
        include/qt_node/data_types/loop_settings_data.hpp
        include/qt_node/data_types/circuit_settings_data.hpp
        include/qt_node/data_types/regulator_settings_data.hpp
        include/qt_node/data_types/system_settings_data.hpp
        src/qt_ros_node.cpp
        src/ros_proxy.cpp
        src/data_types/sample_settings_data.cpp
        src/data_types/loop_settings_data.cpp
        src/data_types/circuit_settings_data.cpp
        src/data_types/regulator_settings_data.cpp
        src/data_types/system_settings_data.cpp
)

# --- 链接库 ---
target_link_libraries(qt_node_executable PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Qml
    Qt6::Quick
    Qt6::Charts
    Qt6::QuickControls2
    Qt6::Widgets
    # ROS 2 依赖项 (使用导入的目标)
    # rclcpp 提供了现代的导入目标
    rclcpp::rclcpp

    # std_msgs 依赖的底层库通过变量提供
    ${std_msgs_LIBRARIES}

    # ros2_interfaces 依赖的底层库通过变量提供
    ${ros2_interfaces_LIBRARIES}
)

if(MSVC)
  # 为你的目标（target）添加 /bigobj 编译选项
  # 将 your_target_name 替换为你的 CMake target 名称，很可能就是 qt_ros_node
  target_compile_options(qt_node_executable PRIVATE /bigobj)
endif()

# =======================================================================
#               手动触发QML分析（这是解决问题的关键新增步骤）
# =======================================================================
# 强制CMake立即处理所有由 qt_add_qml_module 等函数排入计划的任务。
# 这会生成部署脚本所依赖的QML导入信息文件。
qt_finalize_target(qt_node_executable)

# =======================================================================
#               自动化部署和安装（修正版）
# =======================================================================

# 1. 安装可执行文件
install(TARGETS qt_node_executable
    RUNTIME DESTINATION bin
)

# 2. 使用专为QML应用设计的部署脚本生成函数
qt_generate_deploy_qml_app_script(
    TARGET qt_node_executable
    OUTPUT_SCRIPT deploy_script
    NO_UNSUPPORTED_PLATFORM_ERROR
)

# 3. 在安装时执行上面生成的部署脚本
install(SCRIPT ${deploy_script})

# 4. Colcon/ament 需要的安装指令
install(
    FILES package.xml
    DESTINATION share/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 安装 config 文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# 使用了 ament_cmake
ament_package()
