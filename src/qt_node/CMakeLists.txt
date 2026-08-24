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
    CoreTools
)

qt_policy(SET QTP0001 NEW)
qt_policy(SET QTP0004 NEW)

message(STATUS "CMake found Qt version: ${Qt6_VERSION} at ${Qt6_DIR}")

# =======================================================================
# 核心修复区：手动去重 ROS 2 的巨长依赖链，避开 MSVC 的 128KB 字符限制
# 这样既能缩短命令，又不需要调用 ament_target_dependencies 导致 Qt 报错
# =======================================================================

# 1. 收集并去重 Include 目录
set(ROS2_INCLUDE_DIRS
    ${std_msgs_INCLUDE_DIRS}
    ${ros2_interfaces_INCLUDE_DIRS}
)
if(ROS2_INCLUDE_DIRS)
    list(REMOVE_DUPLICATES ROS2_INCLUDE_DIRS)
endif()

# 2. 收集并去重 Libraries
set(ROS2_LIBRARIES
    ${std_msgs_LIBRARIES}
    ${ros2_interfaces_LIBRARIES}
)
if(ROS2_LIBRARIES)
    list(REMOVE_DUPLICATES ROS2_LIBRARIES)
endif()

# =======================================================================

# --- 定义可执行文件 ---
add_executable(qt_node_executable
    src/main.cpp
)

target_include_directories(qt_node_executable
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        # 使用去重后的 Include 目录
        ${ROS2_INCLUDE_DIRS}
)

# --- 添加QML模块 ---
qt_add_qml_module(qt_node_executable
    URI qt_node
    VERSION 1.0
    RESOURCE_PREFIX /
    QML_FILES
        qml/Main.qml
        qml/MainForm.ui.qml
        qml/Theme.qml
        qml/components/StyledButton.qml
        qml/components/StyledButtonForm.ui.qml
        qml/components/ModeControl.qml
        qml/components/ModeControlForm.ui.qml
        qml/components/LoopStatus.qml
        qml/components/LoopStatusForm.ui.qml
        qml/components/CircuitStatus.qml
        qml/components/CircuitStatusForm.ui.qml
        qml/components/RegulatorControl.qml
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
        qml/components/InputBlocker.qml
        qml/components/QueryPanel.qml
        qml/components/CableEditDialog.qml
        qml/pages/TemperatureMonitorPageForm.ui.qml
        qml/pages/TemperatureMonitorPage.qml
        qml/pages/SystemSettingsPageForm.ui.qml
        qml/pages/SystemSettingsPage.qml
        qml/pages/CircuitSettingsPageForm.ui.qml
        qml/pages/CircuitSettingsPage.qml
        qml/pages/TestDataPageForm.ui.qml
        qml/pages/TestDataPage.qml
        qml/pages/HistoryPageForm.ui.qml
        qml/pages/HistoryPage.qml
        qml/pages/DataExportPageForm.ui.qml
        qml/pages/DataExportPage.qml
        qml/pages/CableManagementPageForm.ui.qml
        qml/pages/CableManagementPage.qml
    SOURCES
        include/qt_node/qt_node_constants.hpp
        include/qt_node/qt_ros_node.hpp
        include/qt_node/ros_proxy.hpp
        include/qt_node/data_types/data_types.hpp
        include/qt_node/data_types/loop_settings_data.hpp
        include/qt_node/data_types/circuit_settings_data.hpp
        include/qt_node/data_types/regulator_settings_data.hpp
        include/qt_node/data_types/system_settings_data.hpp
        include/qt_node/data_types/cable_data.hpp
        src/qt_ros_node.cpp
        src/ros_proxy.cpp
        src/data_types/loop_settings_data.cpp
        src/data_types/circuit_settings_data.cpp
        src/data_types/regulator_settings_data.cpp
        src/data_types/system_settings_data.cpp
        src/data_types/cable_data.cpp
)

# --- 链接库 ---
# 使用严格的 PRIVATE 关键字，完美迎合 Qt6 的胃口
target_link_libraries(qt_node_executable PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Qml
    Qt6::Quick
    Qt6::Charts
    Qt6::QuickControls2
    Qt6::Widgets
    rclcpp::rclcpp
    # 使用去重后的库变量
    ${ROS2_LIBRARIES}
)

if(MSVC)
  target_compile_options(qt_node_executable PRIVATE /bigobj)
endif()

# =======================================================================
#               手动触发QML分析
# =======================================================================
qt_finalize_target(qt_node_executable)

# =======================================================================
#               自动化部署和安装
# =======================================================================
install(TARGETS qt_node_executable
    RUNTIME DESTINATION bin
)

qt_generate_deploy_qml_app_script(
    TARGET qt_node_executable
    OUTPUT_SCRIPT deploy_script
    NO_UNSUPPORTED_PLATFORM_ERROR
)

install(SCRIPT ${deploy_script})

install(
    FILES package.xml
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
