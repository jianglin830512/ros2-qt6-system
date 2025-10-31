cmake_minimum_required(VERSION 3.16)
project(control_node)

#-----------------------------------------------------------------------------
# 1. 设置编译选项
#-----------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 为 MSVC 编译器（在 Windows 上）单独设置 UTF-8 选项
if(MSVC)
  add_compile_options(/utf-8)
endif()

#-----------------------------------------------------------------------------
# 2. 查找依赖项 (先 ament，后外部库)
#-----------------------------------------------------------------------------
# ament/ROS 2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_interfaces REQUIRED)

# 外部库依赖
find_package(SQLite3 REQUIRED)
# 可以在这里加一句 message 检查 find_package 是否成功，方便调试
message(STATUS "Found SQLite3: ${SQLite3_LIBRARIES}")

#-----------------------------------------------------------------------------
# 3. 创建可执行文件
#-----------------------------------------------------------------------------
add_executable(control_node_executable
  src/main.cpp
  src/control_node.cpp
  src/state_manager.cpp
  src/hardware_coordinator.cpp
  src/control_logic.cpp
  src/strategies/manual_strategy.cpp
  src/strategies/auto_current_strategy.cpp
  src/persistence_coordinator.cpp
)

#-----------------------------------------------------------------------------
# 4. 配置目标 (包含目录、链接等)
#-----------------------------------------------------------------------------
# 将头文件添加到目标中，方便 IDE 识别
target_sources(control_node_executable PRIVATE
  include/control_node/control_node.hpp
  include/control_node/control_node_constants.hpp
  include/control_node/state_manager.hpp
  include/control_node/hardware_coordinator.hpp
  include/control_node/control_logic.hpp
  include/control_node/strategies/manual_strategy.hpp
  include/control_node/strategies/auto_current_strategy.hpp
  include/control_node/control_node_context.hpp
  include/control_node/i_controller_strategy.hpp
  include/control_node/persistence_coordinator.hpp
)

# 添加本包的 include 目录
target_include_directories(control_node_executable PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 分别链接 ament 依赖和外部库
# (Step 4.1) 使用 ament_target_dependencies 链接所有 ament 包
ament_target_dependencies(control_node_executable
  rclcpp
  std_msgs
  ros2_interfaces
)

# (Step 4.2) 使用标准的 target_link_libraries 链接所有外部库
target_link_libraries(control_node_executable
  SQLite::SQLite3
)

#-----------------------------------------------------------------------------
# 5. 安装规则
#-----------------------------------------------------------------------------
install(TARGETS
  control_node_executable
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES
  config/control_node_config.yaml
  config/settings_params.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

#-----------------------------------------------------------------------------
# 6. ament 包清单
#-----------------------------------------------------------------------------
ament_package()
