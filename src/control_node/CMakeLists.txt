cmake_minimum_required(VERSION 3.16)
project(control_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 在 windows 系统上，编译格式和系统保持一致（GBK），所以这里要显示的指定为UTF-8
add_compile_options(/utf-8)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_interfaces REQUIRED)

# 1. 创建可执行文件，并列出其【需要编译】的源文件
add_executable(control_node_executable
  src/main.cpp
  src/control_node.cpp
)

# 2. 将【头文件】作为源文件添加到目标中，以便 Qt Creator 能够识别和显示它们
target_sources(control_node_executable PRIVATE
  include/control_node/control_node.hpp
  include/control_node/control_node_constants.hpp
)

# 3. 添加本包的 include 目录，以便编译器能找到头文件
target_include_directories(control_node_executable PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 4. 链接所有依赖项
ament_target_dependencies(control_node_executable
  rclcpp
  std_msgs
  ros2_interfaces
)

# 5. 安装可执行文件
install(TARGETS
  control_node_executable
  DESTINATION lib/${PROJECT_NAME}
)

# <<< 新增：安装 launch 文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# <<< 新增：安装 yaml 配置文件
install(FILES
  config/control_node_config.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
