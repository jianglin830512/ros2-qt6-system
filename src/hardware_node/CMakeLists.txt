cmake_minimum_required(VERSION 3.16)
project(hardware_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 为 MSVC 编译器（在 Windows 上）单独设置 UTF-8 选项
if(MSVC)
  add_compile_options(/utf-8)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_interfaces REQUIRED)

add_executable(hardware_node_executable
  src/main.cpp
  # Classes
  src/hardware_node.cpp
  src/tcp_hardware_driver.cpp
  src/mock_hardware_driver.cpp
  src/mock_device.cpp
)

target_sources(hardware_node_executable PRIVATE
  include/hardware_node/hardware_node_constants.hpp
  include/hardware_node/i_hardware_driver.hpp
  # Classes
  include/hardware_node/hardware_node.hpp
  include/hardware_node/tcp_hardware_driver.hpp
  include/hardware_node/mock_hardware_driver.hpp
  include/hardware_node/mock_device.hpp
)

# 添加本包的 include 目录
target_include_directories(hardware_node_executable PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(hardware_node_executable
  rclcpp
  std_msgs
  ros2_interfaces
)

install(TARGETS
  hardware_node_executable
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES
  config/hardware_node_config.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
