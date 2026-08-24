cmake_minimum_required(VERSION 3.16)
project(global_node)

#-----------------------------------------------------------------------------
# 1. 设置编译选项
#-----------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_options(/utf-8)
endif()

#-----------------------------------------------------------------------------
# 2. 查找依赖项
#-----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_interfaces REQUIRED)
find_package(SQLite3 REQUIRED)

#-----------------------------------------------------------------------------
# 3. 创建可执行文件
#-----------------------------------------------------------------------------
add_executable(global_node_executable
  src/main.cpp
  src/global_node.cpp
  src/database_manager.cpp
)

# 将头文件添加到目标中，方便 IDE 识别
target_sources(global_node_executable PRIVATE
  include/global_node/global_node.hpp
  include/global_node/global_node_constants.hpp
  include/global_node/database_manager.hpp
)

#-----------------------------------------------------------------------------
# 4. 配置目标 (包含目录、链接等)
#-----------------------------------------------------------------------------
target_include_directories(global_node_executable PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(global_node_executable
  rclcpp
  std_msgs
  ros2_interfaces
)

# 链接 SQLite3
target_link_libraries(global_node_executable
  SQLite::SQLite3
)

#-----------------------------------------------------------------------------
# 5. 安装规则
#-----------------------------------------------------------------------------
install(TARGETS
  global_node_executable
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES
  config/global_node_config.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()