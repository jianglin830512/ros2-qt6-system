cmake_minimum_required(VERSION 3.16)
project(record_node)

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
# 2. 查找依赖项 (先 ament，后外部库)
#-----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_interfaces REQUIRED)
find_package(SQLite3 REQUIRED)

#-----------------------------------------------------------------------------
# 3. 创建可执行文件 (添加了 main.cpp 和 record_node.cpp)
#-----------------------------------------------------------------------------
add_executable(record_node_executable
  src/main.cpp
  src/record_node.cpp
  src/database_manager.cpp
)

# 将头文件添加到目标中，方便 IDE 识别
target_sources(record_node_executable PRIVATE
  include/record_node/record_node.hpp
  include/record_node/record_node_constants.hpp
  include/record_node/database_manager.hpp
)

#-----------------------------------------------------------------------------
# 4. 配置目标 (包含目录、链接等)
#-----------------------------------------------------------------------------
# 使用 target_include_directories 使头文件可用
target_include_directories(record_node_executable PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 使用 ament_target_dependencies 处理 ROS 依赖
ament_target_dependencies(record_node_executable
  rclcpp
  std_msgs
  ros2_interfaces
)

# 链接外部库
target_link_libraries(record_node_executable
  SQLite::SQLite3
)

#-----------------------------------------------------------------------------
# 5. 安装规则
#-----------------------------------------------------------------------------
install(TARGETS
  record_node_executable
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 include 目录下的所有头文件
install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 安装我们新创建的配置文件
install(FILES
  config/record_node_config.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
