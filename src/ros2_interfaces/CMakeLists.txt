cmake_minimum_required(VERSION 3.16)
project(ros2_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 列出您所有的 .msg 文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CircuitStatus.msg"
  "msg/VoltageRegulatorStatus.msg"
  "msg/CircuitCommand.msg"
  "msg/VoltageRegulatorCommand.msg"
  "msg/SampleSettings.msg"
  "msg/LoopSettings.msg"
  "msg/CircuitSettings.msg"
  "msg/VoltageRegulatorSettings.msg"
  "msg/SystemSettings.msg"
  "srv/SetCircuitSettings.srv"
  "srv/SetRegulatorSettings.srv"
  "srv/SetSystemSettings.srv"
  # 声明这些 msg 文件内部所依赖的其他消息包
  DEPENDENCIES std_msgs builtin_interfaces
)
# ========================

ament_package()
