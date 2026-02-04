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
  "msg/SystemStatus.msg"
  "msg/CircuitSettings.msg"
  "msg/CircuitStatus.msg"
  "msg/HardwareCircuitSettings.msg"
  "msg/HardwareCircuitStatus.msg"
  "msg/HardwareLoopSettings.msg"
  "msg/HardwareLoopStatus.msg"
  "msg/LoopSettings.msg"
  "msg/LoopStatus.msg"
  "msg/RegulatorOperationCommand.msg"
  "msg/RegulatorSettings.msg"
  "msg/RegulatorStatus.msg"
  "msg/SampleSettings.msg"
  "msg/SystemSettings.msg"
  "msg/DataRecord.msg"
  "srv/CircuitBreakerCommand.srv"
  "srv/CircuitModeCommand.srv"
  "srv/RegulatorBreakerCommand.srv"
  "srv/SetCircuitSettings.srv"
  "srv/SetHardwareCircuitSettings.srv"
  "srv/SetRegulatorSettings.srv"
  "srv/SetSystemSettings.srv"
  "srv/GetDataRecords.srv"
  "srv/GetSystemSettings.srv"
  "srv/GetRegulatorSettings.srv"
  "srv/GetCircuitSettings.srv"
  "srv/SetHardwareCircuitControlMode.srv"
  "srv/SetHardwareCircuitControlSource.srv"
  # 声明这些 msg 文件内部所依赖的其他消息包
  DEPENDENCIES std_msgs builtin_interfaces
)
# ========================

ament_package()
