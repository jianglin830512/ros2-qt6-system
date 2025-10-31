#ifndef CONTROL_NODE_CONTEXT_HPP
#define CONTROL_NODE_CONTEXT_HPP

#include "rclcpp/rclcpp.hpp"  // IWYU pragma: keep
#include "ros2_interfaces/msg/circuit_status.hpp"
#include "ros2_interfaces/msg/voltage_regulator_status.hpp"
#include "ros2_interfaces/srv/set_system_settings.hpp"
#include "ros2_interfaces/srv/set_regulator_settings.hpp"
#include "ros2_interfaces/srv/set_circuit_settings.hpp"

// 这是一个被动的数据结构，用于保存和共享ROS句柄
struct ControlNodeContext
{
    // --- 发布器 ---
    rclcpp::Publisher<ros2_interfaces::msg::CircuitStatus>::SharedPtr circuit_status_publisher;
    rclcpp::Publisher<ros2_interfaces::msg::VoltageRegulatorStatus>::SharedPtr regulator_status_publisher;

    // --- 服务客户端 ---
    rclcpp::Client<ros2_interfaces::srv::SetSystemSettings>::SharedPtr save_system_settings_client;
    rclcpp::Client<ros2_interfaces::srv::SetRegulatorSettings>::SharedPtr save_regulator_settings_client;
    rclcpp::Client<ros2_interfaces::srv::SetCircuitSettings>::SharedPtr save_circuit_settings_client;

    // 指向ROS节点的日志记录器，方便所有类使用
     rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface;
};

#endif // CONTROL_NODE_CONTEXT_HPP
