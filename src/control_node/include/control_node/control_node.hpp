#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"  // IWYU pragma: keep
#include <memory>

// === 包含所有需要的消息和服务的头文件 ===

// 命令消息 (订阅)
#include "ros2_interfaces/msg/voltage_regulator_command.hpp"
#include "ros2_interfaces/msg/circuit_command.hpp"
#include "std_msgs/msg/empty.hpp"

// 服务消息 (服务器)
#include "ros2_interfaces/srv/set_system_settings.hpp"
#include "ros2_interfaces/srv/set_regulator_settings.hpp"
#include "ros2_interfaces/srv/set_circuit_settings.hpp"

// === 前向声明，以避免在头文件中包含具体实现，保持解耦 ===
class StateManager;
class HardwareCoordinator;
class ControlLogic;
class PersistenceCoordinator;
struct ControlNodeContext;

class ControlNode : public rclcpp::Node
{
public:
    ControlNode();
    ~ControlNode(); // 析构函数是个好习惯

private:
    // === 回调函数声明 ===
    // --- 命令回调 (来自订阅) ---
    void regulator_command_callback(const ros2_interfaces::msg::VoltageRegulatorCommand::SharedPtr msg);
    void circuit_command_callback(const ros2_interfaces::msg::CircuitCommand::SharedPtr msg);
    void clear_alarm_callback(const std_msgs::msg::Empty::SharedPtr msg);

    // --- 服务回调 (来自服务) ---
    void set_system_settings_callback(
        const std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Request> request,
        std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Response> response);

    void set_regulator_settings_callback(
        const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request> request,
        std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Response> response);

    void set_circuit_settings_callback(
        const std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Request> request,
        std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Response> response);


    // === ROS 句柄 (API层) ===
    // --- 订阅器 (保留，因为它们是节点的输入接口) ---
    rclcpp::Subscription<ros2_interfaces::msg::VoltageRegulatorCommand>::SharedPtr regulator_command_sub_;
    rclcpp::Subscription<ros2_interfaces::msg::CircuitCommand>::SharedPtr circuit_command_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_alarm_sub_;

    // --- 服务服务器 (保留，因为它们是节点的输入接口) ---
    rclcpp::Service<ros2_interfaces::srv::SetSystemSettings>::SharedPtr set_system_settings_service_;
    rclcpp::Service<ros2_interfaces::srv::SetRegulatorSettings>::SharedPtr set_regulator_settings_service_;
    rclcpp::Service<ros2_interfaces::srv::SetCircuitSettings>::SharedPtr set_circuit_settings_service_;

    // --- 核心定时器 (新的，用于驱动控制逻辑) ---
    rclcpp::TimerBase::SharedPtr control_logic_timer_;


    // === 核心组件实例 (新的内部架构) ===
    std::shared_ptr<ControlNodeContext> context_;
    std::unique_ptr<ControlLogic> control_logic_;

    // === 配置变量 (保留，用于初始化) ===
    std::string circuit_status_topic_;
    std::string regulator_status_topic_;
    std::string regulator_command_topic_;
    std::string circuit_command_topic_;
    std::string clear_alarm_topic_;
    std::string set_system_settings_service_name_;
    std::string set_regulator_settings_service_name_;
    std::string set_circuit_settings_service_name_;
    std::string save_system_settings_service_name_;
    std::string save_regulator_settings_service_name_;
    std::string save_circuit_settings_service_name_;

    // === 多线程的组 ===
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr server_cb_group_;
};

#endif // CONTROL_NODE_HPP_
