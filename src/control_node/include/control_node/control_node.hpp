#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"  // IWYU pragma: keep

// 状态消息 (发布)
#include "ros2_interfaces/msg/circuit_status.hpp"
#include "ros2_interfaces/msg/voltage_regulator_status.hpp"

// 命令消息 (订阅)
#include "ros2_interfaces/msg/voltage_regulator_command.hpp"
#include "ros2_interfaces/msg/circuit_command.hpp"
#include "std_msgs/msg/empty.hpp"

// 服务消息 (服务器)
#include "ros2_interfaces/srv/set_system_settings.hpp"
#include "ros2_interfaces/srv/set_regulator_settings.hpp"
#include "ros2_interfaces/srv/set_circuit_settings.hpp"

class ControlNode : public rclcpp::Node
{
public:
    ControlNode();
    ~ControlNode();

private:
    /**
     * @brief 定时器回调函数，用于定期发布状态消息
     */
    void timer_callback();

    // --- 命令回调函数 ---
    void regulator_command_callback(const ros2_interfaces::msg::VoltageRegulatorCommand::SharedPtr msg);
    void circuit_command_callback(const ros2_interfaces::msg::CircuitCommand::SharedPtr msg);
    void clear_alarm_callback(const std_msgs::msg::Empty::SharedPtr msg);

    // --- 服务回调函数 ---
    void set_system_settings_callback(
        const std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Request> request,
        std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Response> response);

    void set_regulator_settings_callback(
        const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request> request,
        std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Response> response);

    void set_circuit_settings_callback(
        const std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Request> request,
        std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Response> response);

    // --- 发布器相关成员 ---
    rclcpp::Publisher<ros2_interfaces::msg::CircuitStatus>::SharedPtr circuit_status_pub_;
    std::string circuit_status_topic_;
    rclcpp::Publisher<ros2_interfaces::msg::VoltageRegulatorStatus>::SharedPtr regulator_status_pub_;
    std::string regulator_status_topic_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --- 订阅器相关成员 ---
    rclcpp::Subscription<ros2_interfaces::msg::VoltageRegulatorCommand>::SharedPtr regulator_command_sub_;
    std::string regulator_command_topic_;
    rclcpp::Subscription<ros2_interfaces::msg::CircuitCommand>::SharedPtr circuit_command_sub_;
    std::string circuit_command_topic_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_alarm_sub_;
    std::string clear_alarm_topic_;

    // --- 服务服务器相关成员 ---
    rclcpp::Service<ros2_interfaces::srv::SetSystemSettings>::SharedPtr set_system_settings_service_;
    std::string set_system_settings_service_name_;
    rclcpp::Service<ros2_interfaces::srv::SetRegulatorSettings>::SharedPtr set_regulator_settings_service_;
    std::string set_regulator_settings_service_name_;
    rclcpp::Service<ros2_interfaces::srv::SetCircuitSettings>::SharedPtr set_circuit_settings_service_;
    std::string set_circuit_settings_service_name_;
};

#endif // CONTROL_NODE_HPP_
