#ifndef RECORD_NODE_HPP_
#define RECORD_NODE_HPP_

#include "rclcpp/rclcpp.hpp"  // IWYU pragma: keep
#include "ros2_interfaces/srv/set_system_settings.hpp"
#include "ros2_interfaces/srv/set_regulator_settings.hpp"
#include "ros2_interfaces/srv/set_circuit_settings.hpp"
#include "record_node/database_manager.hpp"
#include <memory>

class RecordNode : public rclcpp::Node
{
public:
    RecordNode();

private:
    // --- 服务回调函数 ---
    void save_system_settings_callback(
        const std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Request> request,
        std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Response> response);

    void save_regulator_settings_callback(
        const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request> request,
        std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Response> response);

    void save_circuit_settings_callback(
        const std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Request> request,
        std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Response> response);

    // --- 核心组件 ---
    std::unique_ptr<DatabaseManager> db_manager_;

    // --- ROS 服务服务器 ---
    rclcpp::Service<ros2_interfaces::srv::SetSystemSettings>::SharedPtr save_system_settings_service_;
    rclcpp::Service<ros2_interfaces::srv::SetRegulatorSettings>::SharedPtr save_regulator_settings_service_;
    rclcpp::Service<ros2_interfaces::srv::SetCircuitSettings>::SharedPtr save_circuit_settings_service_;
};

#endif // RECORD_NODE_HPP_
