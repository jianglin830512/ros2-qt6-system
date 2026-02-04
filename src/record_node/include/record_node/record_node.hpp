#ifndef RECORD_NODE_HPP_
#define RECORD_NODE_HPP_

#include "rclcpp/rclcpp.hpp"  // IWYU pragma: keep
#include "ros2_interfaces/srv/set_system_settings.hpp"
#include "ros2_interfaces/srv/set_regulator_settings.hpp"
#include "ros2_interfaces/srv/set_circuit_settings.hpp"
#include "ros2_interfaces/srv/get_system_settings.hpp"
#include "ros2_interfaces/srv/get_regulator_settings.hpp"
#include "ros2_interfaces/srv/get_circuit_settings.hpp"
#include "ros2_interfaces/srv/get_data_records.hpp"
#include "ros2_interfaces/msg/circuit_status.hpp"
#include "ros2_interfaces/msg/regulator_status.hpp"
#include "record_node/database_manager.hpp"
#include <memory>
#include <map>

class RecordNode : public rclcpp::Node
{
public:
    RecordNode();

private:
    // --- 服务回调函数 (设置类) ---
    void save_system_settings_callback(
        const std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Request> request,
        std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Response> response);

    void save_regulator_settings_callback(
        const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request> request,
        std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Response> response);

    void save_circuit_settings_callback(
        const std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Request> request,
        std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Response> response);

    // --- 服务回调函数 (查询类 - 新增) ---
    void get_system_settings_callback(
        const std::shared_ptr<ros2_interfaces::srv::GetSystemSettings::Request> request,
        std::shared_ptr<ros2_interfaces::srv::GetSystemSettings::Response> response);

    void get_regulator_settings_callback(
        const std::shared_ptr<ros2_interfaces::srv::GetRegulatorSettings::Request> request,
        std::shared_ptr<ros2_interfaces::srv::GetRegulatorSettings::Response> response);

    void get_circuit_settings_callback(
        const std::shared_ptr<ros2_interfaces::srv::GetCircuitSettings::Request> request,
        std::shared_ptr<ros2_interfaces::srv::GetCircuitSettings::Response> response);

    void get_data_records_callback(
        const std::shared_ptr<ros2_interfaces::srv::GetDataRecords::Request> request,
        std::shared_ptr<ros2_interfaces::srv::GetDataRecords::Response> response);

    // --- Topic 回调函数 ---
    void circuit_status_callback(const ros2_interfaces::msg::CircuitStatus::SharedPtr msg);
    void regulator_status_callback(const ros2_interfaces::msg::RegulatorStatus::SharedPtr msg);

    // --- 定时记录逻辑 ---
    void start_alignment_timer();
    void record_timer_callback();

    // --- 核心组件 ---
    std::unique_ptr<DatabaseManager> db_manager_;

    // --- ROS 服务服务器 (设置类) ---
    rclcpp::Service<ros2_interfaces::srv::SetSystemSettings>::SharedPtr save_system_settings_service_;
    rclcpp::Service<ros2_interfaces::srv::SetRegulatorSettings>::SharedPtr save_regulator_settings_service_;
    rclcpp::Service<ros2_interfaces::srv::SetCircuitSettings>::SharedPtr save_circuit_settings_service_;

    // --- ROS 服务服务器 (查询类 - 新增) ---
    rclcpp::Service<ros2_interfaces::srv::GetSystemSettings>::SharedPtr get_system_settings_service_;
    rclcpp::Service<ros2_interfaces::srv::GetRegulatorSettings>::SharedPtr get_regulator_settings_service_;
    rclcpp::Service<ros2_interfaces::srv::GetCircuitSettings>::SharedPtr get_circuit_settings_service_;
    rclcpp::Service<ros2_interfaces::srv::GetDataRecords>::SharedPtr get_data_records_service_;

    // --- ROS 订阅者 ---
    rclcpp::Subscription<ros2_interfaces::msg::CircuitStatus>::SharedPtr circuit_status_sub_;
    rclcpp::Subscription<ros2_interfaces::msg::RegulatorStatus>::SharedPtr regulator_status_sub_;

    // --- 内存数据存储 ---
    std::map<uint8_t, ros2_interfaces::msg::CircuitStatus> latest_circuit_status_;
    std::map<uint8_t, ros2_interfaces::msg::RegulatorStatus> latest_regulator_status_;

    // --- 定时器 ---
    rclcpp::TimerBase::SharedPtr alignment_timer_;
    rclcpp::TimerBase::SharedPtr record_timer_;

    // --- 配置与状态 ---
    int64_t record_interval_min_;
    bool keep_record_on_shutdown_ = true;
};

#endif // RECORD_NODE_HPP_
