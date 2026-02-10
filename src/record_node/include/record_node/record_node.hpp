#ifndef RECORD_NODE_HPP_
#define RECORD_NODE_HPP_

#include "rclcpp/rclcpp.hpp"  // IWYU pragma: keep
#include "ros2_interfaces/msg/system_settings.hpp"
#include "ros2_interfaces/msg/regulator_settings.hpp"
#include "ros2_interfaces/msg/circuit_settings.hpp"
#include "ros2_interfaces/msg/circuit_status.hpp"
#include "ros2_interfaces/msg/regulator_status.hpp"

// Query Services
#include "ros2_interfaces/srv/get_system_settings.hpp"
#include "ros2_interfaces/srv/get_regulator_settings.hpp"
#include "ros2_interfaces/srv/get_circuit_settings.hpp"
#include "ros2_interfaces/srv/get_data_records.hpp"

#include "record_node/database_manager.hpp"
#include <memory>
#include <map>

class RecordNode : public rclcpp::Node
{
public:
    RecordNode();

private:
    /**
     * @brief 从数据库加载初始设置到内存缓存中
     * 确保节点启动时内存中的状态与数据库一致
     */
    void load_initial_settings();

    // --- Topic 回调函数 (设置监听与自动保存) ---
    void system_settings_topic_callback(const ros2_interfaces::msg::SystemSettings::SharedPtr msg);
    void regulator_settings_topic_callback(const ros2_interfaces::msg::RegulatorSettings::SharedPtr msg);
    void circuit_settings_topic_callback(const ros2_interfaces::msg::CircuitSettings::SharedPtr msg);

    // --- 服务回调函数 (查询类) ---
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

    // --- Topic 回调函数 (状态记录) ---
    void circuit_status_callback(const ros2_interfaces::msg::CircuitStatus::SharedPtr msg);
    void regulator_status_callback(const ros2_interfaces::msg::RegulatorStatus::SharedPtr msg);

    // --- 定时记录逻辑 ---
    void reschedule_timers();
    void record_timer_callback();

    // --- 核心组件 ---
    std::unique_ptr<DatabaseManager> db_manager_;

    // --- ROS 订阅者 (设置类 - 新增) ---
    rclcpp::Subscription<ros2_interfaces::msg::SystemSettings>::SharedPtr system_settings_sub_;
    rclcpp::Subscription<ros2_interfaces::msg::RegulatorSettings>::SharedPtr regulator_settings_sub_;
    rclcpp::Subscription<ros2_interfaces::msg::CircuitSettings>::SharedPtr circuit_settings_sub_;

    // --- ROS 订阅者 (状态类) ---
    rclcpp::Subscription<ros2_interfaces::msg::CircuitStatus>::SharedPtr circuit_status_sub_;
    rclcpp::Subscription<ros2_interfaces::msg::RegulatorStatus>::SharedPtr regulator_status_sub_;

    // --- ROS 服务服务器 (查询类) ---
    rclcpp::Service<ros2_interfaces::srv::GetSystemSettings>::SharedPtr get_system_settings_service_;
    rclcpp::Service<ros2_interfaces::srv::GetRegulatorSettings>::SharedPtr get_regulator_settings_service_;
    rclcpp::Service<ros2_interfaces::srv::GetCircuitSettings>::SharedPtr get_circuit_settings_service_;
    rclcpp::Service<ros2_interfaces::srv::GetDataRecords>::SharedPtr get_data_records_service_;

    // --- 内存数据存储 (状态) ---
    std::map<uint8_t, ros2_interfaces::msg::CircuitStatus> latest_circuit_status_;
    std::map<uint8_t, ros2_interfaces::msg::RegulatorStatus> latest_regulator_status_;

    // --- 内存数据存储 (设置缓存 - 用于对比变更) ---
    ros2_interfaces::msg::SystemSettings current_system_settings_;
    std::map<uint8_t, ros2_interfaces::msg::RegulatorSettings> current_regulator_settings_;
    std::map<uint8_t, ros2_interfaces::msg::CircuitSettings> current_circuit_settings_;

    // --- 定时器 ---
    rclcpp::TimerBase::SharedPtr alignment_timer_;
    rclcpp::TimerBase::SharedPtr record_timer_;

    // --- 配置与状态 ---
    int64_t record_interval_min_;
    bool keep_record_on_shutdown_ = true;
};

#endif // RECORD_NODE_HPP_
