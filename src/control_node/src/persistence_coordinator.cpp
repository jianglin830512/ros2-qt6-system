#include "control_node/persistence_coordinator.hpp"
#include "control_node/control_node_constants.hpp"

PersistenceCoordinator::PersistenceCoordinator(
    StateManager* state_manager,
    rclcpp::Node::SharedPtr node,
    rclcpp::CallbackGroup::SharedPtr client_cb_group)
    : state_manager_(state_manager), node_(node),
    last_db_status_time_(0, 0, RCL_SYSTEM_TIME)
{
    // --- 1. 初始化查询服务客户端 (Get) ---
    auto get_sys_name = node_->declare_parameter<std::string>(control_node_constants::GET_SYSTEM_SETTINGS_SERVICE_PARAM, control_node_constants::DEFAULT_GET_SYSTEM_SETTINGS_SERVICE);
    get_system_settings_client_ = node_->create_client<ros2_interfaces::srv::GetSystemSettings>(get_sys_name, rclcpp::ServicesQoS(), client_cb_group);

    auto get_reg_name = node_->declare_parameter<std::string>(control_node_constants::GET_REGULATOR_SETTINGS_SERVICE_PARAM, control_node_constants::DEFAULT_GET_REGULATOR_SETTINGS_SERVICE);
    get_regulator_settings_client_ = node_->create_client<ros2_interfaces::srv::GetRegulatorSettings>(get_reg_name, rclcpp::ServicesQoS(), client_cb_group);

    auto get_cir_name = node_->declare_parameter<std::string>(control_node_constants::GET_CIRCUIT_SETTINGS_SERVICE_PARAM, control_node_constants::DEFAULT_GET_CIRCUIT_SETTINGS_SERVICE);
    get_circuit_settings_client_ = node_->create_client<ros2_interfaces::srv::GetCircuitSettings>(get_cir_name, rclcpp::ServicesQoS(), client_cb_group);

    // --- 2. 订阅 RECORD NODE 的 DatabaseStatus (心跳) ---
    auto db_status_topic = node_->declare_parameter<std::string>(
        control_node_constants::DATABASE_STATUS_TOPIC_PARAM,
        control_node_constants::DEFAULT_DATABASE_STATUS_TOPIC);
    database_status_sub_ = node_->create_subscription<ros2_interfaces::msg::DatabaseStatus>(
        db_status_topic, rclcpp::QoS(10),
        std::bind(&PersistenceCoordinator::database_status_callback, this, std::placeholders::_1));
}

// 接收心跳与状态信息
void PersistenceCoordinator::database_status_callback(const ros2_interfaces::msg::DatabaseStatus::SharedPtr msg)
{
    last_db_status_time_ = node_->get_clock()->now();
    state_manager_->update_database_status(*msg);
}

bool PersistenceCoordinator::is_connected() const
{
    // [MODIFIED] 判断与 RECORD NODE 之间是否断线 (3秒超时逻辑)
    if (last_db_status_time_.nanoseconds() == 0) {
        return false; // 初始化阶段还没收到过任何消息，认为断线
    }

    auto now = node_->get_clock()->now();
    if ((now - last_db_status_time_).seconds() > 3.0) {
        return false; // 超过3秒没收到，认为掉线
    }

    // [NEW] 满足通信心跳的同时，还需要判断底层的 SQLite 是否真正处于 Connected 状态
    return state_manager_->get_database_status().database_connected;
}

// --- 获取方法 (Adapter) 的实现：将 Service Response 剥离，只回传 Message ---
void PersistenceCoordinator::get_system_settings(GetSystemSettingsCallback callback) {
    auto request = std::make_shared<ros2_interfaces::srv::GetSystemSettings::Request>();
    get_system_settings_client_->async_send_request(request, [callback](rclcpp::Client<ros2_interfaces::srv::GetSystemSettings>::SharedFuture future) {
        try {
            auto response = future.get();
            if (callback) callback(response->success, response->settings);
        } catch (...) {
            if (callback) callback(false, ros2_interfaces::msg::SystemSettings());
        }
    });
}
void PersistenceCoordinator::get_regulator_settings(uint8_t regulator_id, GetRegulatorSettingsCallback callback) {
    auto request = std::make_shared<ros2_interfaces::srv::GetRegulatorSettings::Request>();
    request->regulator_id = regulator_id;
    get_regulator_settings_client_->async_send_request(request, [callback](rclcpp::Client<ros2_interfaces::srv::GetRegulatorSettings>::SharedFuture future) {
        try {
            auto response = future.get();
            if (callback) callback(response->success, response->settings);
        } catch (...) {
            if (callback) callback(false, ros2_interfaces::msg::RegulatorSettings());
        }
    });
}
void PersistenceCoordinator::get_circuit_settings(uint8_t circuit_id, GetCircuitSettingsCallback callback) {
    auto request = std::make_shared<ros2_interfaces::srv::GetCircuitSettings::Request>();
    request->circuit_id = circuit_id;
    get_circuit_settings_client_->async_send_request(request, [callback](rclcpp::Client<ros2_interfaces::srv::GetCircuitSettings>::SharedFuture future) {
        try {
            auto response = future.get();
            if (callback) callback(response->success, response->settings);
        } catch (...) {
            if (callback) callback(false, ros2_interfaces::msg::CircuitSettings());
        }
    });
}
