#include "control_node/persistence_coordinator.hpp"
#include "control_node/control_node_constants.hpp"
PersistenceCoordinator::PersistenceCoordinator(
    StateManager* state_manager,
    rclcpp::Node::SharedPtr node,
    rclcpp::CallbackGroup::SharedPtr client_cb_group)
    : state_manager_(state_manager), node_(node)
{
    // --- 1. 初始化查询服务客户端 (Get) ---
    auto get_sys_name = node_->declare_parameter<std::string>(control_node_constants::GET_SYSTEM_SETTINGS_SERVICE_PARAM, control_node_constants::DEFAULT_GET_SYSTEM_SETTINGS_SERVICE);
    get_system_settings_client_ = node_->create_client<ros2_interfaces::srv::GetSystemSettings>(get_sys_name, rclcpp::ServicesQoS(), client_cb_group);

    auto get_reg_name = node_->declare_parameter<std::string>(control_node_constants::GET_REGULATOR_SETTINGS_SERVICE_PARAM, control_node_constants::DEFAULT_GET_REGULATOR_SETTINGS_SERVICE);
    get_regulator_settings_client_ = node_->create_client<ros2_interfaces::srv::GetRegulatorSettings>(get_reg_name, rclcpp::ServicesQoS(), client_cb_group);

    auto get_cir_name = node_->declare_parameter<std::string>(control_node_constants::GET_CIRCUIT_SETTINGS_SERVICE_PARAM, control_node_constants::DEFAULT_GET_CIRCUIT_SETTINGS_SERVICE);
    get_circuit_settings_client_ = node_->create_client<ros2_interfaces::srv::GetCircuitSettings>(get_cir_name, rclcpp::ServicesQoS(), client_cb_group);
}
bool PersistenceCoordinator::is_connected() const
{
    // 检查 "get_system_settings" 服务代表 Record Node 是否在线
    return get_system_settings_client_->service_is_ready();
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
