#include "control_node/persistence_coordinator.hpp"
#include "control_node/control_node_constants.hpp"

PersistenceCoordinator::PersistenceCoordinator(
    StateManager* state_manager,
    rclcpp::Node::SharedPtr node,
    rclcpp::CallbackGroup::SharedPtr client_cb_group)
    : state_manager_(state_manager), node_(node)
{
    // --- 1. 初始化保存服务客户端 (Set) ---
    auto save_sys_name = node_->declare_parameter<std::string>(control_node_constants::SAVE_SYSTEM_SETTINGS_SERVICE_PARAM, control_node_constants::DEFAULT_SAVE_SYSTEM_SETTINGS_SERVICE);
    save_system_settings_client_ = node_->create_client<ros2_interfaces::srv::SetSystemSettings>(save_sys_name, rclcpp::ServicesQoS(), client_cb_group);

    auto save_reg_name = node_->declare_parameter<std::string>(control_node_constants::SAVE_REGULATOR_SETTINGS_SERVICE_PARAM, control_node_constants::DEFAULT_SAVE_REGULATOR_SETTINGS_SERVICE);
    save_regulator_settings_client_ = node_->create_client<ros2_interfaces::srv::SetRegulatorSettings>(save_reg_name, rclcpp::ServicesQoS(), client_cb_group);

    auto save_cir_name = node_->declare_parameter<std::string>(control_node_constants::SAVE_CIRCUIT_SETTINGS_SERVICE_PARAM, control_node_constants::DEFAULT_SAVE_CIRCUIT_SETTINGS_SERVICE);
    save_circuit_settings_client_ = node_->create_client<ros2_interfaces::srv::SetCircuitSettings>(save_cir_name, rclcpp::ServicesQoS(), client_cb_group);

    // --- 2. 初始化查询服务客户端 (Get) ---
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

// --- 保存方法的实现 ---
void PersistenceCoordinator::save_system_settings(PersistenceCallback callback) {
    if (!save_system_settings_client_->service_is_ready()) { if(callback) callback(false, "Service Not Ready"); return; }
    auto request = std::make_shared<ros2_interfaces::srv::SetSystemSettings::Request>();
    request->settings = state_manager_->get_system_settings();
    save_system_settings_client_->async_send_request(request, [callback](rclcpp::Client<ros2_interfaces::srv::SetSystemSettings>::SharedFuture future) {
        auto resp = future.get();
        if(callback) callback(resp->success, resp->message);
    });
}
// (注：save_regulator_settings 和 save_circuit_settings 逻辑类似，此处省略具体重复代码，参考 save_system_settings 即可)
void PersistenceCoordinator::save_regulator_settings(uint8_t regulator_id, PersistenceCallback callback) {
    auto request = std::make_shared<ros2_interfaces::srv::SetRegulatorSettings::Request>();
    request->settings = state_manager_->get_regulator_settings(regulator_id);
    save_regulator_settings_client_->async_send_request(request, [callback](rclcpp::Client<ros2_interfaces::srv::SetRegulatorSettings>::SharedFuture future) {
        auto resp = future.get();
        if(callback) callback(resp->success, resp->message);
    });
}

void PersistenceCoordinator::save_circuit_settings(uint8_t circuit_id, PersistenceCallback callback) {
    auto request = std::make_shared<ros2_interfaces::srv::SetCircuitSettings::Request>();
    request->settings = state_manager_->get_circuit_settings(circuit_id);
    save_circuit_settings_client_->async_send_request(request, [callback](rclcpp::Client<ros2_interfaces::srv::SetCircuitSettings>::SharedFuture future) {
        auto resp = future.get();
        if(callback) callback(resp->success, resp->message);
    });
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
