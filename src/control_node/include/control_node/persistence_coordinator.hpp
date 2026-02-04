#ifndef PERSISTENCE_COORDINATOR_HPP
#define PERSISTENCE_COORDINATOR_HPP

#include "control_node/i_persistence_coordinator.hpp"
#include "control_node/state_manager.hpp"
#include "rclcpp/rclcpp.hpp" // IWYU pragma: keep

// 必须包含 Service 定义用于内部通信
#include "ros2_interfaces/srv/get_system_settings.hpp"
#include "ros2_interfaces/srv/get_regulator_settings.hpp"
#include "ros2_interfaces/srv/get_circuit_settings.hpp"
#include "ros2_interfaces/srv/set_system_settings.hpp"
#include "ros2_interfaces/srv/set_regulator_settings.hpp"
#include "ros2_interfaces/srv/set_circuit_settings.hpp"

class PersistenceCoordinator : public IPersistenceCoordinator
{
public:
    PersistenceCoordinator(
        StateManager* state_manager,
        rclcpp::Node::SharedPtr node,
        rclcpp::CallbackGroup::SharedPtr client_cb_group);

    bool is_connected() const override;

    // --- 实现 IPersistenceCoordinator 接口的保存方法 ---
    void save_system_settings(PersistenceCallback callback) override;
    void save_regulator_settings(uint8_t regulator_id, PersistenceCallback callback) override;
    void save_circuit_settings(uint8_t circuit_id, PersistenceCallback callback) override;

    // --- 实现 IPersistenceCoordinator 接口的获取方法 (Adapter) ---
    void get_system_settings(GetSystemSettingsCallback callback) override;
    void get_regulator_settings(uint8_t regulator_id, GetRegulatorSettingsCallback callback) override;
    void get_circuit_settings(uint8_t circuit_id, GetCircuitSettingsCallback callback) override;

private:
    StateManager* state_manager_;
    rclcpp::Node::SharedPtr node_;

    // --- ROS Service Clients ---
    // 保存类 (Set)
    rclcpp::Client<ros2_interfaces::srv::SetSystemSettings>::SharedPtr save_system_settings_client_;
    rclcpp::Client<ros2_interfaces::srv::SetRegulatorSettings>::SharedPtr save_regulator_settings_client_;
    rclcpp::Client<ros2_interfaces::srv::SetCircuitSettings>::SharedPtr save_circuit_settings_client_;

    // 查询类 (Get)
    rclcpp::Client<ros2_interfaces::srv::GetSystemSettings>::SharedPtr get_system_settings_client_;
    rclcpp::Client<ros2_interfaces::srv::GetRegulatorSettings>::SharedPtr get_regulator_settings_client_;
    rclcpp::Client<ros2_interfaces::srv::GetCircuitSettings>::SharedPtr get_circuit_settings_client_;
};

#endif // PERSISTENCE_COORDINATOR_HPP
