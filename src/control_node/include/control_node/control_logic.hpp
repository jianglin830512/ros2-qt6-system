// ============================================================
// FILE: E:\WGS\SRC\CONTROL_NODE\include\control_node\control_logic.hpp
// ============================================================
#ifndef CONTROL_LOGIC_HPP
#define CONTROL_LOGIC_HPP

#include <memory>
#include <string>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "ros2_interfaces/srv/set_system_settings.hpp"
#include "ros2_interfaces/srv/set_regulator_settings.hpp"
#include "ros2_interfaces/srv/set_circuit_settings.hpp"
#include "ros2_interfaces/msg/regulator_operation_command.hpp"
#include "ros2_interfaces/srv/regulator_breaker_command.hpp"
#include "ros2_interfaces/srv/circuit_breaker_command.hpp"

class StateManager;
class IHardwareCoordinator;
class IPersistenceCoordinator;

using LogicResultCallback = std::function<void(bool success, const std::string& message)>;

class ControlLogic
{
public:
    ControlLogic(
        std::shared_ptr<StateManager> state_manager,
        std::shared_ptr<IHardwareCoordinator> hardware_coordinator,
        std::shared_ptr<IPersistenceCoordinator> persistence_coordinator);

    void update();
    void maintain_lifecycle();
    bool is_system_ready() const;
    bool is_settings_synced() const;

    // --- Commands ---
    void process_regulator_operation_command(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg);
    void handle_regulator_breaker_command_request(const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request>& request, LogicResultCallback callback);
    void handle_circuit_breaker_command_request(const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request>& request, LogicResultCallback callback);
    void process_clear_alarm();

    // --- Settings ---
    void handle_set_system_settings_request(const std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Request>& request, LogicResultCallback callback);
    void handle_set_regulator_settings_request(const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request>& request, LogicResultCallback callback);
    void handle_set_circuit_settings_request(const std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Request>& request, LogicResultCallback callback);

private:
    void initialize_all_default_settings();
    ros2_interfaces::msg::SystemSettings    create_default_system_settings();
    ros2_interfaces::msg::RegulatorSettings create_default_regulator_settings(uint8_t id);
    ros2_interfaces::msg::CircuitSettings   create_default_circuit_settings(uint8_t id);

    // --- Lifecycle Helpers ---
    void attempt_sync_settings();

    // --- Core Logic Helpers ---
    void process_time_calculation();
    void process_manual_interlock();
    void process_auto_logic();

    // Auto mode control helpers
    void execute_auto_shutdown();
    void manage_loop_auto(uint8_t circuit_id, uint8_t loop_type, bool is_heating, uint8_t regulator_id, uint8_t target_plc_mode, bool control_regulator = true);

    // Hardware limits helper
    bool is_regulator_at_lower_limit(uint8_t regulator_id);

    // Time Overlap Checker
    bool check_time_overlap(const ros2_interfaces::msg::CircuitSettings& c1, const ros2_interfaces::msg::CircuitSettings& c2);

    std::shared_ptr<StateManager> state_manager_;
    std::shared_ptr<IHardwareCoordinator> hardware_coordinator_;
    std::shared_ptr<IPersistenceCoordinator> persistence_coordinator_;

    bool settings_synced_ = false;
    bool is_syncing_ = false;
    bool sys_settings_loaded_ = false;
    // 使用索引 1 和 2 对应 ID
    bool reg_settings_loaded_[3] = {false};
    bool cir_settings_loaded_[3] = {false};

    uint8_t current_lifecycle_state_ = 0;

    // Timer to prevent spamming PLC control commands
    rclcpp::Time last_plc_cmd_time_;
};

#endif // CONTROL_LOGIC_HPP
