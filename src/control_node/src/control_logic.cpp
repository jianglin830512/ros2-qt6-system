#include "control_node/control_logic.hpp"
#include "control_node/state_manager.hpp"
#include "control_node/hardware_coordinator.hpp"
#include "control_node/strategies/manual_strategy.hpp"
#include "control_node/strategies/auto_current_strategy.hpp"

// 构造函数
ControlLogic::ControlLogic(std::shared_ptr<ControlNodeContext> context):context_(context)
{
    state_manager_ = std::make_shared<StateManager>();
    hardware_coordinator_ = std::make_shared<HardwareCoordinator>(
        state_manager_.get(), context_);
    persistence_coordinator_ = std::make_unique<PersistenceCoordinator>(
        state_manager_.get(), context_);

    switch_mode("manual");
}
void ControlLogic::update() { if (active_strategy_) { active_strategy_->update(); } }
void ControlLogic::switch_mode(const std::string& new_mode) { /* ... */ }


// *** 修改点: 实现新的异步 handle_ 方法 ***

void ControlLogic::handle_set_system_settings_request(
    const std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Request>& request,
    LogicResultCallback callback)
{
    RCLCPP_INFO(rclcpp::get_logger("ControlLogic"), "Handling SetSystemSettings request...");
    state_manager_->update_system_settings(request->settings);

    persistence_coordinator_->save_system_settings(
        [callback](bool success, const std::string& message) {
            if (callback) {
                std::string final_message = success ?
                                                "System settings updated and saved successfully." :
                                                "Settings updated in memory, but failed to save: " + message;
                callback(success, final_message);
            }
        }
        );
}

void ControlLogic::handle_set_regulator_settings_request(
    const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request>& request,
    LogicResultCallback callback)
{
    uint8_t id = request->regulator_id;
    const auto& settings = request->settings;

    RCLCPP_INFO(rclcpp::get_logger("ControlLogic"), "Coordinating SetRegulatorSettings flow for ID %u...", id);

    bool hardware_success = hardware_coordinator_->apply_regulator_settings_to_hardware(id, settings);

    if (!hardware_success) {
        RCLCPP_ERROR(rclcpp::get_logger("ControlLogic"), "Failed to apply settings to hardware for regulator %u. Aborting.", id);
        if (callback) {
            callback(false, "Failed to apply settings to hardware.");
        }
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("ControlLogic"), "Hardware update successful for regulator %u.", id);
    state_manager_->update_regulator_settings(id, settings);

    persistence_coordinator_->save_regulator_settings(
        id,
        [callback, id](bool persistence_success, const std::string& message) {
            if (callback) {
                std::string final_message = persistence_success ?
                                                "Regulator settings applied to hardware and saved successfully." :
                                                "Hardware updated, but failed to save settings: " + message;
                // 最终是否成功，取决于持久化是否成功
                callback(persistence_success, final_message);
            }
        }
        );
}

void ControlLogic::handle_set_circuit_settings_request(
    const std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Request>& request,
    LogicResultCallback callback)
{
    uint8_t id = request->circuit_id;
    const auto& settings = request->settings;

    RCLCPP_INFO(rclcpp::get_logger("ControlLogic"), "Coordinating SetCircuitSettings flow for ID %u...", id);

    bool hardware_success = hardware_coordinator_->apply_circuit_settings_to_hardware(id, settings);

    if (!hardware_success) {
        RCLCPP_ERROR(rclcpp::get_logger("ControlLogic"), "Failed to apply settings to hardware for circuit %u. Aborting.", id);
        if (callback) {
            callback(false, "Failed to apply settings to hardware.");
        }
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("ControlLogic"), "Hardware update successful for circuit %u.", id);
    state_manager_->update_circuit_settings(id, settings);

    persistence_coordinator_->save_circuit_settings(
        id,
        [callback, id](bool persistence_success, const std::string& message) {
            if (callback) {
                std::string final_message = persistence_success ?
                                                "Circuit settings applied to hardware and saved successfully." :
                                                "Hardware updated, but failed to save settings: " + message;
                callback(persistence_success, final_message);
            }
        }
        );
}
