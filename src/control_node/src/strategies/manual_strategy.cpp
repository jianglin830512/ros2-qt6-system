#include "control_node/strategies/manual_strategy.hpp"
#include "ros2_interfaces/msg/hardware_circuit_status.hpp"

using HwStatus = ros2_interfaces::msg::HardwareCircuitStatus;

ManualStrategy::ManualStrategy(
    uint8_t circuit_id,
    std::shared_ptr<StateManager> state_manager,
    std::shared_ptr<IHardwareCoordinator> hardware_coordinator)
    : IControllerStrategy(circuit_id, state_manager, hardware_coordinator),
    last_check_time_(0, 0, RCL_SYSTEM_TIME)
{
}

void ManualStrategy::reset()
{
    last_check_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
}

void ManualStrategy::update()
{
    // Check synchronization with PLC (Low frequency: 1 Hz)
    auto now = rclcpp::Clock().now();
    if (now - last_check_time_ < check_interval_) {
        return;
    }
    last_check_time_ = now;

    // [NEW] 1. 强制 PLC 控制模式为 Manual
    if (current_plc_control_mode_ != HwStatus::PLC_MODE_MANUAL) {
        RCLCPP_INFO(rclcpp::get_logger("ManualStrategy"), "[Circuit %d] PLC Mode mismatch. Expected: MANUAL. Enforcing...", circuit_id_);
        hardware_coordinator_->set_circuit_control_mode(circuit_id_, HwStatus::PLC_MODE_MANUAL);
    }

    // [NEW] 2. 强制 PLC 反馈源与配置参数对齐（即使在手动模式，也保持配置一致性）
    auto settings = state_manager_->get_circuit_settings(circuit_id_);
    uint8_t target_plc_source = settings.curr_mode_use_ref ?
                                    HwStatus::PLC_SOURCE_REF_LOOP :
                                    HwStatus::PLC_SOURCE_TEST_LOOP;

    if (current_plc_control_source_ != target_plc_source) {
        RCLCPP_INFO(rclcpp::get_logger("ManualStrategy"), "[Circuit %d] PLC Source mismatch. Expected: %d. Enforcing...", circuit_id_, target_plc_source);
        hardware_coordinator_->set_circuit_control_source(circuit_id_, target_plc_source);
    }
}

// --- Command Handlers: Pass-through allowed in Manual Mode ---

void ManualStrategy::handle_regulator_operation_command(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg)
{
    // Directly forward to hardware
    hardware_coordinator_->send_regulator_operation_command(msg);
}

void ManualStrategy::handle_regulator_breaker_command(
    const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request>& request,
    StrategyCallback callback)
{
    // Directly forward to hardware
    hardware_coordinator_->execute_regulator_breaker_command(request, callback);
}

void ManualStrategy::handle_circuit_breaker_command(
    const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request>& request,
    StrategyCallback callback)
{
    // Directly forward to hardware
    hardware_coordinator_->execute_circuit_breaker_command(request, callback);
}
