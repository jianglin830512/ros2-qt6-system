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
    // Check synchronization with PLC (Low frequency)
    auto now = rclcpp::Clock().now();
    if (now - last_check_time_ < check_interval_) {
        return;
    }
    last_check_time_ = now;

    // Enforce PLC Mode: Manual
    // Note: PLC_MODE_MANUAL corresponds to ROS Control Mode MANUAL
    if (current_plc_control_mode_ != HwStatus::PLC_MODE_MANUAL) {
        // Send command to switch PLC to Manual
        hardware_coordinator_->set_circuit_control_mode(circuit_id_, HwStatus::PLC_MODE_MANUAL);
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
