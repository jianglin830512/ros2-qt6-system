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
    // 手动模式没有积分项，但重置一下时间戳是个好习惯
    last_check_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
}

void ManualStrategy::update()
{
    // 手动模式对账逻辑：
    // 持续检查硬件是否处于 Manual 模式。如果不是，发送指令切回 Manual。

    auto now = rclcpp::Clock().now();
    if ((now - last_check_time_).seconds() < 1.0) {
        return; // 1Hz 检查频率
    }
    last_check_time_ = now;

    auto hw_status = state_manager_->get_hardware_circuit_status(circuit_id_);

    if (hw_status.plc_control_mode != HwStatus::PLC_MODE_MANUAL) {
        hardware_coordinator_->set_circuit_control_mode(circuit_id_, HwStatus::PLC_MODE_MANUAL);
    }
}

void ManualStrategy::handle_manual_command(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg)
{
    // 直接透传命令
    hardware_coordinator_->send_regulator_operation_command(msg);
}
