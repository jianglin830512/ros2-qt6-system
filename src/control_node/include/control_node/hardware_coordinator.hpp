#ifndef HARDWARE_COORDINATOR_HPP
#define HARDWARE_COORDINATOR_HPP

#include "rclcpp/rclcpp.hpp"  // IWYU pragma: keep
#include "control_node/state_manager.hpp"
#include "control_node/control_node_context.hpp"
#include <memory>

class HardwareCoordinator
{
public:
    // 构造时注入依赖
    HardwareCoordinator(StateManager* state_manager, std::shared_ptr<ControlNodeContext> context);

    // 定期轮询硬件状态的方法
    void poll_hardware_status();

    // 设定调压器参数到硬件
    bool apply_regulator_settings_to_hardware(uint8_t id, const ros2_interfaces::msg::VoltageRegulatorSettings& settings);

    // 设定回路参数到硬件
    bool apply_circuit_settings_to_hardware(uint8_t id, const ros2_interfaces::msg::CircuitSettings& settings);

private:
    StateManager* state_manager_;
    std::shared_ptr<ControlNodeContext> context_;
    rclcpp::TimerBase::SharedPtr polling_timer_;
};

#endif // HARDWARE_COORDINATOR_HPP
