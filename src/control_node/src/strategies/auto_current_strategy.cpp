#include "control_node/strategies/auto_current_strategy.hpp"
#include "rclcpp/rclcpp.hpp" // For logging

AutoCurrentStrategy::AutoCurrentStrategy(
    std::shared_ptr<StateManager> state_manager,
    std::shared_ptr<HardwareCoordinator> hardware_coordinator
    ):
    state_manager_(state_manager),
    hardware_coordinator_(hardware_coordinator)
{}

void AutoCurrentStrategy::update()
{
    // 恒流模式的逻辑
    // 1. 从 state_manager_ 获取目标电流和实际电流
    auto settings = state_manager_->get_system_settings();
    auto status = state_manager_->get_circuit_status(1);

    // float target_current = settings.target_current; // 假设有这个字段
    // float actual_current = status.test_current;

    // 2. 实现PID或其他控制算法
    // if (actual_current < target_current) {
    //     // 3. 调用 hardware_coordinator_ 来执行操作
    //     hardware_coordinator_->increase_voltage();
    // }

    // RCLCPP_INFO(rclcpp::get_logger("AutoCurrentStrategy"), "Auto Current strategy running...");
}
