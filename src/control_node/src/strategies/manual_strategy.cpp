#include "control_node/strategies/manual_strategy.hpp"
#include "rclcpp/rclcpp.hpp" // For logging

ManualStrategy::ManualStrategy(
    std::shared_ptr<StateManager> state_manager,
    std::shared_ptr<HardwareCoordinator> hardware_coordinator)
    :
    state_manager_(state_manager),
    hardware_coordinator_(hardware_coordinator)
{}

void ManualStrategy::update()
{
    // 手动模式的逻辑
    // 也许什么都不做，只等待外部命令
    // 这里可以打印日志来确认它在运行
    // RCLCPP_INFO(rclcpp::get_logger("ManualStrategy"), "Manual strategy is active.");
}
