#ifndef AUTO_CURRENT_STRATEGY_HPP
#define AUTO_CURRENT_STRATEGY_HPP

#include "control_node/i_controller_strategy.hpp"
#include "rclcpp/rclcpp.hpp"   // IWYU pragma: keep

class AutoCurrentStrategy : public IControllerStrategy
{
public:
    AutoCurrentStrategy(
        uint8_t circuit_id,
        std::shared_ptr<StateManager> state_manager,
        std::shared_ptr<IHardwareCoordinator> hardware_coordinator);

    void reset() override;
    void update() override;
    const char* get_name() const override { return "Auto Current Mode"; }

private:
    // 防止命令发送过快（命令冷却时间）
    rclcpp::Time last_command_time_;
    const rclcpp::Duration command_cooldown_{std::chrono::milliseconds(500)};

    // 简单的控制状态
    double last_error_ = 0.0;

    double integral_error_ = 0.0;
};

#endif // AUTO_CURRENT_STRATEGY_HPP
