#ifndef AUTO_CURRENT_STRATEGY_HPP
#define AUTO_CURRENT_STRATEGY_HPP

#include "control_node/i_controller_strategy.hpp"
#include "rclcpp/rclcpp.hpp"

class AutoCurrentStrategy : public IControllerStrategy
{
public:
    AutoCurrentStrategy(
        uint8_t circuit_id,
        std::shared_ptr<StateManager> state_manager,
        std::shared_ptr<IHardwareCoordinator> hardware_coordinator);

    void reset() override;
    void update() override;

    // Command Handlers (Blocked in Auto Mode)
    void handle_regulator_operation_command(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg) override;

    void handle_regulator_breaker_command(
        const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request>& request,
        StrategyCallback callback) override;

    void handle_circuit_breaker_command(
        const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request>& request,
        StrategyCallback callback) override;

    const char* get_name() const override { return "Auto Current Mode"; }

private:
    rclcpp::Time last_sync_time_;
    const rclcpp::Duration sync_interval_{std::chrono::milliseconds(500)};
};

#endif // AUTO_CURRENT_STRATEGY_HPP
