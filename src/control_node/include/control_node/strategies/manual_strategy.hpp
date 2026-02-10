#ifndef MANUAL_STRATEGY_HPP
#define MANUAL_STRATEGY_HPP

#include "control_node/i_controller_strategy.hpp"
#include "rclcpp/time.hpp"

class ManualStrategy : public IControllerStrategy
{
public:
    ManualStrategy(
        uint8_t circuit_id,
        std::shared_ptr<StateManager> state_manager,
        std::shared_ptr<IHardwareCoordinator> hardware_coordinator);

    void reset() override;
    void update() override;

    // Command Handlers
    void handle_regulator_operation_command(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg) override;

    void handle_regulator_breaker_command(
        const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request>& request,
        StrategyCallback callback) override;

    void handle_circuit_breaker_command(
        const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request>& request,
        StrategyCallback callback) override;

    const char* get_name() const override { return "Manual Mode"; }

private:
    rclcpp::Time last_check_time_;
    const rclcpp::Duration check_interval_{std::chrono::seconds(1)};
};

#endif // MANUAL_STRATEGY_HPP
