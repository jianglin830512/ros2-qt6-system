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
    void handle_manual_command(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg) override;
    const char* get_name() const override { return "Manual Mode"; }

private:
    rclcpp::Time last_check_time_;
};

#endif // MANUAL_STRATEGY_HPP
