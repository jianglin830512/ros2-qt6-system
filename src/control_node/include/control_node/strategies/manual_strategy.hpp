#ifndef MANUAL_STRATEGY_HPP
#define MANUAL_STRATEGY_HPP

#include "control_node/i_controller_strategy.hpp"

class ManualStrategy : public IControllerStrategy
{
public:
    ManualStrategy(
        std::shared_ptr<StateManager> state_manager,
        std::shared_ptr<HardwareCoordinator> hardware_coordinator);
    void update() override;
    const char* get_name() const override { return "Manual Mode"; }
private:
    std::shared_ptr<StateManager> state_manager_;
    std::shared_ptr<HardwareCoordinator> hardware_coordinator_;
};

#endif // MANUAL_STRATEGY_HPP
