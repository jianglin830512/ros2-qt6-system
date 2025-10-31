#ifndef AUTO_CURRENT_STRATEGY_HPP
#define AUTO_CURRENT_STRATEGY_HPP

#include "control_node/i_controller_strategy.hpp"

class AutoCurrentStrategy : public IControllerStrategy
{
public:
    AutoCurrentStrategy(
        std::shared_ptr<StateManager> state_manager,
        std::shared_ptr<HardwareCoordinator> hardware_coordinator);
    void update() override;
    const char* get_name() const override { return "Auto Current Mode"; }
private:
    std::shared_ptr<StateManager> state_manager_;
    std::shared_ptr<HardwareCoordinator> hardware_coordinator_;
};

#endif // AUTO_CURRENT_STRATEGY_HPP
