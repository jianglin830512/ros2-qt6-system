#ifndef I_CONTROLLER_STRATEGY_HPP
#define I_CONTROLLER_STRATEGY_HPP

#include <memory>
#include "control_node/state_manager.hpp"
#include "control_node/i_hardware_coordinator.hpp"
#include "ros2_interfaces/msg/regulator_operation_command.hpp"

class IControllerStrategy
{
public:
    // 构造函数接收 circuit_id
    IControllerStrategy(
        uint8_t circuit_id,
        std::shared_ptr<StateManager> state_manager,
        std::shared_ptr<IHardwareCoordinator> hardware_coordinator
        ) : circuit_id_(circuit_id), state_manager_(state_manager), hardware_coordinator_(hardware_coordinator) {}

    virtual ~IControllerStrategy() = default;

    virtual void reset() = 0;

    // 核心循环逻辑
    virtual void update() = 0;

    // 处理手动命令（默认实现：不做任何事，手动模式会重写它）
    virtual void handle_manual_command(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg)
    {
        (void)msg;
    }

    virtual const char* get_name() const = 0;

protected:
    uint8_t circuit_id_;
    std::shared_ptr<StateManager> state_manager_;
    std::shared_ptr<IHardwareCoordinator> hardware_coordinator_;
};

#endif // I_CONTROLLER_STRATEGY_HPP
