#ifndef I_CONTROLLER_STRATEGY_HPP
#define I_CONTROLLER_STRATEGY_HPP

#include "control_node/state_manager.hpp"
#include "control_node/hardware_coordinator.hpp"

class IControllerStrategy
{
public:
    virtual ~IControllerStrategy() = default;

    // 每个策略都必须实现这个核心的 update 方法
    virtual void update() = 0;

    // 返回策略的名称
    virtual const char* get_name() const = 0;
};

#endif // I_CONTROLLER_STRATEGY_HPP
