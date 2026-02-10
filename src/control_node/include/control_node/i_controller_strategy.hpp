#ifndef I_CONTROLLER_STRATEGY_HPP
#define I_CONTROLLER_STRATEGY_HPP

#include <memory>
#include <string>
#include <functional>
#include "control_node/state_manager.hpp"
#include "control_node/i_hardware_coordinator.hpp"
#include "ros2_interfaces/msg/regulator_operation_command.hpp"
#include "ros2_interfaces/srv/regulator_breaker_command.hpp"
#include "ros2_interfaces/srv/circuit_breaker_command.hpp"

// Define a generic callback type to avoid circular dependency with ControlLogic
using StrategyCallback = std::function<void(bool success, const std::string& message)>;

class IControllerStrategy
{
public:
    IControllerStrategy(
        uint8_t circuit_id,
        std::shared_ptr<StateManager> state_manager,
        std::shared_ptr<IHardwareCoordinator> hardware_coordinator
        ) : circuit_id_(circuit_id),
        state_manager_(state_manager),
        hardware_coordinator_(hardware_coordinator),
        current_plc_control_mode_(0),
        current_plc_control_source_(0)
    {}

    virtual ~IControllerStrategy() = default;

    virtual void reset() = 0;

    // Core logic loop
    virtual void update() = 0;

    // --- State Synchronization ---
    // Called by ControlLogic when new hardware status is available
    virtual void update_plc_status(uint8_t plc_mode, uint8_t plc_source)
    {
        current_plc_control_mode_ = plc_mode;
        current_plc_control_source_ = plc_source;
    }

    virtual const char* get_name() const = 0;

    // --- Command Handlers (Gatekeepers) ---

    // 1. Regulator Operation (Voltage Up/Down/Stop) - Changed name as requested
    virtual void handle_regulator_operation_command(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg) = 0;

    // 2. Regulator Breaker Command
    virtual void handle_regulator_breaker_command(
        const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request>& request,
        StrategyCallback callback) = 0;

    // 3. Circuit Breaker Command
    virtual void handle_circuit_breaker_command(
        const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request>& request,
        StrategyCallback callback) = 0;

protected:
    uint8_t circuit_id_;
    std::shared_ptr<StateManager> state_manager_;
    std::shared_ptr<IHardwareCoordinator> hardware_coordinator_;

    // Current feedback from PLC
    uint8_t current_plc_control_mode_;
    uint8_t current_plc_control_source_;
};

#endif // I_CONTROLLER_STRATEGY_HPP
