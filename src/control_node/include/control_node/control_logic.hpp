#ifndef CONTROL_LOGIC_HPP
#define CONTROL_LOGIC_HPP

#include <memory>
#include <string>
#include <functional> // 包含 std::function

#include "rclcpp/rclcpp.hpp"   // IWYU pragma: keep
#include "ros2_interfaces/srv/set_system_settings.hpp"
#include "ros2_interfaces/srv/set_regulator_settings.hpp"
#include "ros2_interfaces/srv/set_circuit_settings.hpp"
#include "control_node/i_controller_strategy.hpp"
#include "control_node/persistence_coordinator.hpp"

    // === 前向声明 ===
    class StateManager;
class HardwareCoordinator;
struct ControlNodeContext;

// 定义一个通用的回调函数类型，用于从ControlLogic向ControlNode报告最终结果
using LogicResultCallback = std::function<void(bool success, const std::string& message)>;

class ControlLogic
{
public:
    ControlLogic(std::shared_ptr<ControlNodeContext> context);

    void update();
    void switch_mode(const std::string& new_mode);

    // *** 修改点: 所有 handle_ 方法都改为 void + callback 模式 ***
    void handle_set_system_settings_request(
        const std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Request>& request,
        LogicResultCallback callback);

    void handle_set_regulator_settings_request(
        const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request>& request,
        LogicResultCallback callback);

    void handle_set_circuit_settings_request(
        const std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Request>& request,
        LogicResultCallback callback);

private:
    std::shared_ptr<ControlNodeContext> context_;
    std::shared_ptr<StateManager> state_manager_;
    std::shared_ptr<HardwareCoordinator> hardware_coordinator_;
    std::unique_ptr<PersistenceCoordinator> persistence_coordinator_;
    std::unique_ptr<IControllerStrategy> active_strategy_;
};

#endif // CONTROL_LOGIC_HPP
