#ifndef I_HARDWARE_DRIVER_HPP_
#define I_HARDWARE_DRIVER_HPP_
#include <string>
#include <functional>
#include <memory>
// --- 包含所有需要的消息和服务头文件 ---
#include "ros2_interfaces/msg/hardware_circuit_settings.hpp"
#include "ros2_interfaces/msg/hardware_circuit_status.hpp"
#include "ros2_interfaces/msg/regulator_operation_command.hpp"
#include "ros2_interfaces/msg/regulator_settings.hpp"
#include "ros2_interfaces/msg/regulator_status.hpp"
#include "ros2_interfaces/srv/circuit_breaker_command.hpp"
#include "ros2_interfaces/srv/regulator_breaker_command.hpp"
#include "ros2_interfaces/srv/set_hardware_circuit_settings.hpp"
#include "ros2_interfaces/srv/set_regulator_settings.hpp"
// [新增] 新的控制模式和服务头文件
#include "ros2_interfaces/srv/set_hardware_circuit_control_mode.hpp"
#include "ros2_interfaces/srv/set_hardware_circuit_control_source.hpp"
// A callback function to be called when an async operation completes.
using AsyncCallback = std::function<void(bool success, const std::string& message)>;
class IHardwareDriver
{
public:
    virtual ~IHardwareDriver() = default;

    // --- 服务的异步处理器 ---

    // (设置服务)
    virtual void handle_set_hardware_regulator_settings_request(
        const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request> request,
        AsyncCallback callback) = 0;

    virtual void handle_set_hardware_circuit_settings_request(
        const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitSettings::Request> request,
        AsyncCallback callback) = 0;

    // (命令服务)
    virtual void handle_regulator_breaker_command(
        const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request> request,
        AsyncCallback callback) = 0;

    virtual void handle_circuit_breaker_command(
        const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request> request,
        AsyncCallback callback) = 0;

    // [新增] PLC 控制模式设置
    virtual void handle_set_control_mode(
        const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitControlMode::Request> request,
        AsyncCallback callback) = 0;

    // [新增] PLC 控制源设置
    virtual void handle_set_control_source(
        const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitControlSource::Request> request,
        AsyncCallback callback) = 0;

    // --- 话题回调的同步处理器 ---
    virtual void handle_regulator_operation_command(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg) = 0;
    virtual void handle_clear_alarm() = 0;

    // --- 用于轮询的数据检索方法 ---
    virtual bool get_regulator_status(uint8_t regulator_id, ros2_interfaces::msg::RegulatorStatus& status) = 0;
    virtual bool get_circuit_status(uint8_t circuit_id, ros2_interfaces::msg::HardwareCircuitStatus& status) = 0;
    virtual bool get_regulator_settings(uint8_t regulator_id, ros2_interfaces::msg::RegulatorSettings& settings) = 0;
    virtual bool get_circuit_settings(uint8_t circuit_id, ros2_interfaces::msg::HardwareCircuitSettings& settings) = 0;
};
#endif // I_HARDWARE_DRIVER_HPP_
