#ifndef I_HARDWARE_COORDINATOR_HPP
#define I_HARDWARE_COORDINATOR_HPP

#include "rclcpp/rclcpp.hpp" // IWYU pragma: keep
// --- 包含所有与硬件通信所需的消息类型 ---
#include "ros2_interfaces/msg/regulator_settings.hpp"
#include "ros2_interfaces/msg/hardware_circuit_settings.hpp" // 使用硬件特定的设置
#include "ros2_interfaces/msg/regulator_operation_command.hpp"
#include "ros2_interfaces/srv/regulator_breaker_command.hpp"
#include "ros2_interfaces/srv/circuit_breaker_command.hpp"
#include <cstdint>

// 定义一个通用的回调函数类型，用于报告异步操作的结果
using HardwareCallback = std::function<void(bool success, const std::string& message)>;

/**
 * @class IHardwareCoordinator
 * @brief 硬件协调器的接口，定义了与 hardware_node 交互的所有操作。
 *
 * 这个接口抽象了与硬件节点通信的细节。
 */
class IHardwareCoordinator
{
public:
    virtual ~IHardwareCoordinator() = default;

    // 连通测试
    virtual bool is_connected() const = 0;

    /**
     * @brief 将调压器设置发送到硬件节点。
     * @param id 调压器的ID。
     * @param settings 要应用的设置。
     * @return 如果成功发送并获得硬件节点确认，则返回true。
     */
    virtual void apply_regulator_settings_to_hardware(
        uint8_t id,
        const ros2_interfaces::msg::RegulatorSettings& settings,
        HardwareCallback callback) = 0;

    /**
     * @brief 将硬件回路设置发送到硬件节点。
     * @param id 回路的ID。
     * @param settings 要应用的硬件特定设置。
     * @return 如果成功发送并获得硬件节点确认，则返回true。
     */
    virtual void apply_circuit_settings_to_hardware(
        uint8_t id,
        const ros2_interfaces::msg::HardwareCircuitSettings& settings,
        HardwareCallback callback) = 0;

    // commands
    virtual void send_regulator_operation_command(
        const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr command_msg) = 0;
    virtual void execute_regulator_breaker_command(
        const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request> request,
        HardwareCallback callback) = 0;
    virtual void execute_circuit_breaker_command(
        const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request> request,
        HardwareCallback callback) = 0;

    /**
     * @brief 发送回路命令到硬件节点。
     * @param command_msg 包含ID和命令的消息。
     */
    virtual void send_clear_alarm() = 0;

    virtual void set_circuit_control_mode(uint8_t circuit_id, uint8_t mode) = 0; // 设为 手动/自动
    virtual void set_circuit_control_source(uint8_t circuit_id, uint8_t source) = 0; // 设为 试验/参考
};

#endif // I_HARDWARE_COORDINATOR_HPP
