#ifndef TCP_HARDWARE_DRIVER_HPP_
#define TCP_HARDWARE_DRIVER_HPP_

#include "hardware_node/i_hardware_driver.hpp"
#include <map>
#include <mutex>
#include "rclcpp/rclcpp.hpp" // IWYU pragma: keep , for logger

class TcpHardwareDriver : public IHardwareDriver
{
public:
    explicit TcpHardwareDriver(rclcpp::Logger logger);
    ~TcpHardwareDriver() override;

    // --- 服务的异步处理器 ---
    // (设置服务)
    void handle_set_hardware_regulator_settings_request(
        const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request> request,
        AsyncCallback callback) override;

    void handle_set_hardware_circuit_settings_request(
        const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitSettings::Request> request,
        AsyncCallback callback) override;

    // (命令服务)
    void handle_regulator_breaker_command(
        const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request> request,
        AsyncCallback callback) override;

    void handle_circuit_breaker_command(
        const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request> request,
        AsyncCallback callback) override;

    // [新增] PLC 控制模式和源 (替代旧的 circuit_mode_command)
    void handle_set_control_mode(
        const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitControlMode::Request> request,
        AsyncCallback callback) override;

    void handle_set_control_source(
        const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitControlSource::Request> request,
        AsyncCallback callback) override;


    // --- 话题回调的同步处理器 ---
    void handle_regulator_operation_command(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg) override;
    void handle_clear_alarm() override;

    // --- 用于轮询的数据检索方法 ---
    bool get_regulator_status(uint8_t regulator_id, ros2_interfaces::msg::RegulatorStatus& status) override;
    bool get_circuit_status(uint8_t circuit_id, ros2_interfaces::msg::HardwareCircuitStatus& status) override;
    bool get_regulator_settings(uint8_t regulator_id, ros2_interfaces::msg::RegulatorSettings& settings) override;
    bool get_circuit_settings(uint8_t circuit_id, ros2_interfaces::msg::HardwareCircuitSettings& settings) override;

private:
    void initialize_default_states();

    std::mutex data_mutex_;
    rclcpp::Logger logger_;

    // 模拟的硬件状态
    std::map<uint8_t, ros2_interfaces::msg::RegulatorStatus> regulator_status_map_;
    std::map<uint8_t, ros2_interfaces::msg::HardwareCircuitStatus> circuit_status_map_;
    std::map<uint8_t, ros2_interfaces::msg::RegulatorSettings> regulator_settings_map_;
    std::map<uint8_t, ros2_interfaces::msg::HardwareCircuitSettings> circuit_settings_map_;
};

#endif // TCP_HARDWARE_DRIVER_HPP_
