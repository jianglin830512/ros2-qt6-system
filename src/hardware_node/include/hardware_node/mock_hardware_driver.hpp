#ifndef MOCK_HARDWARE_DRIVER_HPP_
#define MOCK_HARDWARE_DRIVER_HPP_
#include "hardware_node/i_hardware_driver.hpp"
#include "rclcpp/rclcpp.hpp" // IWYU pragma: keep , for logger
#include <memory>
// 前向声明模拟设备类
class MockDevice;
class MockHardwareDriver : public IHardwareDriver
{
public:
    explicit MockHardwareDriver(rclcpp::Logger logger);
    ~MockHardwareDriver() override;

    // --- 服务处理器 ---
    void handle_set_hardware_regulator_settings_request(const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request> request, AsyncCallback callback) override;
    void handle_set_hardware_circuit_settings_request(const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitSettings::Request> request, AsyncCallback callback) override;
    void handle_regulator_breaker_command(const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request> request, AsyncCallback callback) override;
    void handle_circuit_breaker_command(const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request> request, AsyncCallback callback) override;

    // [新增]
    void handle_set_control_mode(const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitControlMode::Request> request, AsyncCallback callback) override;
    void handle_set_control_source(const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitControlSource::Request> request, AsyncCallback callback) override;

    // --- 话题处理器 ---
    void handle_regulator_operation_command(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg) override;
    void handle_clear_alarm() override;

    // --- 数据检索 ---
    bool get_regulator_status(uint8_t regulator_id, ros2_interfaces::msg::RegulatorStatus& status) override;
    bool get_circuit_status(uint8_t circuit_id, ros2_interfaces::msg::HardwareCircuitStatus& status) override;
    bool get_regulator_settings(uint8_t regulator_id, ros2_interfaces::msg::RegulatorSettings& settings) override;
    bool get_circuit_settings(uint8_t circuit_id, ros2_interfaces::msg::HardwareCircuitSettings& settings) override;
private:
    rclcpp::Logger logger_;
    std::unique_ptr<MockDevice> device_; // 核心模拟逻辑
};
#endif
