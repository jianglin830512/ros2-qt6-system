#include "hardware_node/mock_hardware_driver.hpp"
#include "hardware_node/mock_device.hpp"

MockHardwareDriver::MockHardwareDriver(rclcpp::Logger logger)
    : logger_(logger), device_(std::make_unique<MockDevice>())
{
    RCLCPP_INFO(logger_, "MockHardwareDriver with PLC-style logic started.");
}

MockHardwareDriver::~MockHardwareDriver() = default;

// --- 服务：指令下发 ---

void MockHardwareDriver::handle_regulator_breaker_command(
    const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request> request, AsyncCallback callback)
{
    device_->set_regulator_breaker(request->regulator_id, request->command == 1);
    callback(true, "Mock: Regulator breaker command processed.");
}

void MockHardwareDriver::handle_circuit_breaker_command(
    const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request> request, AsyncCallback callback)
{
    device_->set_loop_breaker(request->circuit_id, request->command);
    callback(true, "Mock: Circuit breaker command processed.");
}

void MockHardwareDriver::handle_regulator_operation_command(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg)
{
    device_->set_regulator_op(msg->regulator_id, msg->command);
}

void MockHardwareDriver::handle_set_hardware_regulator_settings_request(
    const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request> request, AsyncCallback callback)
{
    MockDevice::RegulatorState s;
    s.over_voltage_limit = request->settings.over_voltage_v;
    s.speed_percent = request->settings.voltage_up_speed_percent;
    s.ovp_enabled = request->settings.over_voltage_protection_mode;
    device_->update_reg_settings(request->settings.regulator_id, s);
    callback(true, "Mock: Regulator settings updated.");
}

void MockHardwareDriver::handle_set_hardware_circuit_settings_request(
    const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitSettings::Request> request, AsyncCallback callback)
{
    device_->update_circ_settings(request->settings.circuit_id,
                                  request->settings.test_loop.max_current_a,
                                  request->settings.ref_loop.max_current_a);
    callback(true, "Mock: Circuit settings updated.");
}

// [新增] 适配 IHardwareDriver 的新接口，替代旧的 handle_circuit_mode_command
void MockHardwareDriver::handle_set_control_mode(
    const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitControlMode::Request> request, AsyncCallback callback)
{
    device_->set_plc_mode(request->circuit_id, request->mode);
    callback(true, "Mock: PLC Control Mode updated.");
}

void MockHardwareDriver::handle_set_control_source(
    const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitControlSource::Request> request, AsyncCallback callback)
{
    device_->set_plc_source(request->circuit_id, request->source);
    callback(true, "Mock: PLC Control Source updated.");
}

void MockHardwareDriver::handle_clear_alarm() {
    device_->clear_alarms();
}

// --- 数据检索：状态回传 ---

bool MockHardwareDriver::get_regulator_status(uint8_t regulator_id, ros2_interfaces::msg::RegulatorStatus& status)
{
    auto dev = device_->get_reg(regulator_id);
    status.regulator_id = dev.id;
    status.voltage_reading = dev.voltage;
    status.current_reading = dev.current;
    status.breaker_closed_switch_ack = dev.breaker_closed;
    status.breaker_opened_switch_ack = !dev.breaker_closed;
    status.voltage_direction = dev.direction;
    status.over_voltage_on = dev.over_voltage_alarm;
    status.over_current_on = dev.over_current_alarm;
    status.upper_limit_switch_on = dev.upper_limit_on;
    status.lower_limit_switch_on = dev.lower_limit_on;
    return true;
}

bool MockHardwareDriver::get_circuit_status(uint8_t circuit_id, ros2_interfaces::msg::HardwareCircuitStatus& status)
{
    auto dev = device_->get_circ(circuit_id);
    status.circuit_id = dev.id;

    // 填充 PLC 模式反馈
    status.plc_control_mode = dev.plc_mode;
    status.plc_control_source = dev.plc_source;

    // 填充试验回路
    status.test_loop.current = dev.test_loop.current;
    status.test_loop.breaker_closed_switch_ack = dev.test_loop.breaker_closed;
    status.test_loop.breaker_opened_switch_ack = !dev.test_loop.breaker_closed;
    status.test_loop.over_current_on = dev.test_loop.over_current_alarm;
    std::copy(std::begin(dev.test_loop.temperatures), std::end(dev.test_loop.temperatures), status.test_loop.temperature_array.begin());

    // 填充参考回路
    status.ref_loop.current = dev.ref_loop.current;
    status.ref_loop.breaker_closed_switch_ack = dev.ref_loop.breaker_closed;
    status.ref_loop.breaker_opened_switch_ack = !dev.ref_loop.breaker_closed;
    status.ref_loop.over_current_on = dev.ref_loop.over_current_alarm;
    std::copy(std::begin(dev.ref_loop.temperatures), std::end(dev.ref_loop.temperatures), status.ref_loop.temperature_array.begin());

    return true;
}

bool MockHardwareDriver::get_regulator_settings(uint8_t regulator_id, ros2_interfaces::msg::RegulatorSettings& settings) {
    auto dev = device_->get_reg(regulator_id);
    settings.regulator_id = regulator_id;
    settings.over_voltage_v = dev.over_voltage_limit;
    settings.voltage_up_speed_percent = dev.speed_percent;
    settings.over_voltage_protection_mode = dev.ovp_enabled;
    return true;
}

bool MockHardwareDriver::get_circuit_settings(uint8_t circuit_id, ros2_interfaces::msg::HardwareCircuitSettings& settings) {
    auto dev = device_->get_circ(circuit_id);
    settings.circuit_id = circuit_id;
    settings.test_loop.max_current_a = dev.test_loop.max_current_setting;
    settings.ref_loop.max_current_a = dev.ref_loop.max_current_setting;
    return true;
}
