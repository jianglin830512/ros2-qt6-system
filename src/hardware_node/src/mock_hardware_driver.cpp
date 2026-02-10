#include "hardware_node/mock_hardware_driver.hpp"
#include "hardware_node/mock_device.hpp"

MockHardwareDriver::MockHardwareDriver(rclcpp::Logger logger)
    : logger_(logger), device_(std::make_unique<MockDevice>())
{
    RCLCPP_INFO(logger_, "MockHardwareDriver with PLC-style logic started.");
}

MockHardwareDriver::~MockHardwareDriver() = default;

void MockHardwareDriver::update()
{
    // MockDevice 自带线程，不需要轮询驱动
}

// --- Service Handlers ---

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
    // 映射所有设置字段
    s.id = request->settings.regulator_id;
    s.over_voltage_limit = request->settings.over_voltage_v;
    s.over_current_limit = request->settings.over_current_a;
    s.speed_up_percent = request->settings.voltage_up_speed_percent;
    s.speed_down_percent = request->settings.voltage_down_speed_percent;
    s.ovp_enabled = request->settings.over_voltage_protection_mode;

    device_->update_reg_settings(request->settings.regulator_id, s);
    callback(true, "Mock: Regulator settings updated completely.");
}

void MockHardwareDriver::handle_set_hardware_circuit_settings_request(
    const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitSettings::Request> request, AsyncCallback callback)
{
    MockDevice::LoopState test_s, ref_s;

    // 映射试验回路设置
    test_s.max_current_setting = request->settings.test_loop.max_current_a;
    test_s.start_current_setting = request->settings.test_loop.start_current_a;
    test_s.current_change_range = request->settings.test_loop.current_change_range_percent;
    test_s.ct_ratio = request->settings.test_loop.ct_ratio;

    // 映射参考回路设置
    ref_s.max_current_setting = request->settings.ref_loop.max_current_a;
    ref_s.start_current_setting = request->settings.ref_loop.start_current_a;
    ref_s.current_change_range = request->settings.ref_loop.current_change_range_percent;
    ref_s.ct_ratio = request->settings.ref_loop.ct_ratio;

    device_->update_circ_settings(request->settings.circuit_id, test_s, ref_s);
    callback(true, "Mock: Circuit settings updated completely.");
}

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

// --- Data Getters ---

bool MockHardwareDriver::get_regulator_status(uint8_t regulator_id, ros2_interfaces::msg::RegulatorStatus& status)
{
    auto dev = device_->get_reg(regulator_id);
    status.regulator_id = dev.id;
    status.voltage_reading = dev.voltage;
    status.current_reading = dev.current;
    status.breaker_closed_switch_ack = dev.breaker_closed;
    status.breaker_opened_switch_ack = !dev.breaker_closed;
    status.voltage_direction = dev.direction; // 1, -1, 0
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

    // PLC 模式反馈
    status.plc_control_mode = dev.plc_mode;
    status.plc_control_source = dev.plc_source;

    // 辅助 lambda 填充 LoopStatus
    auto fill_loop = [](ros2_interfaces::msg::HardwareLoopStatus& loop_msg, const MockDevice::LoopState& loop_dev) {
        loop_msg.current = loop_dev.current;
        loop_msg.breaker_closed_switch_ack = loop_dev.breaker_closed;
        loop_msg.breaker_opened_switch_ack = !loop_dev.breaker_closed;
        loop_msg.over_current_on = loop_dev.over_current_alarm;
        // 复制温度数组 (float -> double/float64)
        for(int i=0; i<16; ++i) {
            loop_msg.temperature_array[i] = loop_dev.temperatures[i];
        }
    };

    fill_loop(status.test_loop, dev.test_loop);
    fill_loop(status.ref_loop, dev.ref_loop);

    return true;
}

bool MockHardwareDriver::get_regulator_settings(uint8_t regulator_id, ros2_interfaces::msg::RegulatorSettings& settings) {
    auto dev = device_->get_reg(regulator_id);
    settings.regulator_id = regulator_id;

    settings.over_voltage_v = dev.over_voltage_limit;
    settings.over_current_a = dev.over_current_limit;
    settings.voltage_up_speed_percent = dev.speed_up_percent;
    settings.voltage_down_speed_percent = dev.speed_down_percent;
    settings.over_voltage_protection_mode = dev.ovp_enabled;

    return true;
}

bool MockHardwareDriver::get_circuit_settings(uint8_t circuit_id, ros2_interfaces::msg::HardwareCircuitSettings& settings) {
    auto dev = device_->get_circ(circuit_id);
    settings.circuit_id = circuit_id;

    // 辅助 lambda 填充 LoopSettings
    auto fill_settings = [](ros2_interfaces::msg::HardwareLoopSettings& l_msg, const MockDevice::LoopState& l_dev) {
        l_msg.max_current_a = l_dev.max_current_setting;
        l_msg.start_current_a = l_dev.start_current_setting;
        l_msg.current_change_range_percent = l_dev.current_change_range;
        l_msg.ct_ratio = l_dev.ct_ratio;
    };

    fill_settings(settings.test_loop, dev.test_loop);
    fill_settings(settings.ref_loop, dev.ref_loop);

    return true;
}
