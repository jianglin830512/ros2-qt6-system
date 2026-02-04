#include "hardware_node/tcp_hardware_driver.hpp"

using namespace std::chrono_literals;

TcpHardwareDriver::TcpHardwareDriver(rclcpp::Logger logger) : logger_(logger)
{
    initialize_default_states();
    RCLCPP_INFO(logger_, "TcpHardwareDriver initialized.");
}

TcpHardwareDriver::~TcpHardwareDriver() = default;

void TcpHardwareDriver::initialize_default_states()
{
    // Initialize default state for 2 regulators and 2 circuits
    for (uint8_t id = 1; id <= 2; ++id)
    {
        // Settings
        regulator_settings_map_[id] = ros2_interfaces::msg::RegulatorSettings();
        circuit_settings_map_[id] = ros2_interfaces::msg::HardwareCircuitSettings();

        // Status
        regulator_status_map_[id].regulator_id = id;
        circuit_status_map_[id].circuit_id = id;
    }
}

// --- 服务回调实现 ---

void TcpHardwareDriver::handle_set_hardware_regulator_settings_request(
    const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request> request,
    AsyncCallback callback)
{
    RCLCPP_INFO(logger_, "Simulating async TCP set regulator settings for ID: %u", request->settings.regulator_id);
    std::thread([this, request, callback]() {
        // TODO: Implement actual TCP logic here (Modbus/S7/etc)
        std::this_thread::sleep_for(500ms);
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (regulator_settings_map_.count(request->settings.regulator_id)) {
            regulator_settings_map_[request->settings.regulator_id] = request->settings;
            RCLCPP_INFO(logger_, "TCP: Regulator %u settings updated.", request->settings.regulator_id);
            callback(true, "Settings updated successfully.");
        } else {
            callback(false, "Invalid regulator ID.");
        }
    }).detach();
}

void TcpHardwareDriver::handle_set_hardware_circuit_settings_request(
    const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitSettings::Request> request,
    AsyncCallback callback)
{
    RCLCPP_INFO(logger_, "Simulating async TCP set circuit settings for ID: %u", request->settings.circuit_id);
    std::thread([this, request, callback]() {
        // TODO: Implement actual TCP logic here
        std::this_thread::sleep_for(500ms);
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (circuit_settings_map_.count(request->settings.circuit_id)) {
            circuit_settings_map_[request->settings.circuit_id] = request->settings;
            RCLCPP_INFO(logger_, "TCP: Circuit %u settings updated.", request->settings.circuit_id);
            callback(true, "Settings updated successfully.");
        } else {
            callback(false, "Invalid circuit ID.");
        }
    }).detach();
}

void TcpHardwareDriver::handle_regulator_breaker_command(const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request> request, AsyncCallback callback)
{
    RCLCPP_INFO(logger_, "Simulating async TCP regulator breaker command for ID: %u", request->regulator_id);
    std::thread([this, request, callback]() {
        // TODO: Implement actual TCP logic here
        std::this_thread::sleep_for(200ms);
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (regulator_status_map_.count(request->regulator_id)) {
            regulator_status_map_[request->regulator_id].breaker_closed_switch_ack = (request->command == 1);
            RCLCPP_INFO(logger_, "TCP: Regulator %u breaker command executed.", request->regulator_id);
            callback(true, "Regulator breaker command sent.");
        } else {
            callback(false, "Invalid regulator ID.");
        }
    }).detach();
}

void TcpHardwareDriver::handle_circuit_breaker_command(const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request> request, AsyncCallback callback)
{
    RCLCPP_INFO(logger_, "Simulating async TCP circuit breaker command for ID: %u", request->circuit_id);
    std::thread([this, request, callback]() {
        // TODO: Implement actual TCP logic here
        std::this_thread::sleep_for(200ms);
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (circuit_status_map_.count(request->circuit_id)) {
            if(request->command == 1) { // test_loop close
                circuit_status_map_[request->circuit_id].test_loop.breaker_closed_switch_ack = true;
            } else if(request->command == 2){ // test_loop open
                circuit_status_map_[request->circuit_id].test_loop.breaker_closed_switch_ack = false;
            }else if(request->command == 3) { // ref_loop close
                circuit_status_map_[request->circuit_id].ref_loop.breaker_closed_switch_ack = true;
            }else if(request->command == 4){ // ref_loop open
                circuit_status_map_[request->circuit_id].ref_loop.breaker_closed_switch_ack = false;
            }
            RCLCPP_INFO(logger_, "TCP: Circuit %u breaker command executed.", request->circuit_id);
            callback(true, "Circuit breaker command sent.");
        } else {
            callback(false, "Invalid circuit ID.");
        }
    }).detach();
}

// [新增] 新的接口实现
void TcpHardwareDriver::handle_set_control_mode(
    const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitControlMode::Request> request,
    AsyncCallback callback)
{
    RCLCPP_INFO(logger_, "Simulating async TCP set control mode for Circuit: %u", request->circuit_id);
    std::thread([this, request, callback]() {
        // TODO: Implement actual TCP logic here to write PLC Mode Register
        std::this_thread::sleep_for(100ms);
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (circuit_status_map_.count(request->circuit_id)) {
            circuit_status_map_[request->circuit_id].plc_control_mode = request->mode;
            RCLCPP_INFO(logger_, "TCP: Circuit %u control mode set to %u.", request->circuit_id, request->mode);
            callback(true, "Control Mode set.");
        } else {
            callback(false, "Invalid circuit ID.");
        }
    }).detach();
}

void TcpHardwareDriver::handle_set_control_source(
    const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitControlSource::Request> request,
    AsyncCallback callback)
{
    RCLCPP_INFO(logger_, "Simulating async TCP set control source for Circuit: %u", request->circuit_id);
    std::thread([this, request, callback]() {
        // TODO: Implement actual TCP logic here to write PLC Source Register
        std::this_thread::sleep_for(100ms);
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (circuit_status_map_.count(request->circuit_id)) {
            circuit_status_map_[request->circuit_id].plc_control_source = request->source;
            RCLCPP_INFO(logger_, "TCP: Circuit %u control source set to %u.", request->circuit_id, request->source);
            callback(true, "Control Source set.");
        } else {
            callback(false, "Invalid circuit ID.");
        }
    }).detach();
}


// --- 话题回调实现 ---

void TcpHardwareDriver::handle_regulator_operation_command(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    RCLCPP_INFO(logger_, "TCP: Received operation command %u for regulator %u.", msg->command, msg->regulator_id);
    // TODO: Write to hardware register directly (no async callback for topic sub)
}

void TcpHardwareDriver::handle_clear_alarm()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    RCLCPP_INFO(logger_, "TCP: Clearing all hardware alarms.");
    // TODO: Write to hardware reset coil
    for(auto& pair : circuit_status_map_) {
        pair.second.test_loop.over_current_on = false;
        pair.second.ref_loop.over_current_on = false;
    }
    for(auto& pair : regulator_status_map_) {
        pair.second.over_current_on = false;
        pair.second.over_voltage_on = false;
    }
}

// --- 数据检索实现 ---

bool TcpHardwareDriver::get_regulator_status(uint8_t regulator_id, ros2_interfaces::msg::RegulatorStatus& status)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (regulator_status_map_.count(regulator_id)) {
        // TODO: In real driver, this map is updated by a separate polling thread reading from TCP
        // Here we simulate values
        regulator_status_map_[regulator_id].voltage_reading = 100.0 + (rand() % 20);
        regulator_status_map_[regulator_id].current_reading = 10.0 + (rand() % 5);
        status = regulator_status_map_[regulator_id];
        return true;
    }
    return false;
}

bool TcpHardwareDriver::get_circuit_status(uint8_t circuit_id, ros2_interfaces::msg::HardwareCircuitStatus& status)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (circuit_status_map_.count(circuit_id)) {
        // TODO: In real driver, read PLC mode/source/currents from TCP
        circuit_status_map_[circuit_id].test_loop.current = 50.0 + (rand() % 10);
        circuit_status_map_[circuit_id].ref_loop.current = 50.0 + (rand() % 10);
        status = circuit_status_map_[circuit_id];
        return true;
    }
    return false;
}

bool TcpHardwareDriver::get_regulator_settings(uint8_t regulator_id, ros2_interfaces::msg::RegulatorSettings& settings)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (regulator_settings_map_.count(regulator_id)) {
        settings = regulator_settings_map_[regulator_id];
        return true;
    }
    return false;
}

bool TcpHardwareDriver::get_circuit_settings(uint8_t circuit_id, ros2_interfaces::msg::HardwareCircuitSettings& settings)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (circuit_settings_map_.count(circuit_id)) {
        settings = circuit_settings_map_[circuit_id];
        return true;
    }
    return false;
}
