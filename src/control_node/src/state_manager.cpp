#include "control_node/state_manager.hpp"
#include <stdexcept>

// --- 私有帮助函数 ---
size_t StateManager::circuit_id_to_index(uint8_t id) const
{
    if (id < 1 || id > NUM_CIRCUITS) {
        throw std::out_of_range("Circuit ID is out of range.");
    }
    return id - 1;
}
size_t StateManager::regulator_id_to_index(uint8_t id) const
{
    if (id < 1 || id > NUM_REGULATORS) {
        throw std::out_of_range("Regulator ID is out of range.");
    }
    return id - 1;
}
// ==========================================
// 写方法的实现
// ==========================================

StateManager::StateManager() {}

// --- 常规数据 ---
void StateManager::update_system_status(const ros2_interfaces::msg::SystemStatus& status)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    system_status_ = status;
}
void StateManager::update_circuit_status(uint8_t id, const ros2_interfaces::msg::CircuitStatus& status)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
        circuit_statuses_.at(circuit_id_to_index(id)) = status;
    } catch (const std::out_of_range& e) {}
}
void StateManager::update_regulator_status(uint8_t id, const ros2_interfaces::msg::RegulatorStatus& status)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
        regulator_statuses_.at(regulator_id_to_index(id)) = status;
    } catch (const std::out_of_range& e) {}
}
void StateManager::update_system_settings(const ros2_interfaces::msg::SystemSettings& settings)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    system_settings_ = settings;
}
void StateManager::update_circuit_settings(uint8_t id, const ros2_interfaces::msg::CircuitSettings& settings)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
        circuit_settings_.at(circuit_id_to_index(id)) = settings;
    } catch (const std::out_of_range& e) {}
}
void StateManager::update_regulator_settings(uint8_t id, const ros2_interfaces::msg::RegulatorSettings& settings)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
        regulator_settings_.at(regulator_id_to_index(id)) = settings;
    } catch (const std::out_of_range& e) {}
}
// --- 硬件数据合并更新 ---
void StateManager::update_circuit_status_from_hardware(uint8_t id, const ros2_interfaces::msg::HardwareCircuitStatus& hw_status)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    size_t idx = 0;
    try {
        idx = circuit_id_to_index(id);
        auto& status = circuit_statuses_.at(idx);

        // Update standard mapping
        status.header = hw_status.header;
        status.circuit_id = hw_status.circuit_id;

        // 直接将硬件的 Loop 状态映射过来（包含了下放到 Loop 里的 plc_control_mode）
        status.test_loop.hardware_loop_status = hw_status.test_loop;
        status.ref_loop.hardware_loop_status = hw_status.ref_loop;

    } catch (const std::out_of_range& e) {}
}

void StateManager::update_circuit_settings_from_hardware(uint8_t id, const ros2_interfaces::msg::HardwareCircuitSettings& hw_settings)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
        auto& settings = circuit_settings_.at(circuit_id_to_index(id));
        settings.test_loop.hardware_loop_settings = hw_settings.test_loop;
        settings.ref_loop.hardware_loop_settings = hw_settings.ref_loop;
    } catch (const std::out_of_range& e) {}
}

void StateManager::update_system_status_from_hardware(const ros2_interfaces::msg::HardwareSystemStatus& hw_status)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    // 仅更新 system_status_ 中的硬件子结构，不影响逻辑层写入的 connection_state 等
    system_status_.hardware_system_status = hw_status;
}

void StateManager::update_database_status(const ros2_interfaces::msg::DatabaseStatus& db_status)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    database_status_ = db_status;
}

// ==========================================
// 读方法的实现
// ==========================================
// --- 常规数据 ---
ros2_interfaces::msg::SystemStatus StateManager::get_system_status() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return system_status_;
}
ros2_interfaces::msg::CircuitStatus StateManager::get_circuit_status(uint8_t id) const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
        return circuit_statuses_.at(circuit_id_to_index(id));
    } catch (const std::out_of_range& e) {
        return ros2_interfaces::msg::CircuitStatus();
    }
}
ros2_interfaces::msg::RegulatorStatus StateManager::get_regulator_status(uint8_t id) const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
        return regulator_statuses_.at(regulator_id_to_index(id));
    } catch (const std::out_of_range& e) {
        return ros2_interfaces::msg::RegulatorStatus();
    }
}
ros2_interfaces::msg::SystemSettings StateManager::get_system_settings() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return system_settings_;
}
ros2_interfaces::msg::CircuitSettings StateManager::get_circuit_settings(uint8_t id) const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
        return circuit_settings_.at(circuit_id_to_index(id));
    } catch (const std::out_of_range& e) {
        return ros2_interfaces::msg::CircuitSettings();
    }
}
ros2_interfaces::msg::RegulatorSettings StateManager::get_regulator_settings(uint8_t id) const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
        return regulator_settings_.at(regulator_id_to_index(id));
    } catch (const std::out_of_range& e) {
        return ros2_interfaces::msg::RegulatorSettings();
    }
}

ros2_interfaces::msg::DatabaseStatus StateManager::get_database_status() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return database_status_;
}
