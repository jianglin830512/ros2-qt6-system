#include "control_node/state_manager.hpp"
#include <stdexcept>

StateManager::StateManager() {}

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
//               写方法的实现
// ==========================================

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


// --- 硬件原始数据 ---
void StateManager::update_hardware_circuit_status(uint8_t id, const ros2_interfaces::msg::HardwareCircuitStatus& status)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
        hardware_circuit_statuses_.at(circuit_id_to_index(id)) = status;
    } catch (const std::out_of_range& e) {}
}

void StateManager::update_hardware_regulator_status(uint8_t id, const ros2_interfaces::msg::RegulatorStatus& status)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
        hardware_regulator_statuses_.at(regulator_id_to_index(id)) = status;
    } catch (const std::out_of_range& e) {}
}

void StateManager::update_hardware_circuit_settings(uint8_t id, const ros2_interfaces::msg::HardwareCircuitSettings& settings)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
        hardware_circuit_settings_.at(circuit_id_to_index(id)) = settings;
    } catch (const std::out_of_range& e) {}
}

void StateManager::update_hardware_regulator_settings(uint8_t id, const ros2_interfaces::msg::RegulatorSettings& settings)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
        hardware_regulator_settings_.at(regulator_id_to_index(id)) = settings;
    } catch (const std::out_of_range& e) {}
}


// ==========================================
//               读方法的实现
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

// --- 硬件原始数据 ---
ros2_interfaces::msg::HardwareCircuitStatus StateManager::get_hardware_circuit_status(uint8_t id) const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
        return hardware_circuit_statuses_.at(circuit_id_to_index(id));
    } catch (const std::out_of_range& e) {
        return ros2_interfaces::msg::HardwareCircuitStatus();
    }
}

ros2_interfaces::msg::RegulatorStatus StateManager::get_hardware_regulator_status(uint8_t id) const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
        return hardware_regulator_statuses_.at(regulator_id_to_index(id));
    } catch (const std::out_of_range& e) {
        return ros2_interfaces::msg::RegulatorStatus();
    }
}

ros2_interfaces::msg::HardwareCircuitSettings StateManager::get_hardware_circuit_settings(uint8_t id) const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
        return hardware_circuit_settings_.at(circuit_id_to_index(id));
    } catch (const std::out_of_range& e) {
        return ros2_interfaces::msg::HardwareCircuitSettings();
    }
}

ros2_interfaces::msg::RegulatorSettings StateManager::get_hardware_regulator_settings(uint8_t id) const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
        return hardware_regulator_settings_.at(regulator_id_to_index(id));
    } catch (const std::out_of_range& e) {
        return ros2_interfaces::msg::RegulatorSettings();
    }
}

