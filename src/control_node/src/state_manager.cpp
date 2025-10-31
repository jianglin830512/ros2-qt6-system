#include "control_node/state_manager.hpp"
#include <stdexcept> // 用于抛出异常

StateManager::StateManager() {}

// --- 私有帮助函数 ---
// 将外部使用的 ID (通常从1开始) 转换为数组索引 (从0开始)
// 并进行边界检查，保证内存安全
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


// --- 写方法的实现 ---

void StateManager::update_circuit_status(uint8_t id, const ros2_interfaces::msg::CircuitStatus& status)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
        circuit_statuses_.at(circuit_id_to_index(id)) = status;
    } catch (const std::out_of_range& e) {
        // 在这里可以添加日志记录
    }
}

void StateManager::update_regulator_status(uint8_t id, const ros2_interfaces::msg::VoltageRegulatorStatus& status)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
        regulator_statuses_.at(regulator_id_to_index(id)) = status;
    } catch (const std::out_of_range& e) {
        // 日志
    }
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
    } catch (const std::out_of_range& e) {
        // 日志
    }
}

void StateManager::update_regulator_settings(uint8_t id, const ros2_interfaces::msg::VoltageRegulatorSettings& settings)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
        regulator_settings_.at(regulator_id_to_index(id)) = settings;
    } catch (const std::out_of_range& e) {
        // 日志
    }
}


// --- 读方法的实现 ---

ros2_interfaces::msg::CircuitStatus StateManager::get_circuit_status(uint8_t id) const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
        return circuit_statuses_.at(circuit_id_to_index(id));
    } catch (const std::out_of_range& e) {
        // 如果ID无效，返回一个默认构造的对象
        return ros2_interfaces::msg::CircuitStatus();
    }
}

ros2_interfaces::msg::VoltageRegulatorStatus StateManager::get_regulator_status(uint8_t id) const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
        return regulator_statuses_.at(regulator_id_to_index(id));
    } catch (const std::out_of_range& e) {
        return ros2_interfaces::msg::VoltageRegulatorStatus();
    }
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

ros2_interfaces::msg::VoltageRegulatorSettings StateManager::get_regulator_settings(uint8_t id) const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
        return regulator_settings_.at(regulator_id_to_index(id));
    } catch (const std::out_of_range& e) {
        return ros2_interfaces::msg::VoltageRegulatorSettings();
    }
}

ros2_interfaces::msg::SystemSettings StateManager::get_system_settings() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return system_settings_;
}
