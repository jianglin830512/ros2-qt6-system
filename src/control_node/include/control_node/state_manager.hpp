#ifndef STATE_MANAGER_HPP
#define STATE_MANAGER_HPP

#include "ros2_interfaces/msg/circuit_status.hpp"
#include "ros2_interfaces/msg/voltage_regulator_status.hpp"
#include "ros2_interfaces/msg/system_settings.hpp"
#include "ros2_interfaces/msg/circuit_settings.hpp"
#include "ros2_interfaces/msg/voltage_regulator_settings.hpp"
#include <mutex>
#include <array>

// 管理系统的所有状态，提供线程安全的访问
class StateManager
{
public:
    // 定义系统中硬件组件的数量，便于维护
    static constexpr size_t NUM_CIRCUITS = 2;
    static constexpr size_t NUM_REGULATORS = 2;

    StateManager();

    // --- 写方法 (带锁) ---
    void update_circuit_status(uint8_t id, const ros2_interfaces::msg::CircuitStatus& status);
    void update_regulator_status(uint8_t id, const ros2_interfaces::msg::VoltageRegulatorStatus& status);

    void update_system_settings(const ros2_interfaces::msg::SystemSettings& settings);
    void update_circuit_settings(uint8_t id, const ros2_interfaces::msg::CircuitSettings& settings);
    void update_regulator_settings(uint8_t id, const ros2_interfaces::msg::VoltageRegulatorSettings& settings);

    // --- 读方法 (带锁) ---
    ros2_interfaces::msg::CircuitStatus get_circuit_status(uint8_t id) const;
    ros2_interfaces::msg::VoltageRegulatorStatus get_regulator_status(uint8_t id) const;
    ros2_interfaces::msg::CircuitSettings get_circuit_settings(uint8_t id) const;
    ros2_interfaces::msg::VoltageRegulatorSettings get_regulator_settings(uint8_t id) const;
    ros2_interfaces::msg::SystemSettings get_system_settings() const;

private:
    // 内部帮助函数，用于将外部ID (1-based) 转换为内部索引 (0-based)
    // 这样做可以保持代码的清晰和安全
    size_t circuit_id_to_index(uint8_t id) const;
    size_t regulator_id_to_index(uint8_t id) const;

    mutable std::mutex state_mutex_;

    // 使用 std::array 来存储固定数量的组件状态/设置
    std::array<ros2_interfaces::msg::CircuitStatus, NUM_CIRCUITS> circuit_statuses_;
    std::array<ros2_interfaces::msg::VoltageRegulatorStatus, NUM_REGULATORS> regulator_statuses_;
    std::array<ros2_interfaces::msg::CircuitSettings, NUM_CIRCUITS> circuit_settings_;
    std::array<ros2_interfaces::msg::VoltageRegulatorSettings, NUM_REGULATORS> regulator_settings_;

    ros2_interfaces::msg::SystemSettings system_settings_;
};

#endif // STATE_MANAGER_HPP
