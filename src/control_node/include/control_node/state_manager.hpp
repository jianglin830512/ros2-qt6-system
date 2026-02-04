#ifndef STATE_MANAGER_HPP
#define STATE_MANAGER_HPP

// --- 包含所有需要的数据类型 ---
#include "ros2_interfaces/msg/system_status.hpp"
#include "ros2_interfaces/msg/circuit_status.hpp"
#include "ros2_interfaces/msg/regulator_status.hpp"
#include "ros2_interfaces/msg/system_settings.hpp"
#include "ros2_interfaces/msg/circuit_settings.hpp"
#include "ros2_interfaces/msg/regulator_settings.hpp"
#include "ros2_interfaces/msg/hardware_circuit_status.hpp"
#include "ros2_interfaces/msg/hardware_circuit_settings.hpp"
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

    // === 写方法 (带锁) ===
    // --- 常规数据 ---
    void update_system_status(const ros2_interfaces::msg::SystemStatus& status);
    void update_circuit_status(uint8_t id, const ros2_interfaces::msg::CircuitStatus& status);
    void update_regulator_status(uint8_t id, const ros2_interfaces::msg::RegulatorStatus& status);
    void update_system_settings(const ros2_interfaces::msg::SystemSettings& settings);
    void update_circuit_settings(uint8_t id, const ros2_interfaces::msg::CircuitSettings& settings);
    void update_regulator_settings(uint8_t id, const ros2_interfaces::msg::RegulatorSettings& settings);

    // --- 硬件原始数据 ---
    void update_hardware_circuit_status(uint8_t id, const ros2_interfaces::msg::HardwareCircuitStatus& status);
    void update_hardware_regulator_status(uint8_t id, const ros2_interfaces::msg::RegulatorStatus& status);
    void update_hardware_circuit_settings(uint8_t id, const ros2_interfaces::msg::HardwareCircuitSettings& settings);
    void update_hardware_regulator_settings(uint8_t id, const ros2_interfaces::msg::RegulatorSettings& settings);


    // === 读方法 (带锁) ===
    // --- 常规数据 ---
    ros2_interfaces::msg::SystemStatus get_system_status() const;
    ros2_interfaces::msg::CircuitStatus get_circuit_status(uint8_t id) const;
    ros2_interfaces::msg::RegulatorStatus get_regulator_status(uint8_t id) const;
    ros2_interfaces::msg::SystemSettings get_system_settings() const;
    ros2_interfaces::msg::CircuitSettings get_circuit_settings(uint8_t id) const;
    ros2_interfaces::msg::RegulatorSettings get_regulator_settings(uint8_t id) const;

    // --- 硬件原始数据 ---
    ros2_interfaces::msg::HardwareCircuitStatus get_hardware_circuit_status(uint8_t id) const;
    ros2_interfaces::msg::RegulatorStatus get_hardware_regulator_status(uint8_t id) const;
    ros2_interfaces::msg::HardwareCircuitSettings get_hardware_circuit_settings(uint8_t id) const;
    ros2_interfaces::msg::RegulatorSettings get_hardware_regulator_settings(uint8_t id) const;


private:
    size_t circuit_id_to_index(uint8_t id) const;
    size_t regulator_id_to_index(uint8_t id) const;

    mutable std::mutex state_mutex_;

    // --- 常规数据存储 ---
    ros2_interfaces::msg::SystemStatus system_status_;
    std::array<ros2_interfaces::msg::CircuitStatus, NUM_CIRCUITS> circuit_statuses_;
    std::array<ros2_interfaces::msg::RegulatorStatus, NUM_REGULATORS> regulator_statuses_;
    ros2_interfaces::msg::SystemSettings system_settings_;
    std::array<ros2_interfaces::msg::CircuitSettings, NUM_CIRCUITS> circuit_settings_;
    std::array<ros2_interfaces::msg::RegulatorSettings, NUM_REGULATORS> regulator_settings_;

    // --- 硬件原始数据存储 ---
    std::array<ros2_interfaces::msg::HardwareCircuitStatus, NUM_CIRCUITS> hardware_circuit_statuses_;
    std::array<ros2_interfaces::msg::RegulatorStatus, NUM_REGULATORS> hardware_regulator_statuses_;
    std::array<ros2_interfaces::msg::HardwareCircuitSettings, NUM_CIRCUITS> hardware_circuit_settings_;
    std::array<ros2_interfaces::msg::RegulatorSettings, NUM_REGULATORS> hardware_regulator_settings_;
};

#endif // STATE_MANAGER_HPP
