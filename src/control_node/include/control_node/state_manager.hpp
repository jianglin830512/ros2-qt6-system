#ifndef STATE_MANAGER_HPP
#define STATE_MANAGER_HPP

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
#include <utility> // for std::pair

class StateManager
{
public:
    static constexpr size_t NUM_CIRCUITS = 2;
    static constexpr size_t NUM_REGULATORS = 2;

    StateManager();

    // === Write Methods ===
    void update_system_status(const ros2_interfaces::msg::SystemStatus& status);
    void update_circuit_status(uint8_t id, const ros2_interfaces::msg::CircuitStatus& status);
    void update_regulator_status(uint8_t id, const ros2_interfaces::msg::RegulatorStatus& status);
    void update_system_settings(const ros2_interfaces::msg::SystemSettings& settings);
    void update_circuit_settings(uint8_t id, const ros2_interfaces::msg::CircuitSettings& settings);
    void update_regulator_settings(uint8_t id, const ros2_interfaces::msg::RegulatorSettings& settings);

    void update_circuit_status_from_hardware(uint8_t id, const ros2_interfaces::msg::HardwareCircuitStatus& hw_status);
    void update_circuit_settings_from_hardware(uint8_t id, const ros2_interfaces::msg::HardwareCircuitSettings& hw_settings);

    // === Read Methods ===
    ros2_interfaces::msg::SystemStatus get_system_status() const;
    ros2_interfaces::msg::CircuitStatus get_circuit_status(uint8_t id) const;
    ros2_interfaces::msg::RegulatorStatus get_regulator_status(uint8_t id) const;
    ros2_interfaces::msg::SystemSettings get_system_settings() const;
    ros2_interfaces::msg::CircuitSettings get_circuit_settings(uint8_t id) const;
    ros2_interfaces::msg::RegulatorSettings get_regulator_settings(uint8_t id) const;

    // [NEW] Get raw PLC status (Mode, Source) that is not in the standard CircuitStatus
    // Returns {mode, source}
    std::pair<uint8_t, uint8_t> get_last_known_plc_status(uint8_t id) const;

private:
    size_t circuit_id_to_index(uint8_t id) const;
    size_t regulator_id_to_index(uint8_t id) const;

    mutable std::mutex state_mutex_;

    ros2_interfaces::msg::SystemStatus system_status_;
    std::array<ros2_interfaces::msg::CircuitStatus, NUM_CIRCUITS> circuit_statuses_;
    std::array<ros2_interfaces::msg::RegulatorStatus, NUM_REGULATORS> regulator_statuses_;
    ros2_interfaces::msg::SystemSettings system_settings_;
    std::array<ros2_interfaces::msg::CircuitSettings, NUM_CIRCUITS> circuit_settings_;
    std::array<ros2_interfaces::msg::RegulatorSettings, NUM_REGULATORS> regulator_settings_;

    // [NEW] Cache for raw PLC status
    struct PlcStatusCache {
        uint8_t plc_control_mode = 0;
        uint8_t plc_control_source = 0;
    };
    std::array<PlcStatusCache, NUM_CIRCUITS> plc_status_cache_;
};
#endif // STATE_MANAGER_HPP
