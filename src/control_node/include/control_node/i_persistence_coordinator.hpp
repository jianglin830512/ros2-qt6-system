#ifndef I_PERSISTENCE_COORDINATOR_HPP
#define I_PERSISTENCE_COORDINATOR_HPP

#include <functional>
#include <string>
#include <cstdint>

// 仅引用消息定义，不引用服务定义
#include "ros2_interfaces/msg/system_settings.hpp"
#include "ros2_interfaces/msg/regulator_settings.hpp"
#include "ros2_interfaces/msg/circuit_settings.hpp"

// 定义数据驱动的回调类型（只传数据消息本身）
using PersistenceCallback = std::function<void(bool success, const std::string& message)>;
using GetSystemSettingsCallback = std::function<void(bool success, const ros2_interfaces::msg::SystemSettings& settings)>;
using GetRegulatorSettingsCallback = std::function<void(bool success, const ros2_interfaces::msg::RegulatorSettings& settings)>;
using GetCircuitSettingsCallback = std::function<void(bool success, const ros2_interfaces::msg::CircuitSettings& settings)>;

class IPersistenceCoordinator {
public:
    virtual ~IPersistenceCoordinator() = default;

    virtual bool is_connected() const = 0;

    // --- 保存操作 ---
    virtual void save_system_settings(PersistenceCallback callback) = 0;
    virtual void save_regulator_settings(uint8_t regulator_id, PersistenceCallback callback) = 0;
    virtual void save_circuit_settings(uint8_t circuit_id, PersistenceCallback callback) = 0;

    // --- 获取操作 (只返回数据，不返回服务包装) ---
    virtual void get_system_settings(GetSystemSettingsCallback callback) = 0;
    virtual void get_regulator_settings(uint8_t regulator_id, GetRegulatorSettingsCallback callback) = 0;
    virtual void get_circuit_settings(uint8_t circuit_id, GetCircuitSettingsCallback callback) = 0;
};

#endif
