#ifndef CONTROL_NODE_CONSTANTS_HPP
#define CONTROL_NODE_CONSTANTS_HPP

namespace control_node_constants
{
// ament_cmake 声明的hpp文件，有可能使用了的常量还是被认为“未使用”并给出warning。所以可以用[[maybe_unused]]解决。
// 使用 constexpr const char* 可以获得编译时的常量，效率更高

// --- 发布器相关常量 ---
[[maybe_unused]] constexpr const char* CIRCUIT_STATUS_TOPIC_PARAM = "circuit_status_topic";
[[maybe_unused]] constexpr const char* REGULATOR_STATUS_TOPIC_PARAM = "regulator_status_topic";
[[maybe_unused]] constexpr const char* DEFAULT_CIRCUIT_STATUS_TOPIC = "default_circuit_status";
[[maybe_unused]] constexpr const char* DEFAULT_REGULATOR_STATUS_TOPIC = "default_voltage_regulator_status";

// --- 订阅器相关常量 ---
[[maybe_unused]] constexpr const char* REGULATOR_COMMAND_TOPIC_PARAM = "regulator_command_topic";
[[maybe_unused]] constexpr const char* CIRCUIT_COMMAND_TOPIC_PARAM = "circuit_command_topic";
[[maybe_unused]] constexpr const char* CLEAR_ALARM_TOPIC_PARAM = "clear_alarm_topic";
[[maybe_unused]] constexpr const char* DEFAULT_REGULATOR_COMMAND_TOPIC = "default_regulator_command";
[[maybe_unused]] constexpr const char* DEFAULT_CIRCUIT_COMMAND_TOPIC = "default_circuit_command";
[[maybe_unused]] constexpr const char* DEFAULT_CLEAR_ALARM_TOPIC = "default_clear_alarm_command";

// --- 服务服务器相关常量 ---
[[maybe_unused]] constexpr const char* SET_SYSTEM_SETTINGS_SERVICE_PARAM = "set_system_settings_service";
[[maybe_unused]] constexpr const char* SET_REGULATOR_SETTINGS_SERVICE_PARAM = "set_regulator_settings_service";
[[maybe_unused]] constexpr const char* SET_CIRCUIT_SETTINGS_SERVICE_PARAM = "set_circuit_settings_service";
[[maybe_unused]] constexpr const char* DEFAULT_SET_SYSTEM_SETTINGS_SERVICE = "default_set_system_settings";
[[maybe_unused]] constexpr const char* DEFAULT_SET_REGULATOR_SETTINGS_SERVICE = "default_set_regulator_settings";
[[maybe_unused]] constexpr const char* DEFAULT_SET_CIRCUIT_SETTINGS_SERVICE = "default_set_circuit_settings";

} // namespace control_node_constants

#endif // CONTROL_NODE_CONSTANTS_HPP
