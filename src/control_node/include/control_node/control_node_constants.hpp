#ifndef CONTROL_NODE_CONSTANTS_HPP
#define CONTROL_NODE_CONSTANTS_HPP

namespace control_node_constants
{
// ament_cmake 声明的hpp文件，有可能使用了的常量还是被认为“未使用”并给出warning。所以可以用[[maybe_unused]]解决。
// 使用 constexpr const char* 可以获得编译时的常量，效率更高

// --- 与【QT_NODE】通信 ---

// control_node 作为【发布者】
// 发布状态给 qt_node
[[maybe_unused]] constexpr const char* CIRCUIT_STATUS_TOPIC_PARAM = "circuit_status_topic";
[[maybe_unused]] constexpr const char* REGULATOR_STATUS_TOPIC_PARAM = "regulator_status_topic";
[[maybe_unused]] constexpr const char* DEFAULT_CIRCUIT_STATUS_TOPIC = "default_circuit_status";
[[maybe_unused]] constexpr const char* DEFAULT_REGULATOR_STATUS_TOPIC = "default_voltage_regulator_status";

// 发布当前设置给 qt_node
[[maybe_unused]] constexpr const char* SYSTEM_SETTINGS_TOPIC_PARAM = "system_settings_topic";
[[maybe_unused]] constexpr const char* REGULATOR_SETTINGS_TOPIC_PARAM = "regulator_settings_topic";
[[maybe_unused]] constexpr const char* CIRCUIT_SETTINGS_TOPIC_PARAM = "circuit_settings_topic";
[[maybe_unused]] constexpr const char* DEFAULT_SYSTEM_SETTINGS_TOPIC = "default_system_settings";
[[maybe_unused]] constexpr const char* DEFAULT_REGULATOR_SETTINGS_TOPIC = "default_regulator_settings";
[[maybe_unused]] constexpr const char* DEFAULT_CIRCUIT_SETTINGS_TOPIC = "default_circuit_settings";


// control_node 作为【订阅者】
// [变更] 只订阅调压器的操作命令 (升/降/停)
[[maybe_unused]] constexpr const char* REGULATOR_OPERATION_COMMAND_TOPIC_PARAM = "regulator_operation_command_topic";
[[maybe_unused]] constexpr const char* DEFAULT_REGULATOR_OPERATION_COMMAND_TOPIC = "default_regulator_operation_command";

// 订阅清除报警命令
[[maybe_unused]] constexpr const char* CLEAR_ALARM_TOPIC_PARAM = "clear_alarm_topic";
[[maybe_unused]] constexpr const char* DEFAULT_CLEAR_ALARM_TOPIC = "default_clear_alarm";


// control_node 作为【服务Server】
// 提供命令执行服务
[[maybe_unused]] constexpr const char* REGULATOR_BREAKER_COMMAND_SERVICE_PARAM = "regulator_breaker_command_service";
[[maybe_unused]] constexpr const char* CIRCUIT_MODE_COMMAND_SERVICE_PARAM = "circuit_mode_command_service";
[[maybe_unused]] constexpr const char* CIRCUIT_BREAKER_COMMAND_SERVICE_PARAM = "circuit_breaker_command_service";
[[maybe_unused]] constexpr const char* DEFAULT_REGULATOR_BREAKER_COMMAND_SERVICE = "default_regulator_breaker_command";
[[maybe_unused]] constexpr const char* DEFAULT_CIRCUIT_MODE_COMMAND_SERVICE = "default_circuit_mode_command";
[[maybe_unused]] constexpr const char* DEFAULT_CIRCUIT_BREAKER_COMMAND_SERVICE = "default_circuit_breaker_command";

// 提供参数设置服务
[[maybe_unused]] constexpr const char* SET_SYSTEM_SETTINGS_SERVICE_PARAM = "set_system_settings_service";
[[maybe_unused]] constexpr const char* SET_REGULATOR_SETTINGS_SERVICE_PARAM = "set_regulator_settings_service";
[[maybe_unused]] constexpr const char* SET_CIRCUIT_SETTINGS_SERVICE_PARAM = "set_circuit_settings_service";
[[maybe_unused]] constexpr const char* DEFAULT_SET_SYSTEM_SETTINGS_SERVICE = "default_set_system_settings";
[[maybe_unused]] constexpr const char* DEFAULT_SET_REGULATOR_SETTINGS_SERVICE = "default_set_regulator_settings";
[[maybe_unused]] constexpr const char* DEFAULT_SET_CIRCUIT_SETTINGS_SERVICE = "default_set_circuit_settings";

// --- 与【RECORD_NODE】通信 ---

// Client (保存)
[[maybe_unused]] constexpr const char* SAVE_SYSTEM_SETTINGS_SERVICE_PARAM = "save_system_settings_service";
[[maybe_unused]] constexpr const char* SAVE_REGULATOR_SETTINGS_SERVICE_PARAM = "save_regulator_settings_service";
[[maybe_unused]] constexpr const char* SAVE_CIRCUIT_SETTINGS_SERVICE_PARAM = "save_circuit_settings_service";
[[maybe_unused]] constexpr const char* DEFAULT_SAVE_SYSTEM_SETTINGS_SERVICE = "default_save_system_settings";
[[maybe_unused]] constexpr const char* DEFAULT_SAVE_REGULATOR_SETTINGS_SERVICE = "default_save_regulator_settings";
[[maybe_unused]] constexpr const char* DEFAULT_SAVE_CIRCUIT_SETTINGS_SERVICE = "default_save_circuit_settings";

// Client (查询)
[[maybe_unused]] constexpr const char* GET_SYSTEM_SETTINGS_SERVICE_PARAM = "get_system_settings_service";
[[maybe_unused]] constexpr const char* GET_REGULATOR_SETTINGS_SERVICE_PARAM = "get_regulator_settings_service";
[[maybe_unused]] constexpr const char* GET_CIRCUIT_SETTINGS_SERVICE_PARAM = "get_circuit_settings_service";
[[maybe_unused]] constexpr const char* DEFAULT_GET_SYSTEM_SETTINGS_SERVICE = "default_get_system_settings";
[[maybe_unused]] constexpr const char* DEFAULT_GET_REGULATOR_SETTINGS_SERVICE = "default_get_regulator_settings";
[[maybe_unused]] constexpr const char* DEFAULT_GET_CIRCUIT_SETTINGS_SERVICE = "default_get_circuit_settings";

// --- 与【HARDWARE_NODE】通信 ---

// pub
[[maybe_unused]] constexpr const char* HARDWARE_REGULATOR_OPERATION_COMMAND_TOPIC_PARAM = "hardware_regulator_operation_command_topic";
[[maybe_unused]] constexpr const char* HARDWARE_CLEAR_ALARM_TOPIC_PARAM = "hardware_clear_alarm_topic";
[[maybe_unused]] constexpr const char* DEFAULT_HARDWARE_REGULATOR_OPERATION_COMMAND_TOPIC = "default_hardware_regulator_operation_command";
[[maybe_unused]] constexpr const char* DEFAULT_HARDWARE_CLEAR_ALARM_TOPIC = "default_hardware_clear_alarm";

// sub
[[maybe_unused]] constexpr const char* HARDWARE_CIRCUIT_STATUS_TOPIC_PARAM = "hardware_circuit_status_topic";
[[maybe_unused]] constexpr const char* HARDWARE_REGULATOR_STATUS_TOPIC_PARAM = "hardware_regulator_status_topic";
[[maybe_unused]] constexpr const char* HARDWARE_CIRCUIT_SETTINGS_TOPIC_PARAM = "hardware_circuit_settings_topic";
[[maybe_unused]] constexpr const char* HARDWARE_REGULATOR_SETTINGS_TOPIC_PARAM = "hardware_regulator_settings_topic";
[[maybe_unused]] constexpr const char* DEFAULT_HARDWARE_CIRCUIT_STATUS_TOPIC = "default_hardware_circuit_status";
[[maybe_unused]] constexpr const char* DEFAULT_HARDWARE_REGULATOR_STATUS_TOPIC = "default_hardware_regulator_status";
[[maybe_unused]] constexpr const char* DEFAULT_HARDWARE_CIRCUIT_SETTINGS_TOPIC = "default_hardware_circuit_settings";
[[maybe_unused]] constexpr const char* DEFAULT_HARDWARE_REGULATOR_SETTINGS_TOPIC = "default_hardware_regulator_settings";

// Client (settings)
[[maybe_unused]] constexpr const char* SET_HARDWARE_REGULATOR_SETTINGS_SERVICE_PARAM = "set_hardware_regulator_settings_service";
[[maybe_unused]] constexpr const char* SET_HARDWARE_CIRCUIT_SETTINGS_SERVICE_PARAM = "set_hardware_circuit_settings_service";
[[maybe_unused]] constexpr const char* DEFAULT_SET_HARDWARE_REGULATOR_SETTINGS_SERVICE = "default_set_hardware_regulator_settings";
[[maybe_unused]] constexpr const char* DEFAULT_SET_HARDWARE_CIRCUIT_SETTINGS_SERVICE = "default_set_hardware_circuit_settings";

// Client (command)
[[maybe_unused]] constexpr const char* HARDWARE_REGULATOR_BREAKER_COMMAND_SERVICE_PARAM = "hardware_regulator_breaker_command_service";
[[maybe_unused]] constexpr const char* HARDWARE_CIRCUIT_BREAKER_COMMAND_SERVICE_PARAM = "hardware_circuit_breaker_command_service";
[[maybe_unused]] constexpr const char* DEFAULT_HARDWARE_REGULATOR_BREAKER_COMMAND_SERVICE = "default_hardware_regulator_breaker_command";
[[maybe_unused]] constexpr const char* DEFAULT_HARDWARE_CIRCUIT_BREAKER_COMMAND_SERVICE = "default_hardware_circuit_breaker_command";

[[maybe_unused]] constexpr const char* HARDWARE_SET_CONTROL_MODE_SERVICE_PARAM = "hardware_set_control_mode_service";
[[maybe_unused]] constexpr const char* HARDWARE_SET_CONTROL_SOURCE_SERVICE_PARAM = "hardware_set_control_source_service";
[[maybe_unused]] constexpr const char* DEFAULT_HARDWARE_SET_CONTROL_MODE_SERVICE = "default_hardware_set_control_mode";
[[maybe_unused]] constexpr const char* DEFAULT_HARDWARE_SET_CONTROL_SOURCE_SERVICE = "default_hardware_set_control_source";
} // namespace control_node_constants

#endif // CONTROL_NODE_CONSTANTS_HPP
