#ifndef HARDWARE_NODE_CONSTANTS_HPP
#define HARDWARE_NODE_CONSTANTS_HPP

namespace hardware_node_constants
{
// --- 发布器相关常量 ---
[[maybe_unused]] constexpr const char* HARDWARE_CIRCUIT_STATUS_TOPIC_PARAM = "hardware_circuit_status_topic";
[[maybe_unused]] constexpr const char* HARDWARE_REGULATOR_STATUS_TOPIC_PARAM = "hardware_regulator_status_topic";
[[maybe_unused]] constexpr const char* HARDWARE_SYSTEM_STATUS_TOPIC_PARAM = "hardware_system_status_topic";
[[maybe_unused]] constexpr const char* DEFAULT_HARDWARE_CIRCUIT_STATUS_TOPIC = "default_hardware_circuit_status";
[[maybe_unused]] constexpr const char* DEFAULT_HARDWARE_REGULATOR_STATUS_TOPIC = "default_hardware_regulator_status";
[[maybe_unused]] constexpr const char* DEFAULT_HARDWARE_SYSTEM_STATUS_TOPIC = "default_hardware_system_status";

// --- settings ---
[[maybe_unused]] constexpr const char* HARDWARE_CIRCUIT_SETTINGS_TOPIC_PARAM = "hardware_circuit_settings_topic";
[[maybe_unused]] constexpr const char* HARDWARE_REGULATOR_SETTINGS_TOPIC_PARAM = "hardware_regulator_settings_topic";
[[maybe_unused]] constexpr const char* DEFAULT_HARDWARE_CIRCUIT_SETTINGS_TOPIC = "default_hardware_circuit_settings";
[[maybe_unused]] constexpr const char* DEFAULT_HARDWARE_REGULATOR_SETTINGS_TOPIC = "default_hardware_regulator_settings";

// --- 订阅器相关常量 ---
[[maybe_unused]] constexpr const char* HARDWARE_REGULATOR_OPERATION_COMMAND_TOPIC_PARAM = "hardware_regulator_operation_command_topic";
[[maybe_unused]] constexpr const char* DEFAULT_HARDWARE_REGULATOR_OPERATION_COMMAND_TOPIC = "default_hardware_regulator_operation_command";
[[maybe_unused]] constexpr const char* HARDWARE_CLEAR_ALARM_TOPIC_PARAM = "hardware_clear_alarm_topic";
[[maybe_unused]] constexpr const char* DEFAULT_HARDWARE_CLEAR_ALARM_TOPIC = "default_hardware_clear_alarm";

// --- 服务服务器相关常量 (Server) ---
[[maybe_unused]] constexpr const char* SET_HARDWARE_REGULATOR_SETTINGS_SERVICE_PARAM = "set_hardware_regulator_settings_service";
[[maybe_unused]] constexpr const char* SET_HARDWARE_CIRCUIT_SETTINGS_SERVICE_PARAM = "set_hardware_circuit_settings_service";
[[maybe_unused]] constexpr const char* DEFAULT_SET_HARDWARE_REGULATOR_SETTINGS_SERVICE = "default_set_hardware_regulator_settings";
[[maybe_unused]] constexpr const char* DEFAULT_SET_HARDWARE_CIRCUIT_SETTINGS_SERVICE = "default_set_hardware_circuit_settings";

[[maybe_unused]] constexpr const char* HARDWARE_REGULATOR_BREAKER_COMMAND_SERVICE_PARAM = "hardware_regulator_breaker_command_service";
[[maybe_unused]] constexpr const char* HARDWARE_CIRCUIT_BREAKER_COMMAND_SERVICE_PARAM = "hardware_circuit_breaker_command_service";
[[maybe_unused]] constexpr const char* DEFAULT_HARDWARE_REGULATOR_BREAKER_COMMAND_SERVICE = "default_hardware_regulator_breaker_command";
[[maybe_unused]] constexpr const char* DEFAULT_HARDWARE_CIRCUIT_BREAKER_COMMAND_SERVICE = "default_hardware_circuit_breaker_command";

[[maybe_unused]] constexpr const char* HARDWARE_SET_CONTROL_MODE_SERVICE_PARAM = "hardware_set_control_mode_service";
[[maybe_unused]] constexpr const char* DEFAULT_HARDWARE_SET_CONTROL_MODE_SERVICE = "default_hardware_set_control_mode";

// --- 其它参数 ---
[[maybe_unused]] constexpr const char* POLLING_RATE_MS_PARAM = "polling_rate_ms";
[[maybe_unused]] constexpr const char* USE_MOCK_DRIVER = "use_mock_driver";

// --- 网络/超时相关参数 (【新增】) ---
[[maybe_unused]] constexpr const char* TCP_CONNECT_TIMEOUT_MS_PARAM = "tcp_connect_timeout_ms";
[[maybe_unused]] constexpr const char* TCP_RECV_TIMEOUT_MS_PARAM = "tcp_recv_timeout_ms";
[[maybe_unused]] constexpr const char* REGULATOR_CMD_TIMEOUT_MS_PARAM = "regulator_cmd_timeout_ms";
[[maybe_unused]] constexpr const char* SERVICE_CALL_TIMEOUT_MS_PARAM = "service_call_timeout_ms";

// --- TCP 连接参数 ---
[[maybe_unused]] constexpr const char* PLC_IP_ADDRESS_PARAM = "plc_ip_address";
[[maybe_unused]] constexpr const char* PLC_PORT_PARAM = "plc_port";

[[maybe_unused]] constexpr const char* TEMP_MONITOR_IP_ADDRESS_PARAM = "temp_monitor_ip_address";
[[maybe_unused]] constexpr const char* TEMP_MONITOR_PORT_PARAM = "temp_monitor_port";

} // namespace hardware_node_constants

#endif // HARDWARE_NODE_CONSTANTS_HPP
