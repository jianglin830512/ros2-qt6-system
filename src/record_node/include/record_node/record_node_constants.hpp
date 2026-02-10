#ifndef RECORD_NODE_CONSTANTS_HPP_
#define RECORD_NODE_CONSTANTS_HPP_

namespace record_node_constants
{
// 参数名称
[[maybe_unused]] constexpr const char* DB_PATH_PARAM = "db_path";
[[maybe_unused]] constexpr const char* DEFAULT_DB_PATH = "records_and_settings.db";

// 记录间隔 (分钟)
[[maybe_unused]] constexpr const char* RECORD_INTERVAL_MIN_PARAM = "record_interval_min";
[[maybe_unused]] constexpr int DEFAULT_RECORD_INTERVAL_MIN = 1;

// 服务名称 - 保存/设置类
// [[maybe_unused]] constexpr const char* SAVE_SYSTEM_SETTINGS_SERVICE_PARAM = "save_system_settings_service";
// [[maybe_unused]] constexpr const char* SAVE_REGULATOR_SETTINGS_SERVICE_PARAM = "save_regulator_settings_service";
// [[maybe_unused]] constexpr const char* SAVE_CIRCUIT_SETTINGS_SERVICE_PARAM = "save_circuit_settings_service";
// [[maybe_unused]] constexpr const char* DEFAULT_SAVE_SYSTEM_SETTINGS_SERVICE = "default_save_system_settings";
// [[maybe_unused]] constexpr const char* DEFAULT_SAVE_REGULATOR_SETTINGS_SERVICE = "default_save_regulator_settings";
// [[maybe_unused]] constexpr const char* DEFAULT_SAVE_CIRCUIT_SETTINGS_SERVICE = "default_save_circuit_settings";

// 服务名称 - 查询/获取类
[[maybe_unused]] constexpr const char* GET_SYSTEM_SETTINGS_SERVICE_PARAM = "get_system_settings_service";
[[maybe_unused]] constexpr const char* GET_REGULATOR_SETTINGS_SERVICE_PARAM = "get_regulator_settings_service";
[[maybe_unused]] constexpr const char* GET_CIRCUIT_SETTINGS_SERVICE_PARAM = "get_circuit_settings_service";
[[maybe_unused]] constexpr const char* GET_DATA_RECORDS_SERVICE_PARAM = "get_data_records_service";
[[maybe_unused]] constexpr const char* DEFAULT_GET_SYSTEM_SETTINGS_SERVICE = "default_get_system_settings";
[[maybe_unused]] constexpr const char* DEFAULT_GET_REGULATOR_SETTINGS_SERVICE = "default_get_regulator_settings";
[[maybe_unused]] constexpr const char* DEFAULT_GET_CIRCUIT_SETTINGS_SERVICE = "default_get_circuit_settings";
[[maybe_unused]] constexpr const char* DEFAULT_GET_DATA_RECORDS_SERVICE = "default_get_data_records";

// Topic 名称参数 (必须与 Control Node 保持一致)
// status
[[maybe_unused]] constexpr const char* CIRCUIT_STATUS_TOPIC_PARAM = "circuit_status_topic";
[[maybe_unused]] constexpr const char* REGULATOR_STATUS_TOPIC_PARAM = "regulator_status_topic";
[[maybe_unused]] constexpr const char* DEFAULT_CIRCUIT_STATUS_TOPIC = "default_circuit_status";
[[maybe_unused]] constexpr const char* DEFAULT_REGULATOR_STATUS_TOPIC = "default_voltage_regulator_status";
// settings
[[maybe_unused]] constexpr const char* CIRCUIT_SETTINGS_TOPIC_PARAM = "circuit_settings_topic";
[[maybe_unused]] constexpr const char* REGULATOR_SETTINGS_TOPIC_PARAM = "regulator_settings_topic";
[[maybe_unused]] constexpr const char* SYSTEM_SETTINGS_TOPIC_PARAM = "system_settings_topic";
[[maybe_unused]] constexpr const char* DEFAULT_CIRCUIT_SETTINGS_TOPIC = "default_circuit_settings";
[[maybe_unused]] constexpr const char* DEFAULT_REGULATOR_SETTINGS_TOPIC = "default_regulator_settings";
[[maybe_unused]] constexpr const char* DEFAULT_SYSTEM_SETTINGS_TOPIC = "default_system_settings";

} // namespace record_node_constants

#endif // RECORD_NODE_CONSTANTS_HPP_
