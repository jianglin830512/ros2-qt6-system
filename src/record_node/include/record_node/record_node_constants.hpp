#ifndef RECORD_NODE_CONSTANTS_HPP_
#define RECORD_NODE_CONSTANTS_HPP_

namespace record_node_constants
{
    // 参数名称
    [[maybe_unused]] constexpr const char* DB_PATH_PARAM = "db_path";
    [[maybe_unused]] constexpr const char* DEFAULT_DB_PATH = "records_and_settings.db";

    // 服务名称 (必须与 control_node 的客户端配置匹配)
    [[maybe_unused]] constexpr const char* SAVE_SYSTEM_SETTINGS_SERVICE_PARAM = "save_system_settings_service";
    [[maybe_unused]] constexpr const char* SAVE_REGULATOR_SETTINGS_SERVICE_PARAM = "save_regulator_settings_service";
    [[maybe_unused]] constexpr const char* SAVE_CIRCUIT_SETTINGS_SERVICE_PARAM = "save_circuit_settings_service";
    [[maybe_unused]] constexpr const char* DEFAULT_SAVE_SYSTEM_SETTINGS_SERVICE = "default_save_system_settings";
    [[maybe_unused]] constexpr const char* DEFAULT_SAVE_REGULATOR_SETTINGS_SERVICE = "default_save_regulator_settings";
    [[maybe_unused]] constexpr const char* DEFAULT_SAVE_CIRCUIT_SETTINGS_SERVICE = "default_save_circuit_settings";
} // namespace record_node_constants

#endif // RECORD_NODE_CONSTANTS_HPP_
