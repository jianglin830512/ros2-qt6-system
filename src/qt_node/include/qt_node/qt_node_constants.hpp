#ifndef QT_NODE_CONSTANTS_HPP
#define QT_NODE_CONSTANTS_HPP

#include <QObject> // Q_NAMESPACE 需要这个头文件
#include <QtQml/qqml.h>

// =======================================================================
//          合并后的统一常量命名空间
// =======================================================================
namespace qt_node_constants
{
Q_NAMESPACE// 声明此命名空间参与Qt元对象系统

// --- 命令枚举 (供 QML 和 C++ 使用) ---
enum RegulatorOperationCommand {
    CMD_REGULATOR_VOLTAGE_UP = 1,
    CMD_REGULATOR_VOLTAGE_DOWN = 2,
    CMD_REGULATOR_VOLTAGE_STOP = 3
};
Q_ENUM_NS(RegulatorOperationCommand) // 注册枚举

enum RegulatorBreakerCommand {
    CMD_REGULATOR_BREAKER_CLOSE = 1,
    CMD_REGULATOR_BREAKER_OPEN = 2
};
Q_ENUM_NS(RegulatorBreakerCommand) // 注册枚举

enum CircuitModeCommand {
    CMD_CIRCUIT_SET_MANUAL_MODE = 1,
    CMD_CIRCUIT_SET_CONST_CURRENT_MODE = 2,
    CMD_CIRCUIT_SET_TEMP_CONTROL_MODE = 3
};
Q_ENUM_NS(CircuitModeCommand) // 注册枚举

enum CircuitBreakerCommand {
    CMD_CIRCUIT_TEST_BREAKER_CLOSE = 1,
    CMD_CIRCUIT_TEST_BREAKER_OPEN = 2,
    CMD_CIRCUIT_SIM_BREAKER_CLOSE = 3,
    CMD_CIRCUIT_SIM_BREAKER_OPEN = 4
};
Q_ENUM_NS(CircuitBreakerCommand) // 注册枚举

// --- ROS Topic 和参数名 (供 C++ 使用) ---

// 订阅话题
constexpr const char* CIRCUIT_STATUS_TOPIC_PARAM = "circuit_status_topic";
constexpr const char* REGULATOR_STATUS_TOPIC_PARAM = "regulator_status_topic";
constexpr const char* CIRCUIT_SETTINGS_TOPIC_PARAM = "circuit_settings_topic";
constexpr const char* REGULATOR_SETTINGS_TOPIC_PARAM = "regulator_settings_topic";
constexpr const char* SYSTEM_SETTINGS_TOPIC_PARAM = "system_settings_topic";

constexpr const char* DEFAULT_CIRCUIT_STATUS_TOPIC = "default_circuit_status";
constexpr const char* DEFAULT_REGULATOR_STATUS_TOPIC = "default_voltage_regulator_status";
constexpr const char* DEFAULT_CIRCUIT_SETTINGS_TOPIC = "default_circuit_settings";
constexpr const char* DEFAULT_REGULATOR_SETTINGS_TOPIC = "default_regulator_settings";
constexpr const char* DEFAULT_SYSTEM_SETTINGS_TOPIC = "default_system_settings";

// 发布话题
constexpr const char* REGULATOR_OPERATION_COMMAND_TOPIC_PARAM = "regulator_operation_command_topic";
constexpr const char* CLEAR_ALARM_TOPIC_PARAM = "clear_alarm_topic";

constexpr const char* DEFAULT_REGULATOR_OPERATION_COMMAND_TOPIC = "default_regulator_operation_command";
constexpr const char* DEFAULT_CLEAR_ALARM_TOPIC = "default_clear_alarm";

// Service
constexpr const char* REGULATOR_BREAKER_COMMAND_SERVICE_PARAM = "regulator_breaker_command_service";
constexpr const char* CIRCUIT_MODE_COMMAND_SERVICE_PARAM = "circuit_mode_command_service";
constexpr const char* CIRCUIT_BREAKER_COMMAND_SERVICE_PARAM = "circuit_breaker_command_service";
constexpr const char* SET_SYSTEM_SETTINGS_SERVICE_PARAM = "set_system_settings_service";
constexpr const char* SET_REGULATOR_SETTINGS_SERVICE_PARAM = "set_regulator_settings_service";
constexpr const char* SET_CIRCUIT_SETTINGS_SERVICE_PARAM = "set_circuit_settings_service";

constexpr const char* DEFAULT_REGULATOR_BREAKER_COMMAND_SERVICE = "default_regulator_breaker_command";
constexpr const char* DEFAULT_CIRCUIT_MODE_COMMAND_SERVICE = "default_circuit_mode_command";
constexpr const char* CIRCUIT_REGULATOR_BREAKER_COMMAND_SERVICE = "default_circuit_breaker_command";
constexpr const char* DEFAULT_SET_SYSTEM_SETTINGS_SERVICE = "default_set_system_settings";
constexpr const char* DEFAULT_SET_REGULATOR_SETTINGS_SERVICE = "default_set_regulator_settings";
constexpr const char* DEFAULT_SET_CIRCUIT_SETTINGS_SERVICE = "default_set_circuit_settings";

} // namespace qt_node_constants

#endif // QT_NODE_CONSTANTS_HPP


