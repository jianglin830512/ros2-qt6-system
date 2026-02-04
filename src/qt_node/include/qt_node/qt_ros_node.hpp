#ifndef QT_ROS_NODE_H
#define QT_ROS_NODE_H

#include <QObject>
#include <QTimer>
#include "rclcpp/rclcpp.hpp"  // IWYU pragma: keep
#include "std_msgs/msg/empty.hpp"

// 引入自定义数据类型
#include "qt_node/data_types/data_types.hpp"
#include "qt_node/data_types/circuit_settings_data.hpp"
#include "qt_node/data_types/regulator_settings_data.hpp"
#include "qt_node/data_types/system_settings_data.hpp"

// --- 引入更新后的ROS消息和服务头文件 ---
// -- SUB --
// 状态
#include "ros2_interfaces/msg/circuit_status.hpp"
#include "ros2_interfaces/msg/regulator_status.hpp"
// 设置
#include "ros2_interfaces/msg/system_settings.hpp"
#include "ros2_interfaces/msg/regulator_settings.hpp"
#include "ros2_interfaces/msg/circuit_settings.hpp"
// -- PUB --
// 命令
#include "ros2_interfaces/msg/regulator_operation_command.hpp"
// -- Service Client --
// 命令
#include "ros2_interfaces/srv/regulator_breaker_command.hpp"
#include "ros2_interfaces/srv/circuit_mode_command.hpp"
#include "ros2_interfaces/srv/circuit_breaker_command.hpp"
// 设置
#include "ros2_interfaces/srv/set_system_settings.hpp"
#include "ros2_interfaces/srv/set_regulator_settings.hpp"
#include "ros2_interfaces/srv/set_circuit_settings.hpp"

using SystemSettingsMsgPtr = ros2_interfaces::msg::SystemSettings::SharedPtr;
using RegulatorSettingsMsgPtr = ros2_interfaces::msg::RegulatorSettings::SharedPtr;
using CircuitSettingsMsgPtr = ros2_interfaces::msg::CircuitSettings::SharedPtr;

class QtROSNode : public QObject, public rclcpp::Node
{
    Q_OBJECT
public:
    explicit QtROSNode(const std::string &node_name, QObject *parent = nullptr);
    ~QtROSNode();

public slots:
    void onShutdownRequested();
    void startTimer();

    // --- 接收来自Proxy信号的槽 (已重构) ---
    void onSendRegulatorOperationCommand(quint8 regulator_id, quint8 command);
    void onSendRegulatorBreakerCommand(quint8 regulator_id, quint8 command);
    void onSendCircuitModeCommand(quint8 circuit_id, quint8 command);
    void onSendCircuitBreakerCommand(quint8 circuit_id, quint8 command);
    void onSendClearAlarm();

    // --- 设置参数服务的槽 ---
    void onSetSystemSettings(SystemSettingsData* data);
    void onSetRegulatorSettings(quint8 regulator_id, RegulatorSettingsData* data);
    void onSetCircuitSettings(quint8 circuit_id, CircuitSettingsData* data);

private slots:
    void spin_some();

signals:
    void shutdownFinished();

    // --- 传递状态数据的信号 ---
    void circuitStatusReceived(const CircuitStatusData &data);
    void regulatorStatusReceived(const RegulatorStatusData &data);

    // --- 将服务调用结果返回给Proxy的信号 ---
    void settingsUpdateResult(const QString &service_name, bool success, const QString &message);
    void commandResult(const QString &service_name, bool success, const QString &message);

    // --- 用于传递新创建的Settings对象的指针 ---
    void systemSettingsReceived(SystemSettingsMsgPtr msg);
    void regulatorSettingsReceived(RegulatorSettingsMsgPtr msg);
    void circuitSettingsReceived(CircuitSettingsMsgPtr msg);
private:
    // --- 回调函数 ---
    void circuit_status_callback(const ros2_interfaces::msg::CircuitStatus::SharedPtr msg);
    void regulator_status_callback(const ros2_interfaces::msg::RegulatorStatus::SharedPtr msg);
    void system_settings_callback(SystemSettingsMsgPtr msg);
    void regulator_settings_callback(RegulatorSettingsMsgPtr msg);
    void circuit_settings_callback(CircuitSettingsMsgPtr msg);

    // --- 订阅者 ---
    rclcpp::Subscription<ros2_interfaces::msg::CircuitStatus>::SharedPtr circuit_status_sub_;
    std::string circuit_status_topic_;
    rclcpp::Subscription<ros2_interfaces::msg::RegulatorStatus>::SharedPtr regulator_status_sub_; // 类型名变更
    std::string regulator_status_topic_;
    rclcpp::Subscription<ros2_interfaces::msg::SystemSettings>::SharedPtr system_settings_sub_;
    std::string system_settings_topic_;
    rclcpp::Subscription<ros2_interfaces::msg::RegulatorSettings>::SharedPtr regulator_settings_sub_;
    std::string regulator_settings_topic_;
    rclcpp::Subscription<ros2_interfaces::msg::CircuitSettings>::SharedPtr circuit_settings_sub_;
    std::string circuit_settings_topic_;

    // --- 发布者 (已重构) ---
    rclcpp::Publisher<ros2_interfaces::msg::RegulatorOperationCommand>::SharedPtr regulator_operation_command_pub_;
    std::string regulator_operation_command_topic_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr clear_alarm_pub_;
    std::string clear_alarm_topic_;

    // --- 服务客户端 (已重构和新增) ---
    // 命令服务
    rclcpp::Client<ros2_interfaces::srv::RegulatorBreakerCommand>::SharedPtr regulator_breaker_command_client_;
    std::string regulator_breaker_command_service_name_;
    rclcpp::Client<ros2_interfaces::srv::CircuitModeCommand>::SharedPtr circuit_mode_command_client_;
    std::string circuit_mode_command_service_name_;
    rclcpp::Client<ros2_interfaces::srv::CircuitBreakerCommand>::SharedPtr circuit_breaker_command_client_;
    std::string circuit_breaker_command_service_name_;
    // 设置服务
    rclcpp::Client<ros2_interfaces::srv::SetSystemSettings>::SharedPtr set_system_settings_client_;
    std::string set_system_settings_service_name_;
    rclcpp::Client<ros2_interfaces::srv::SetRegulatorSettings>::SharedPtr set_regulator_settings_client_;
    std::string set_regulator_settings_service_name_;
    rclcpp::Client<ros2_interfaces::srv::SetCircuitSettings>::SharedPtr set_circuit_settings_client_;
    std::string set_circuit_settings_service_name_;

    QTimer* m_ros_timer;
};

#endif // QT_ROS_NODE_H
