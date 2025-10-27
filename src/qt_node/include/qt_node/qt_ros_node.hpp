#ifndef QT_ROS_NODE_H
#define QT_ROS_NODE_H

#include <QObject>
#include <QTimer>
#include "rclcpp/rclcpp.hpp"  // IWYU pragma: keep
#include "std_msgs/msg/empty.hpp"

// 引入新的数据类型
#include "qt_node/data_types/data_types.hpp"
#include "qt_node/data_types/circuit_settings_data.hpp"
#include "qt_node/data_types/voltage_regulator_settings_data.hpp"
#include "qt_node/data_types/system_settings_data.hpp"
// 引入ROS消息头文件
#include "ros2_interfaces/msg/circuit_status.hpp"
#include "ros2_interfaces/msg/voltage_regulator_status.hpp"
#include "ros2_interfaces/msg/circuit_command.hpp"
#include "ros2_interfaces/msg/voltage_regulator_command.hpp"
#include "ros2_interfaces/srv/set_system_settings.hpp"
#include "ros2_interfaces/srv/set_regulator_settings.hpp"
#include "ros2_interfaces/srv/set_circuit_settings.hpp"

class QtROSNode : public QObject, public rclcpp::Node
{
    Q_OBJECT
public:
    explicit QtROSNode(const std::string &node_name, QObject *parent = nullptr);
    ~QtROSNode();

public slots:
    void startTimer();
    // --- 接收来自Proxy信号的槽 ---
    void onSendRegulatorCommand(quint8 regulator_id, quint8 command);
    void onSendCircuitCommand(quint8 circuit_id, quint8 command);
    void onSendClearAlarm();

    // --- Set param service: Slots to handle setting requests from the proxy ---
    void onSetSystemSettings(SystemSettingsData* data);
    void onSetRegulatorSettings(quint8 regulator_id, VoltageRegulatorSettingsData* data);
    void onSetCircuitSettings(quint8 circuit_id, CircuitSettingsData* data);

private slots:
    void spin_some();

signals:
    // 用于传递转换后的POD数据
    void circuitStatusReceived(const CircuitStatusData &data);
    void voltageRegulatorStatusReceived(const VoltageRegulatorStatusData &data);

    // --- Set param service: Signal to send results back to the proxy ---
    void settingsUpdateResult(const QString &service_name, bool success, const QString &message);

private:
    // 回调函数
    void circuit_status_callback(const ros2_interfaces::msg::CircuitStatus::SharedPtr msg);
    void voltage_regulator_status_callback(const ros2_interfaces::msg::VoltageRegulatorStatus::SharedPtr msg);

    // 订阅者
    rclcpp::Subscription<ros2_interfaces::msg::CircuitStatus>::SharedPtr circuit_status_sub_;
    std::string circuit_status_topic_;
    rclcpp::Subscription<ros2_interfaces::msg::VoltageRegulatorStatus>::SharedPtr regulator_status_sub_;
    std::string regulator_status_topic_;

    // --- 发布者和对应的话题名 ---
    rclcpp::Publisher<ros2_interfaces::msg::VoltageRegulatorCommand>::SharedPtr regulator_command_pub_;
    std::string regulator_command_topic_;
    rclcpp::Publisher<ros2_interfaces::msg::CircuitCommand>::SharedPtr circuit_command_pub_;
    std::string circuit_command_topic_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr clear_alarm_pub_;
    std::string clear_alarm_topic_;

    // --- Service Clients ---
    rclcpp::Client<ros2_interfaces::srv::SetSystemSettings>::SharedPtr set_system_settings_client_;
    std::string set_system_settings_service_name_;
    rclcpp::Client<ros2_interfaces::srv::SetRegulatorSettings>::SharedPtr set_regulator_settings_client_;
    std::string set_regulator_settings_service_name_;
    rclcpp::Client<ros2_interfaces::srv::SetCircuitSettings>::SharedPtr set_circuit_settings_client_;
    std::string set_circuit_settings_service_name_;

    QTimer* m_ros_timer;
};

#endif // QT_ROS_NODE_H
