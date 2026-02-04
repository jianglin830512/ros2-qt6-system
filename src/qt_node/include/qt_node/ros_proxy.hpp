#ifndef ROS_PROXY_HPP
#define ROS_PROXY_HPP

#include <QObject>
#include "qt_node/data_types/data_types.hpp" // 引入简单的数据类型
#include "qt_node/data_types/circuit_settings_data.hpp"
#include "qt_node/data_types/regulator_settings_data.hpp"
#include "qt_node/data_types/system_settings_data.hpp"
#include "ros2_interfaces/msg/system_settings.hpp"
#include "ros2_interfaces/msg/regulator_settings.hpp"
#include "ros2_interfaces/msg/circuit_settings.hpp"
#include "qt_node/qt_node_constants.hpp" // 引入常量定义

using SystemSettingsMsgPtr = ros2_interfaces::msg::SystemSettings::SharedPtr;
using RegulatorSettingsMsgPtr = ros2_interfaces::msg::RegulatorSettings::SharedPtr;
using CircuitSettingsMsgPtr = ros2_interfaces::msg::CircuitSettings::SharedPtr;

class ROSProxy : public QObject
{
    Q_OBJECT

    // STATUS
    Q_PROPERTY(CircuitStatusData circuitStatus1 READ circuitStatus1 NOTIFY circuitStatus1Changed)
    Q_PROPERTY(CircuitStatusData circuitStatus2 READ circuitStatus2 NOTIFY circuitStatus2Changed)
    Q_PROPERTY(RegulatorStatusData regulatorStatus1 READ regulatorStatus1 NOTIFY regulatorStatus1Changed)
    Q_PROPERTY(RegulatorStatusData regulatorStatus2 READ regulatorStatus2 NOTIFY regulatorStatus2Changed)
    // SETTINGS
    Q_PROPERTY(SystemSettingsData* qmlSystemSettings READ qmlSystemSettings NOTIFY qmlSystemSettingsChanged)
    Q_PROPERTY(RegulatorSettingsData* qmlRegulatorSettings1 READ qmlRegulatorSettings1 NOTIFY qmlRegulatorSettings1Changed)
    Q_PROPERTY(RegulatorSettingsData* qmlRegulatorSettings2 READ qmlRegulatorSettings2 NOTIFY qmlRegulatorSettings2Changed)
    Q_PROPERTY(CircuitSettingsData* qmlCircuitSettings1 READ qmlCircuitSettings1 NOTIFY qmlCircuitSettings1Changed)
    Q_PROPERTY(CircuitSettingsData* qmlCircuitSettings2 READ qmlCircuitSettings2 NOTIFY qmlCircuitSettings2Changed)

public:
    explicit ROSProxy(QObject *parent = nullptr);

    // 提供只读访问器
    CircuitStatusData circuitStatus1() const;
    CircuitStatusData circuitStatus2() const;
    RegulatorStatusData regulatorStatus1() const;
    RegulatorStatusData regulatorStatus2() const;

    SystemSettingsData *qmlSystemSettings() const;
    RegulatorSettingsData *qmlRegulatorSettings1() const;
    RegulatorSettingsData *qmlRegulatorSettings2() const;
    CircuitSettingsData *qmlCircuitSettings1() const;
    CircuitSettingsData *qmlCircuitSettings2() const;


public slots:
    // QML 将调用这个槽来启动关闭流程
    void initiateShutdown();

    // 用于从 ROS 节点接收 STATUS 数据
    void updateCircuitStatus(const CircuitStatusData &data);
    void updateRegulatorStatus(const RegulatorStatusData &data);

    // --- [CHANGE] Slots now accept ROS message SharedPtrs ---
    void updateSystemSettings(SystemSettingsMsgPtr msg);
    void updateRegulatorSettings(RegulatorSettingsMsgPtr msg);
    void updateCircuitSettings(CircuitSettingsMsgPtr msg);

    // QML可调用的命令发送函数
    Q_INVOKABLE void sendRegulatorOperationCommand(quint8 regulator_id, qt_node_constants::RegulatorOperationCommand command);
    Q_INVOKABLE void sendRegulatorBreakerCommand(quint8 regulator_id, qt_node_constants::RegulatorBreakerCommand command);
    Q_INVOKABLE void sendCircuitModeCommand(quint8 circuit_id, qt_node_constants::CircuitModeCommand command);
    Q_INVOKABLE void sendCircuitBreakerCommand(quint8 circuit_id, qt_node_constants::CircuitBreakerCommand command);
    Q_INVOKABLE void sendClearAlarm();

    // QML设定参数
    Q_INVOKABLE void setSystemSettings(SystemSettingsData* data);
    Q_INVOKABLE void setRegulatorSettings(quint8 regulator_id, RegulatorSettingsData* data);
    Q_INVOKABLE void setCircuitSettings(quint8 circuit_id, CircuitSettingsData* data);
    Q_INVOKABLE void setCircuitReferenceSource(quint8 circuit_id, bool use_ref);

    // --- Slot to receive service call results from ROS node ---
    void onSettingsUpdateResult(const QString &service_name, bool success, const QString &message);
    void onCommandResult(const QString &service_name, bool success, const QString &message);

signals:
    // 这个信号将通知 QtRosNode 开始关闭
    void shutdownRequested();

    // 属性的 NOTIFY 信号
    void circuitStatus1Changed();
    void circuitStatus2Changed();
    void regulatorStatus1Changed();
    void regulatorStatus2Changed();
    void qmlSystemSettingsChanged();
    void qmlRegulatorSettings1Changed();
    void qmlRegulatorSettings2Changed();
    void qmlCircuitSettings1Changed();
    void qmlCircuitSettings2Changed();


    // 用于与ROS节点线程通信的信号
    void regulatorOperationCommandRequested(quint8 regulator_id, quint8 command);
    void regulatorBreakerCommandRequested(quint8 regulator_id, quint8 command);
    void circuitModeCommandRequested(quint8 circuit_id, quint8 command);
    void circuitBreakerCommandRequested(quint8 circuit_id, quint8 command);
    void clearAlarmRequested();

    // --- Signals to request service calls on the ROS thread ---
    void systemSettingsUpdateRequest(SystemSettingsData* data);
    void regulatorSettingsUpdateRequest(quint8 regulator_id, RegulatorSettingsData* data);
    void circuitSettingsUpdateRequest(quint8 circuit_id, CircuitSettingsData* data);

    // --- Signal to notify QML about the result ---
    void settingsUpdateResult(const QString &service_name, bool success, const QString &message);
    void commandResult(const QString &service_name, bool success, const QString &message);

private:
    // 存储数据的成员变量
    CircuitStatusData m_circuitStatus1;
    CircuitStatusData m_circuitStatus2;
    RegulatorStatusData  m_regulatorStatus1;
    RegulatorStatusData  m_regulatorStatus2;

    SystemSettingsData *m_qmlSystemSettings = nullptr;
    RegulatorSettingsData *m_qmlRegulatorSettings1 = nullptr;
    RegulatorSettingsData *m_qmlRegulatorSettings2 = nullptr;
    CircuitSettingsData *m_qmlCircuitSettings1 = nullptr;
    CircuitSettingsData *m_qmlCircuitSettings2 = nullptr;
};

#endif // ROS_PROXY_HPP
