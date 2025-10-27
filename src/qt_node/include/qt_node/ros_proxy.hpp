#ifndef ROS_PROXY_HPP
#define ROS_PROXY_HPP

#include <QObject>
#include "qt_node/data_types/data_types.hpp" // 引入简单的数据类型
#include "qt_node/data_types/circuit_settings_data.hpp"
#include "qt_node/data_types/voltage_regulator_settings_data.hpp"
#include "qt_node/data_types/system_settings_data.hpp"
#include "qt_node/qt_node_constants.hpp" // 引入常量定义

class ROSProxy : public QObject
{
    Q_OBJECT

    // 定义 Q_PROPERTY，使其可以直接在 QML 中访问
    // STATUS
    Q_PROPERTY(CircuitStatusData circuitStatus READ circuitStatus NOTIFY circuitStatusChanged)
    Q_PROPERTY(VoltageRegulatorStatusData voltageRegulatorStatus READ voltageRegulatorStatus NOTIFY voltageRegulatorStatusChanged)
    // SETTINGS
    Q_PROPERTY(SystemSettingsData* qmlSystemSettings READ qmlSystemSettings CONSTANT)
    Q_PROPERTY(VoltageRegulatorSettingsData* qmlRegulatorSettings READ qmlRegulatorSettings CONSTANT)
    Q_PROPERTY(CircuitSettingsData* qmlCircuitSettings READ qmlCircuitSettings CONSTANT)
public:
    explicit ROSProxy(QObject *parent = nullptr);

    // 提供只读访问器
    CircuitStatusData circuitStatus() const;
    VoltageRegulatorStatusData voltageRegulatorStatus() const;

    // DEBUG
    Q_INVOKABLE void debugCircuitSettings(int circuitId, const QVariant &settingsFromQml);

    SystemSettingsData *qmlSystemSettings() const;

    VoltageRegulatorSettingsData *qmlRegulatorSettings() const;

    CircuitSettingsData *qmlCircuitSettings() const;

public slots:
    // 用于从 ROS 节点接收 STATUS 数据
    void updateCircuitStatus(const CircuitStatusData &data);
    void updateVoltageRegulatorStatus(const VoltageRegulatorStatusData &data);

    // QML可调用的命令发送函数
    Q_INVOKABLE void sendRegulatorCommand(quint8 regulator_id, qt_node_constants::RegulatorCommand command);
    Q_INVOKABLE void sendCircuitCommand(quint8 circuit_id, qt_node_constants::CircuitCommand command);
    Q_INVOKABLE void sendClearAlarm();

    // QML设定参数
    Q_INVOKABLE void setSystemSettings(SystemSettingsData* data);
    Q_INVOKABLE void setRegulatorSettings(quint8 regulator_id, VoltageRegulatorSettingsData* data);
    Q_INVOKABLE void setCircuitSettings(quint8 circuit_id, CircuitSettingsData* data);

    // --- Slot to receive service call results from ROS node ---
    void onSettingsUpdateResult(const QString &service_name, bool success, const QString &message);

signals:
    // 属性的 NOTIFY 信号
    void circuitStatusChanged();
    void voltageRegulatorStatusChanged();

    // 用于与ROS节点线程通信的信号
    void regulatorCommandRequested(quint8 regulator_id, quint8 command);
    void circuitCommandRequested(quint8 circuit_id, quint8 command);
    void clearAlarmRequested();

    // --- Signals to request service calls on the ROS thread ---
    void systemSettingsUpdateRequest(SystemSettingsData* data);
    void regulatorSettingsUpdateRequest(quint8 regulator_id, VoltageRegulatorSettingsData* data);
    void circuitSettingsUpdateRequest(quint8 circuit_id, CircuitSettingsData* data);

    // --- Signal to notify QML about the result ---
    void settingsUpdateResult(const QString &service_name, bool success, const QString &message);

private:
    // 存储数据的成员变量
    CircuitStatusData m_circuitStatus;
    VoltageRegulatorStatusData m_voltageRegulatorStatus;
    SystemSettingsData *m_qmlSystemSettings = nullptr;
    VoltageRegulatorSettingsData *m_qmlRegulatorSettings = nullptr;
    CircuitSettingsData *m_qmlCircuitSettings = nullptr;
};

#endif // ROS_PROXY_HPP
