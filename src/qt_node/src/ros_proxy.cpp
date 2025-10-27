#include <QDebug>
#include <QVariantMap>
#include <QVariantList>

#include "qt_node/ros_proxy.hpp"

ROSProxy::ROSProxy(QObject *parent) : QObject(parent)
{
    // 初始化数据成员
    m_circuitStatus = CircuitStatusData();
    m_voltageRegulatorStatus = VoltageRegulatorStatusData();
    m_qmlSystemSettings = new SystemSettingsData(this);
    m_qmlRegulatorSettings = new VoltageRegulatorSettingsData(this);
    m_qmlCircuitSettings = new CircuitSettingsData(this);
}

// --- 属性的只读访问器 ---

CircuitStatusData ROSProxy::circuitStatus() const
{
    return m_circuitStatus;
}

VoltageRegulatorStatusData ROSProxy::voltageRegulatorStatus() const
{
    return m_voltageRegulatorStatus;
}

// --- 更新数据的槽函数 ---

void ROSProxy::updateCircuitStatus(const CircuitStatusData &data)
{
    // 更新内部数据并发出通知信号
    m_circuitStatus = data;
    emit circuitStatusChanged();
}

void ROSProxy::updateVoltageRegulatorStatus(const VoltageRegulatorStatusData &data)
{
    // 更新内部数据并发出通知信号
    m_voltageRegulatorStatus = data;
    emit voltageRegulatorStatusChanged();
}

// --- Q_INVOKABLE 方法的实现 ---

void ROSProxy::sendRegulatorCommand(quint8 regulator_id, qt_node_constants::RegulatorCommand command)
{
    // 这个函数在GUI线程被调用，它发射一个信号，由ROS线程中的槽函数接收
    emit regulatorCommandRequested(regulator_id, static_cast<quint8>(command));
}

void ROSProxy::sendCircuitCommand(quint8 circuit_id, qt_node_constants::CircuitCommand command)
{
    emit circuitCommandRequested(circuit_id, static_cast<quint8>(command));
}

void ROSProxy::sendClearAlarm()
{
    emit clearAlarmRequested();
}

// --- Set param service: Q_INVOKABLE methods implementation ---

void ROSProxy::setSystemSettings(SystemSettingsData* data)
{
    emit systemSettingsUpdateRequest(data);
}

void ROSProxy::setRegulatorSettings(quint8 regulator_id, VoltageRegulatorSettingsData* data)
{
    emit regulatorSettingsUpdateRequest(regulator_id, data);
}

void ROSProxy::setCircuitSettings(quint8 circuit_id, CircuitSettingsData* data)
{
    emit circuitSettingsUpdateRequest(circuit_id, data);
}

// --- Set param service: Slot implementation to handle results ---

void ROSProxy::onSettingsUpdateResult(const QString &service_name, bool success, const QString &message)
{
    // Forward the result to QML
    emit settingsUpdateResult(service_name, success, message);
}


void ROSProxy::debugCircuitSettings(int circuitId, const QVariant &settingsFromQml)
{
    qDebug() << "--- [DEBUG] Received call for circuitId:" << circuitId; // 打印ID
    qDebug() << "--- [DEBUG] Received settings variant from QML. Starting inspection... ---";
    qDebug() << "Top-level variant type:" << settingsFromQml.typeName();

    if (!settingsFromQml.canConvert<QVariantMap>()) {
        qDebug() << "[ERROR] Variant cannot be converted to a QVariantMap. Aborting.";
        return;
    }

    QVariantMap rootMap = settingsFromQml.toMap();
    qDebug() << "Top-level keys:" << rootMap.keys();

    QStringList expectedKeys = {"test_loop", "ref_loop", "sample_params"};
    for (const QString& key : expectedKeys) {
        qDebug() << "\n--- Inspecting key:" << key << "---";
        if (!rootMap.contains(key)) {
            qDebug() << "[ERROR] Key does not exist!";
            continue;
        }

        QVariant subVariant = rootMap[key];
        qDebug() << "  Sub-variant type:" << subVariant.typeName();

        if (!subVariant.canConvert<QVariantMap>()) {
            qDebug() << "[ERROR] Sub-variant for key" << key << "is not a map!";
            continue;
        }

        QVariantMap subMap = subVariant.toMap();
        qDebug() << "  Sub-map keys:" << subMap.keys();
        qDebug() << "  --- Sub-map key-value pairs and their types ---";
        for (auto it = subMap.constBegin(); it != subMap.constEnd(); ++it) {
            qDebug().nospace() << "    - key: '" << it.key()
            << "', value: '" << it.value().toString() // .toString() for display
            << "', type: " << it.value().typeName();
        }
    }
    qDebug() << "\n--- [DEBUG] Inspection finished. ---";
}

SystemSettingsData *ROSProxy::qmlSystemSettings() const
{
    return m_qmlSystemSettings;
}

VoltageRegulatorSettingsData *ROSProxy::qmlRegulatorSettings() const
{
    return m_qmlRegulatorSettings;
}

CircuitSettingsData *ROSProxy::qmlCircuitSettings() const
{
    return m_qmlCircuitSettings;
}
