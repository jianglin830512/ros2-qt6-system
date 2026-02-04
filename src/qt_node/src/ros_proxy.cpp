#include <QDebug>
#include <QVariantMap>
#include <QVariantList>

#include "qt_node/ros_proxy.hpp"

namespace {

void convertAndApply(const ros2_interfaces::msg::SystemSettings::SharedPtr& msg, SystemSettingsData* target) {
    if (!msg || !target) return;

    target->blockSignals(true);
    target->setSample_interval_sec(msg->sample_interval_sec);
    target->setRecord_interval_min(msg->record_interval_min);
    target->setKeep_record_on_shutdown(msg->keep_record_on_shutdown);
    target->blockSignals(false);
}

void convertAndApply(const ros2_interfaces::msg::RegulatorSettings::SharedPtr& msg, RegulatorSettingsData* target) {
    if (!msg || !target) return;

    target->blockSignals(true);
    target->setOver_current_a(msg->over_current_a);
    target->setOver_voltage_v(msg->over_voltage_v);
    target->setVoltage_up_speed_percent(msg->voltage_up_speed_percent);
    target->setVoltage_down_speed_percent(msg->voltage_down_speed_percent);
    target->setOver_voltage_protection_mode(msg->over_voltage_protection_mode);
    target->blockSignals(false);
}
// Helper for the helper
void convertAndApplyLoop(const ros2_interfaces::msg::LoopSettings& ros_loop, LoopSettingsData* qt_loop) {
    qt_loop->blockSignals(true);
    qt_loop->setStart_current_a(ros_loop.hardware_loop_settings.start_current_a);
    qt_loop->setMax_current_a(ros_loop.hardware_loop_settings.max_current_a);
    qt_loop->setCurrent_change_range_percent(ros_loop.hardware_loop_settings.current_change_range_percent);
    qt_loop->setCt_ratio(ros_loop.hardware_loop_settings.ct_ratio);
    qt_loop->setCycle_count(ros_loop.cycle_count);
    qt_loop->setStart_date(QDateTime::fromSecsSinceEpoch(ros_loop.start_date.sec));
    qt_loop->setHeating_start_time_sec(ros_loop.heating_time.sec);

    qt_loop->setHeating_duration_sec(ros_loop.heating_duration.sec);
    qt_loop->setEnabled(ros_loop.enabled);
    qt_loop->blockSignals(false);
}

// Helper for the helper
void convertAndApplySample(const ros2_interfaces::msg::SampleSettings& ros_sample, SampleSettingsData* qt_sample) {
    qt_sample->blockSignals(true);
    qt_sample->setCable_type(QString::fromStdString(ros_sample.cable_type));
    qt_sample->setCable_spec(QString::fromStdString(ros_sample.cable_spec));
    qt_sample->setInsulation_material(QString::fromStdString(ros_sample.insulation_material));
    qt_sample->setInsulation_thickness(ros_sample.insulation_thickness);
    qt_sample->blockSignals(false);
}

void convertAndApply(const ros2_interfaces::msg::CircuitSettings::SharedPtr& msg, CircuitSettingsData* target) {
    if (!msg || !target) return;

    // Convert nested objects first
    convertAndApplyLoop(msg->test_loop, target->test_loop());
    convertAndApplyLoop(msg->ref_loop, target->ref_loop());
    convertAndApplySample(msg->sample_params, target->sample_params());

    // Set the direct property
    target->setCurr_mode_use_ref(msg->curr_mode_use_ref);
}

} // End anonymous namespace

ROSProxy::ROSProxy(QObject *parent) : QObject(parent)
{
    // 初始化数据成员
    m_circuitStatus1 = CircuitStatusData();
    m_circuitStatus2 = CircuitStatusData();
    m_regulatorStatus1 = RegulatorStatusData();
    m_regulatorStatus2 = RegulatorStatusData();

    m_qmlSystemSettings = new SystemSettingsData(this);
    m_qmlRegulatorSettings1 = new RegulatorSettingsData(this);
    m_qmlRegulatorSettings2 = new RegulatorSettingsData(this);
    m_qmlCircuitSettings1 = new CircuitSettingsData(this);
    m_qmlCircuitSettings2 = new CircuitSettingsData(this);
}

// --- 属性的只读访问器 ---

CircuitStatusData ROSProxy::circuitStatus1() const
{
    return m_circuitStatus1;
}

CircuitStatusData ROSProxy::circuitStatus2() const
{
    return m_circuitStatus2;
}

RegulatorStatusData  ROSProxy::regulatorStatus1() const
{
    return m_regulatorStatus1;
}

RegulatorStatusData  ROSProxy::regulatorStatus2() const
{
    return m_regulatorStatus2;
}

SystemSettingsData *ROSProxy::qmlSystemSettings() const
{
    return m_qmlSystemSettings;
}

RegulatorSettingsData *ROSProxy::qmlRegulatorSettings1() const
{
    return m_qmlRegulatorSettings1;
}

RegulatorSettingsData *ROSProxy::qmlRegulatorSettings2() const
{
    return m_qmlRegulatorSettings2;
}

CircuitSettingsData *ROSProxy::qmlCircuitSettings1() const
{
    return m_qmlCircuitSettings1;
}

CircuitSettingsData *ROSProxy::qmlCircuitSettings2() const
{
    return m_qmlCircuitSettings2;
}


// --- 更新数据的槽函数 ---

void ROSProxy::updateCircuitStatus(const CircuitStatusData &data)
{
    // 根据传入数据的ID，直接更新对应的成员变量并发出信号
    if (data.circuit_id == 1) {
        m_circuitStatus1 = data;
        emit circuitStatus1Changed();
    } else if (data.circuit_id == 2) {
        m_circuitStatus2 = data;
        emit circuitStatus2Changed();
    }
}

void ROSProxy::updateRegulatorStatus(const RegulatorStatusData &data)
{
    if (data.regulator_id == 1) { // 字段名变更
        m_regulatorStatus1 = data;
        emit regulatorStatus1Changed(); // 信号名变更
    } else if (data.regulator_id == 2) {
        m_regulatorStatus2 = data;
        emit regulatorStatus2Changed(); // 信号名变更
    }
}

void ROSProxy::updateSystemSettings(SystemSettingsMsgPtr msg)
{
    convertAndApply(msg, m_qmlSystemSettings);
    emit qmlSystemSettingsChanged(); // Manually emit the top-level signal
}

void ROSProxy::updateRegulatorSettings(RegulatorSettingsMsgPtr msg)
{
    if (!msg) return;

    if (msg->regulator_id == 1) {
        convertAndApply(msg, m_qmlRegulatorSettings1);
        emit qmlRegulatorSettings1Changed();
    } else if (msg->regulator_id == 2) {
        convertAndApply(msg, m_qmlRegulatorSettings2);
        emit qmlRegulatorSettings2Changed();
    }
}

void ROSProxy::updateCircuitSettings(CircuitSettingsMsgPtr msg)
{
    if (!msg) return;

    if (msg->circuit_id == 1) {
        convertAndApply(msg, m_qmlCircuitSettings1);
        emit qmlCircuitSettings1Changed();
    } else if (msg->circuit_id == 2) {
        convertAndApply(msg, m_qmlCircuitSettings2);
        emit qmlCircuitSettings2Changed();
    }
}

// --- Q_INVOKABLE 方法的实现 ---
void ROSProxy::initiateShutdown()
{
    // 只是简单地发出信号，将任务传递给后端
    emit shutdownRequested();
}

void ROSProxy::sendRegulatorOperationCommand(quint8 regulator_id, qt_node_constants::RegulatorOperationCommand command)
{
    emit regulatorOperationCommandRequested(regulator_id, static_cast<quint8>(command));
}

void ROSProxy::sendRegulatorBreakerCommand(quint8 regulator_id, qt_node_constants::RegulatorBreakerCommand command)
{
    emit regulatorBreakerCommandRequested(regulator_id, static_cast<quint8>(command));
}

void ROSProxy::sendCircuitModeCommand(quint8 circuit_id, qt_node_constants::CircuitModeCommand command)
{
    emit circuitModeCommandRequested(circuit_id, static_cast<quint8>(command));
}

void ROSProxy::sendCircuitBreakerCommand(quint8 circuit_id, qt_node_constants::CircuitBreakerCommand command)
{
    emit circuitBreakerCommandRequested(circuit_id, static_cast<quint8>(command));
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

void ROSProxy::setRegulatorSettings(quint8 regulator_id, RegulatorSettingsData* data)
{
    emit regulatorSettingsUpdateRequest(regulator_id, data);
}

void ROSProxy::setCircuitSettings(quint8 circuit_id, CircuitSettingsData* data)
{
    emit circuitSettingsUpdateRequest(circuit_id, data);
}

void ROSProxy::setCircuitReferenceSource(quint8 circuit_id, bool use_ref)
{
    CircuitSettingsData* targetData = nullptr;

    if (circuit_id == 1) {
        targetData = m_qmlCircuitSettings1;
    } else if (circuit_id == 2) {
        targetData = m_qmlCircuitSettings2;
    }

    if (targetData) {
        // 修改本地缓存的数据对象
        targetData->setCurr_mode_use_ref(use_ref);
        // 发送更新请求 (复用已有的信号，QtROSNode 会处理)
        emit circuitSettingsUpdateRequest(circuit_id, targetData);
    } else {
        qWarning() << "setCircuitReferenceSource: Data is null for circuit" << circuit_id;
    }
}

// --- Set param service: Slot implementation to handle results ---

void ROSProxy::onSettingsUpdateResult(const QString &service_name, bool success, const QString &message)
{
    // Forward the result to QML
    emit settingsUpdateResult(service_name, success, message);
}

void ROSProxy::onCommandResult(const QString &service_name, bool success, const QString &message)
{
    emit commandResult(service_name, success, message);
}

