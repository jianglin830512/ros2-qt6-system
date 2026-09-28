#ifndef ROS_PROXY_HPP
#define ROS_PROXY_HPP

#include <QObject>
#include <QVariantMap>
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
    Q_PROPERTY(SystemStatusData systemStatus READ systemStatus NOTIFY systemStatusChanged)
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
    SystemStatusData systemStatus() const;

    SystemSettingsData *qmlSystemSettings() const;
    RegulatorSettingsData *qmlRegulatorSettings1() const;
    RegulatorSettingsData *qmlRegulatorSettings2() const;
    CircuitSettingsData *qmlCircuitSettings1() const;
    CircuitSettingsData *qmlCircuitSettings2() const;

    // QML可调用的命令发送函数
    Q_INVOKABLE void sendRegulatorOperationCommand(quint8 regulator_id, qt_node_constants::RegulatorOperationCommand command);
    Q_INVOKABLE void sendRegulatorBreakerCommand(quint8 regulator_id, qt_node_constants::RegulatorBreakerCommand command);
    Q_INVOKABLE void sendCircuitBreakerCommand(quint8 circuit_id, qt_node_constants::CircuitBreakerCommand command);
    Q_INVOKABLE void sendClearAlarm();

    // QML设定参数
    Q_INVOKABLE void setSystemSettings(SystemSettingsData* data);
    Q_INVOKABLE void setRegulatorSettings(quint8 regulator_id, RegulatorSettingsData* data);
    Q_INVOKABLE void setCircuitSettings(quint8 circuit_id, CircuitSettingsData* data);

    // QML 调用的接口 (历史与表格查询)
    Q_INVOKABLE void queryHistory(const QString& dateStr, const QString& timeStr, int spanHours, const QStringList& cols);
    Q_INVOKABLE void queryTable(const QString& dateStr, const QString& timeStr, int spanHours, int circuitId);

    // 运行历史数据导出
    Q_INVOKABLE void exportData(const QString& start_date, const QString& end_date, int circuit_id, const QString& file_path);

    // 电缆管理
    Q_INVOKABLE void listCables(const QString& keyword, int page, int pageSize, int sortColumn, bool isAscending);
    Q_INVOKABLE void saveCable(const QVariantMap& cableMap);
    Q_INVOKABLE void deleteCable(int id);

    // 试验管理
    Q_INVOKABLE void listTestRecords(const QString& keyword, int page, int pageSize, int circuitId);
    Q_INVOKABLE void saveTestRecord(const QVariantMap& recordMap);
    Q_INVOKABLE void deleteTestRecord(int id);
    Q_INVOKABLE void exportTestRecords(const QString& file_path);

public slots:
    // QML 将调用这个槽来启动关闭流程
    void initiateShutdown();

    // 用于从 ROS 节点接收 STATUS 数据
    void updateCircuitStatus(const CircuitStatusData &data);
    void updateRegulatorStatus(const RegulatorStatusData &data);
    void updateSystemStatus(const SystemStatusData &data);

    // Slots accept ROS message SharedPtrs
    void updateSystemSettings(SystemSettingsMsgPtr msg);
    void updateRegulatorSettings(RegulatorSettingsMsgPtr msg);
    void updateCircuitSettings(CircuitSettingsMsgPtr msg);

    // 历史数据与表格数据
    void onHistoryDataFetched(const QVariantMap& data);
    void onHistoryQueryFailed(const QString& msg);
    void onTableDataFetched(const QVariantMap& data);
    void onTableQueryFailed(const QString& msg);

    // 接收服务端写入结果
    void onSettingsUpdateResult(const QString &service_name, bool success, const QString &message);
    void onCommandResult(const QString &service_name, bool success, const QString &message);

    // 运行历史数据导出结果
    void onExportProgress(int percentage);
    void onExportFinished(bool success, const QString& message);

    // 电缆管理结果
    void onCablesListed(const QVariantMap& result);
    void onCableSaveResult(bool success, const QString& msg);
    void onCableDeleteResult(bool success, const QString& msg);

    // 试验管理结果
    void onTestRecordsListed(int circuitId, const QVariantMap& result);
    void onTestRecordSaveResult(bool success, const QString& msg);
    void onTestRecordDeleteResult(bool success, const QString& msg);
    void onExportTestRecordResult(bool success, const QString& msg);

signals:
    // 这个信号将通知 QtRosNode 开始关闭
    void shutdownRequested();

    // 属性的 NOTIFY 信号
    void circuitStatus1Changed();
    void circuitStatus2Changed();
    void regulatorStatus1Changed();
    void regulatorStatus2Changed();
    void systemStatusChanged();
    void qmlSystemSettingsChanged();
    void qmlRegulatorSettings1Changed();
    void qmlRegulatorSettings2Changed();
    void qmlCircuitSettings1Changed();
    void qmlCircuitSettings2Changed();

    // 用于与ROS节点线程通信的信号 (命令下发)
    void regulatorOperationCommandRequested(quint8 regulator_id, quint8 command);
    void regulatorBreakerCommandRequested(quint8 regulator_id, quint8 command);
    void circuitBreakerCommandRequested(quint8 circuit_id, quint8 command);
    void clearAlarmRequested();

    // 用于与ROS节点线程通信的信号 (参数下发)
    void systemSettingsUpdateRequest(SystemSettingsData* data);
    void regulatorSettingsUpdateRequest(quint8 regulator_id, RegulatorSettingsData* data);
    void circuitSettingsUpdateRequest(quint8 circuit_id, CircuitSettingsData* data);

    // 通知 QML 关于设置和服务的结果
    void settingsUpdateResult(const QString &service_name, bool success, const QString &message);
    void commandResult(const QString &service_name, bool success, const QString &message);

    // 历史数据请求与返回
    void historyQueryRequested(const QString& start_time_str, int duration, const QStringList& columns);
    void historyDataReady(const QVariantMap& data);
    void historyQueryError(const QString& msg);

    // 表格数据请求与返回
    void tableQueryRequested(const QString& start_time_str, int duration, int circuit_id);
    void tableDataReady(const QVariantMap& data);
    void tableQueryError(const QString& msg);

    // 运行历史数据导出
    void exportDataRequested(const QString& start_date, const QString& end_date, int circuit_id, const QString& file_path);
    void exportProgressChanged(int percentage);
    void exportResult(bool success, const QString& message);

    // 电缆管理请求与返回
    void listCablesRequested(const QString& keyword, int page, int pageSize, int sortColumn, bool isAscending);
    void saveCableRequested(const QVariantMap& cableMap);
    void deleteCableRequested(int id);
    void cablesListed(const QVariantMap& result);
    void cableSaveResult(bool success, const QString& msg);
    void cableDeleteResult(bool success, const QString& msg);

    // 试验管理请求与返回
    void listTestRecordsRequested(const QString& keyword, int page, int pageSize, int circuitId);
    void saveTestRecordRequested(const QVariantMap& recordMap);
    void deleteTestRecordRequested(int id);
    void exportTestRecordsRequested(const QString& file_path);
    void testRecordsListed(int circuitId, const QVariantMap& result);
    void testRecordSaveResult(bool success, const QString& msg);
    void testRecordDeleteResult(bool success, const QString& msg);
    void exportTestRecordResult(bool success, const QString& msg);

private:
    // 存储数据的成员变量
    CircuitStatusData m_circuitStatus1;
    CircuitStatusData m_circuitStatus2;
    RegulatorStatusData  m_regulatorStatus1;
    RegulatorStatusData  m_regulatorStatus2;
    SystemStatusData m_systemStatus;

    SystemSettingsData *m_qmlSystemSettings = nullptr;
    RegulatorSettingsData *m_qmlRegulatorSettings1 = nullptr;
    RegulatorSettingsData *m_qmlRegulatorSettings2 = nullptr;
    CircuitSettingsData *m_qmlCircuitSettings1 = nullptr;
    CircuitSettingsData *m_qmlCircuitSettings2 = nullptr;
};

#endif // ROS_PROXY_HPP
