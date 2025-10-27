// include/qt_node/data_types.hpp

#ifndef DATA_TYPES_HPP
#define DATA_TYPES_HPP

#include <QObject>
#include <QString>
#include <QVector>
#include <QDate>
#include <QTime>

// =======================================================================
//               CircuitStatusData - 回路状态的POD
// =======================================================================
struct CircuitStatusData
{
    Q_GADGET // 允许元对象系统处理，以便在QML中使用

    // --- 成员变量，对应ROS消息字段 ---
    Q_PROPERTY(quint8 circuit_id MEMBER circuit_id)
    Q_PROPERTY(double test_current MEMBER test_current)
    Q_PROPERTY(double ref_current MEMBER ref_current)
    Q_PROPERTY(QVector<double> test_temperature_array MEMBER test_temperature_array)
    Q_PROPERTY(QVector<double> ref_temperature_array MEMBER ref_temperature_array)
    Q_PROPERTY(int elapsed_heating_time_sec MEMBER elapsed_heating_time_sec)
    Q_PROPERTY(int remaining_heating_time_sec MEMBER remaining_heating_time_sec)
    Q_PROPERTY(quint16 completed_cycle_count MEMBER completed_cycle_count)
    Q_PROPERTY(quint16 remaining_cycle_count MEMBER remaining_cycle_count)
    Q_PROPERTY(quint8 control_mode MEMBER control_mode)
    Q_PROPERTY(bool circuit_enabled MEMBER circuit_enabled)
    Q_PROPERTY(bool breaker_closed_switch_ack MEMBER breaker_closed_switch_ack)
    Q_PROPERTY(bool breaker_opened_switch_ack MEMBER breaker_opened_switch_ack)

public:
    quint8 circuit_id = 0;
    double test_current = 0.0;
    double ref_current = 0.0;
    QVector<double> test_temperature_array;
    QVector<double> ref_temperature_array;
    int elapsed_heating_time_sec = 0;
    int remaining_heating_time_sec = 0;
    quint16 completed_cycle_count = 0;
    quint16 remaining_cycle_count = 0;
    quint8 control_mode = 0;
    bool circuit_enabled = false;
    bool breaker_closed_switch_ack = false;
    bool breaker_opened_switch_ack = false;
};

// =======================================================================
//               VoltageRegulatorStatusData - 调压器状态的POD
// =======================================================================
struct VoltageRegulatorStatusData
{
    Q_GADGET

    Q_PROPERTY(quint8 voltage_regulator_id MEMBER voltage_regulator_id)
    Q_PROPERTY(double voltage_reading MEMBER voltage_reading)
    Q_PROPERTY(double current_reading MEMBER current_reading)
    Q_PROPERTY(qint8 voltage_direction MEMBER voltage_direction)
    Q_PROPERTY(bool breaker_closed_switch_ack MEMBER breaker_closed_switch_ack)
    Q_PROPERTY(bool breaker_opened_switch_ack MEMBER breaker_opened_switch_ack)
    Q_PROPERTY(bool upper_limit_switch_on MEMBER upper_limit_switch_on)
    Q_PROPERTY(bool lower_limit_switch_on MEMBER lower_limit_switch_on)
    Q_PROPERTY(bool over_current_on MEMBER over_current_on)
    Q_PROPERTY(bool over_voltage_on MEMBER over_voltage_on)

public:
    quint8 voltage_regulator_id = 0;
    double voltage_reading = 0.0;
    double current_reading = 0.0;
    qint8 voltage_direction = 0;
    bool breaker_closed_switch_ack = false;
    bool breaker_opened_switch_ack = false;
    bool upper_limit_switch_on = false;
    bool lower_limit_switch_on = false;
    bool over_current_on = false;
    bool over_voltage_on = false;
};



// 注册类型，以便它们可以用于信号/槽机制
Q_DECLARE_METATYPE(CircuitStatusData)
Q_DECLARE_METATYPE(VoltageRegulatorStatusData)

#endif // DATA_TYPES_HPP
