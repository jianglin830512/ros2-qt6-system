// include/qt_node/data_types.hpp

#ifndef DATA_TYPES_HPP
#define DATA_TYPES_HPP

#include <QObject>
#include <QString>
#include <QVector>

// =======================================================================
//               LoopStatusData - 单个回路状态的POD
// =======================================================================
struct LoopStatusData
{
    Q_GADGET // 允许元对象系统处理

    // --- 成员变量 ---
    Q_PROPERTY(double current MEMBER current)
    Q_PROPERTY(QVector<double> temperature_array MEMBER temperature_array)
    Q_PROPERTY(int elapsed_heating_time_sec MEMBER elapsed_heating_time_sec)
    Q_PROPERTY(int remaining_heating_time_sec MEMBER remaining_heating_time_sec)
    Q_PROPERTY(quint16 completed_cycle_count MEMBER completed_cycle_count)
    Q_PROPERTY(quint16 remaining_cycle_count MEMBER remaining_cycle_count)
    Q_PROPERTY(bool breaker_closed_switch_ack MEMBER breaker_closed_switch_ack)
    Q_PROPERTY(bool breaker_opened_switch_ack MEMBER breaker_opened_switch_ack)

public:
    double current = 0.0;
    QVector<double> temperature_array;
    int elapsed_heating_time_sec = 0;
    int remaining_heating_time_sec = 0;
    quint16 completed_cycle_count = 0;
    quint16 remaining_cycle_count = 0;
    bool breaker_closed_switch_ack = false;
    bool breaker_opened_switch_ack = false;
};

// 只要将 LoopStatusData 用作 Q_PROPERTY，就必须提供这个比较函数
inline bool operator==(const LoopStatusData& lhs, const LoopStatusData& rhs)
{
    // 逐个比较所有成员变量
    return lhs.current == rhs.current &&
           lhs.temperature_array == rhs.temperature_array && // QVector 已经支持 '=='
           lhs.elapsed_heating_time_sec == rhs.elapsed_heating_time_sec &&
           lhs.remaining_heating_time_sec == rhs.remaining_heating_time_sec &&
           lhs.completed_cycle_count == rhs.completed_cycle_count &&
           lhs.remaining_cycle_count == rhs.remaining_cycle_count&&
           lhs.breaker_closed_switch_ack == rhs.breaker_closed_switch_ack&&
           lhs.breaker_opened_switch_ack == rhs.breaker_opened_switch_ack;
}

inline bool operator!=(const LoopStatusData& lhs, const LoopStatusData& rhs)
{
    return !(lhs == rhs);
}

// =======================================================================
//               CircuitStatusData - 回路状态的POD
// =======================================================================
struct CircuitStatusData
{
    Q_GADGET

    // --- 成员变量，对应ROS消息字段 ---
    Q_PROPERTY(quint8 circuit_id MEMBER circuit_id)
    // --- [修改] 使用 LoopStatusData 替换独立的 test/ref 字段 ---
    Q_PROPERTY(LoopStatusData test_loop MEMBER test_loop)
    Q_PROPERTY(LoopStatusData ref_loop MEMBER ref_loop)
    // --- 全局状态字段 ---
    Q_PROPERTY(quint8 control_mode MEMBER control_mode)
    Q_PROPERTY(bool curr_mode_use_ref MEMBER curr_mode_use_ref) // 假设这个字段也在新msg中

public:
    quint8 circuit_id = 0;
    LoopStatusData test_loop;
    LoopStatusData ref_loop;
    quint8 control_mode = 0;

    bool curr_mode_use_ref = false; // 假设
};

// =======================================================================
//               RegulatorStatusData - 调压器状态的POD
// =======================================================================
struct RegulatorStatusData
{
    Q_GADGET

    Q_PROPERTY(quint8 regulator_id MEMBER regulator_id)
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
    quint8 regulator_id = 0;
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

// --- [修改] 增加新的元类型声明 ---
Q_DECLARE_METATYPE(LoopStatusData)
Q_DECLARE_METATYPE(CircuitStatusData)
Q_DECLARE_METATYPE(RegulatorStatusData)

#endif // DATA_TYPES_HPP
