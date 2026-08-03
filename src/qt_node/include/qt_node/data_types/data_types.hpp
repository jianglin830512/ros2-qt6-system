// include/qt_node/data_types/data_types.hpp
#ifndef DATA_TYPES_HPP
#define DATA_TYPES_HPP

#include <QObject>
#include <QString>
#include <QVector>

struct LoopStatusData {
    Q_GADGET
    Q_PROPERTY(bool is_heat MEMBER is_heat)
    Q_PROPERTY(double current MEMBER current)
    Q_PROPERTY(QVector<double> temperature_array MEMBER temperature_array)
    Q_PROPERTY(int elapsed_heating_time_sec MEMBER elapsed_heating_time_sec)
    Q_PROPERTY(int remaining_heating_time_sec MEMBER remaining_heating_time_sec)
    Q_PROPERTY(quint16 completed_cycle_count MEMBER completed_cycle_count)
    Q_PROPERTY(quint16 remaining_cycle_count MEMBER remaining_cycle_count)
    Q_PROPERTY(bool breaker_closed_switch_ack MEMBER breaker_closed_switch_ack)
    Q_PROPERTY(bool breaker_opened_switch_ack MEMBER breaker_opened_switch_ack)
    Q_PROPERTY(quint8 plc_control_mode MEMBER plc_control_mode) // 新增：PLC反馈模式
public:
    bool is_heat = false;
    double current = 0.0;
    QVector<double> temperature_array;
    int elapsed_heating_time_sec = 0;
    int remaining_heating_time_sec = 0;
    quint16 completed_cycle_count = 0;
    quint16 remaining_cycle_count = 0;
    bool breaker_closed_switch_ack = false;
    bool breaker_opened_switch_ack = false;
    quint8 plc_control_mode = 1;
};

inline bool operator==(const LoopStatusData& lhs, const LoopStatusData& rhs) {
    return lhs.is_heat == rhs.is_heat && lhs.current == rhs.current &&
           lhs.temperature_array == rhs.temperature_array &&
           lhs.elapsed_heating_time_sec == rhs.elapsed_heating_time_sec &&
           lhs.remaining_heating_time_sec == rhs.remaining_heating_time_sec &&
           lhs.completed_cycle_count == rhs.completed_cycle_count &&
           lhs.remaining_cycle_count == rhs.remaining_cycle_count &&
           lhs.breaker_closed_switch_ack == rhs.breaker_closed_switch_ack &&
           lhs.breaker_opened_switch_ack == rhs.breaker_opened_switch_ack &&
           lhs.plc_control_mode == rhs.plc_control_mode;
}
inline bool operator!=(const LoopStatusData& lhs, const LoopStatusData& rhs) { return !(lhs == rhs); }

struct CircuitStatusData {
    Q_GADGET
    Q_PROPERTY(quint8 circuit_id MEMBER circuit_id)
    Q_PROPERTY(LoopStatusData test_loop MEMBER test_loop)
    Q_PROPERTY(LoopStatusData ref_loop MEMBER ref_loop)
    // 删除了 control_mode 和 curr_mode_use_ref
public:
    quint8 circuit_id = 0;
    LoopStatusData test_loop;
    LoopStatusData ref_loop;
};

struct RegulatorStatusData {
    // 保持不变
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

struct SystemStatusData {
    Q_GADGET
    Q_PROPERTY(bool is_remote MEMBER is_remote)
    Q_PROPERTY(bool emergency_stop_on MEMBER emergency_stop_on)
    Q_PROPERTY(uint8_t system_state MEMBER system_state)
    Q_PROPERTY(uint8_t circuit_work_status MEMBER circuit_work_status) // 新增：全局工作状态
    Q_PROPERTY(bool hardware_connected MEMBER hardware_connected)       // 新增：硬件总连接状态
    Q_PROPERTY(bool plc_connected MEMBER plc_connected)                 // 新增：PLC连接状态
    Q_PROPERTY(bool temp_monitor_connected MEMBER temp_monitor_connected) // 新增：测温设备连接状态
public:
    bool is_remote = false;
    bool emergency_stop_on = false;
    uint8_t system_state = 0;
    uint8_t circuit_work_status = 0;
    bool hardware_connected = false;
    bool plc_connected = false;
    bool temp_monitor_connected = false;
};

Q_DECLARE_METATYPE(LoopStatusData)
Q_DECLARE_METATYPE(CircuitStatusData)
Q_DECLARE_METATYPE(RegulatorStatusData)
Q_DECLARE_METATYPE(SystemStatusData)

#endif // DATA_TYPES_HPP
