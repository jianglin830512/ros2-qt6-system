#ifndef MOCK_DEVICE_HPP_
#define MOCK_DEVICE_HPP_
#include <thread>
#include <atomic>
#include <mutex>

class MockDevice {
public:
    struct RegulatorState {
        uint8_t id;

        // --- 状态反馈 ---
        bool breaker_closed = false;
        double voltage = 0.0;
        double current = 0.0;
        int8_t direction = 0; // 1: 升压, -1: 降压, 0: 停止
        bool over_voltage_alarm = false;
        bool over_current_alarm = false;
        bool upper_limit_on = false;
        bool lower_limit_on = true;

        std::chrono::steady_clock::time_point last_cmd_time;

        // --- 参数设置 ---
        // 对应 RegulatorSettings.msg
        int32_t over_current_limit = 450;       // [修改] 调压器最大过流默认改为 450A
        int32_t over_voltage_limit = 450;       // [修改] 最大过压默认改为 450V
        int32_t speed_up_percent = 50;          // voltage_up_speed_percent
        int32_t speed_down_percent = 50;        // voltage_down_speed_percent
        bool ovp_enabled = true;                // over_voltage_protection_mode
    };

    struct LoopState {
        // --- 状态反馈 ---
        bool breaker_closed = false;
        double current = 0.0;
        float temperatures[16];
        bool over_current_alarm = false;
        uint8_t plc_mode = 1;

        // --- 参数设置 ---
        // 对应 HardwareLoopSettings.msg
        int32_t max_current_setting = 7200;     // [修改] 最大电流默认改为 7200A
        int32_t start_current_setting = 0;      // start_current_a (恒流设定值)
        int32_t current_change_range = 10;      // current_change_range_percent
        int32_t ct_ratio = 1;                   // ct_ratio
    };

    struct CircuitState {
        uint8_t id;
        LoopState test_loop;
        LoopState ref_loop;
    };

    MockDevice();
    ~MockDevice();

    // --- 控制接口 (由 Driver 调用) ---
    void set_regulator_breaker(uint8_t id, bool close);
    void set_loop_breaker(uint8_t circ_id, uint8_t command);
    void set_regulator_op(uint8_t id, uint8_t cmd);
    void clear_alarms();

    // PLC 模式和源
    void set_plc_mode(uint8_t circ_id, uint8_t loop_type, uint8_t mode);

    // 更新设置接口，传递完整结构体
    void update_reg_settings(uint8_t id, const RegulatorState& new_settings);
    void update_circ_settings(uint8_t id, const LoopState& test_settings, const LoopState& ref_settings);

    // --- 数据检索接口 (由 Driver 调用) ---
    RegulatorState get_reg(uint8_t id);
    CircuitState get_circ(uint8_t id);

private:
    void plc_cycle();
    void init_data();
    void update_regulator_loops(RegulatorState& reg, LoopState& loop1, LoopState& loop2);

    mutable std::mutex data_mutex_;
    std::thread worker_thread_;
    std::atomic<bool> running_;

    RegulatorState reg1, reg2;
    CircuitState circ1, circ2;
};
#endif // MOCK_DEVICE_HPP_
