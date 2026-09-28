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
        bool auto_reduce_opening = false;

        std::chrono::steady_clock::time_point last_cmd_time;

        // --- 参数设置 ---
        int32_t over_current_limit = 450;
        int32_t over_voltage_limit = 450;
        int32_t speed_up_percent = 50;
        int32_t speed_down_percent = 50;
        bool ovp_enabled = true;
    };

    struct LoopState {
        // --- 状态反馈 ---
        bool breaker_closed = false;
        double current = 0.0;
        bool over_current_alarm = false;
        uint8_t plc_mode = 1;

        // --- 参数设置 ---
        int32_t max_current_setting = 7200;
        int32_t start_current_setting = 0;
        int32_t current_change_range = 10;
        int32_t ct_ratio = 1;
    };

    struct CircuitState {
        uint8_t id;
        LoopState test_loop;
        LoopState ref_loop;

        // 剥离支路层，回路直接维护 40路温度
        float temperatures[40];
    };

    MockDevice();
    ~MockDevice();

    // --- 控制接口 (由 Driver 调用) ---
    void set_regulator_breaker(uint8_t id, uint8_t command);
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
