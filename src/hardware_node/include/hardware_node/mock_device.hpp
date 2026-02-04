#ifndef MOCK_DEVICE_HPP_
#define MOCK_DEVICE_HPP_
#include <thread>
#include <atomic>
#include <mutex>
class MockDevice {
public:
    struct RegulatorState {
        uint8_t id;
        bool breaker_closed = false;
        double voltage = 0.0;
        double current = 0.0;
        int8_t direction = 0; // 1: 升压, -1: 降压, 0: 停止
        bool over_voltage_alarm = false;
        bool over_current_alarm = false;
        bool upper_limit_on = false;
        bool lower_limit_on = true;

        // 设置参数
        int32_t over_voltage_limit = 400;
        int32_t over_current_limit = 100;
        int32_t speed_percent = 50;
        bool ovp_enabled = true;
    };

    struct LoopState {
        bool breaker_closed = false;
        double current = 0.0;
        float temperatures[16];
        bool over_current_alarm = false;
        int32_t max_current_setting = 3500;
    };

    struct CircuitState {
        uint8_t id;
        LoopState test_loop;
        LoopState ref_loop;

        // [新增] PLC 控制模式和源
        uint8_t plc_mode = 0;   // 0: Manual, 1: Auto Current
        uint8_t plc_source = 0; // 0: Test Loop, 1: Ref Loop
    };

    MockDevice();
    ~MockDevice();

    // --- 控制接口 (由 Driver 调用) ---
    void set_regulator_breaker(uint8_t id, bool close);
    void set_loop_breaker(uint8_t circ_id, uint8_t command);
    void set_regulator_op(uint8_t id, uint8_t cmd);
    void clear_alarms();

    // [新增] 设置 PLC 模式和源
    void set_plc_mode(uint8_t circ_id, uint8_t mode);
    void set_plc_source(uint8_t circ_id, uint8_t source);

    void update_reg_settings(uint8_t id, const RegulatorState& settings);
    void update_circ_settings(uint8_t id, int32_t test_max, int32_t ref_max);

    // --- 数据检索接口 (由 Driver 调用) ---
    RegulatorState get_reg(uint8_t id);
    CircuitState get_circ(uint8_t id);
private:
    void plc_cycle();
    void init_data();
    void update_regulator(RegulatorState& reg, CircuitState& circ);

    mutable std::mutex data_mutex_;
    std::thread worker_thread_;
    std::atomic<bool> running_;

    RegulatorState reg1, reg2;
    CircuitState circ1, circ2;
};
#endif // MOCK_DEVICE_HPP_
