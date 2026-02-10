#include "hardware_node/mock_device.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>

MockDevice::MockDevice() {
    init_data();
    running_ = true;
    worker_thread_ = std::thread(&MockDevice::plc_cycle, this);
}

MockDevice::~MockDevice() {
    running_ = false;
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

void MockDevice::init_data() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    reg1.id = 1;
    reg2.id = 2;
    circ1.id = 1;
    circ2.id = 2;

    // 定义一个通用的初始化 Lambda，增加 valid_count 参数
    auto init_loop = [](LoopState& l, int valid_count) {
        l.max_current_setting = 3500;
        l.start_current_setting = 1000;
        l.current_change_range = 5;
        l.ct_ratio = 1000;

        // 0 到 valid_count-1 初始化为室温
        for(int i = 0; i < valid_count; ++i) {
            l.temperatures[i] = 20.0f;
        }

        // valid_count 到 15 初始化为 NaN (无效)
        for(int i = valid_count; i < 16; ++i) {
            l.temperatures[i] = std::nanf("");
        }
    };

    // 无论 ID 是 1 还是 2:
    // Test Loop -> 16 个数据
    // Ref Loop  -> 8 个数据

    // 回路 1
    init_loop(circ1.test_loop, 16);
    init_loop(circ1.ref_loop, 8);

    // 回路 2
    init_loop(circ2.test_loop, 16);
    init_loop(circ2.ref_loop, 8);
}

void MockDevice::plc_cycle() {
    while (running_) {
        auto start_time = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            update_regulator(reg1, circ1);
            update_regulator(reg2, circ2);
        }
        std::this_thread::sleep_until(start_time + std::chrono::milliseconds(100));
    }
}

void MockDevice::update_regulator(RegulatorState& reg, CircuitState& circ) {
    // 1. 电压变化逻辑 (区分升/降压速度)
    if (reg.breaker_closed) {
        double max_step = 4.0; // 假设100ms内最大变化4V (40V/s)

        if (reg.direction == 1) {
            // 升压
            double step = max_step * (reg.speed_up_percent / 100.0);
            reg.voltage += step;
        }
        else if (reg.direction == -1) {
            // 降压
            double step = max_step * (reg.speed_down_percent / 100.0);
            reg.voltage -= step;
        }
    }

    // 2. 物理限位
    reg.upper_limit_on = (reg.voltage >= 400.0);
    reg.lower_limit_on = (reg.voltage <= 0.0);

    if (reg.voltage > 400.0) { reg.voltage = 400.0; if(reg.direction == 1) reg.direction = 0; }
    if (reg.voltage < 0.0)   { reg.voltage = 0.0;   if(reg.direction == -1) reg.direction = 0; }

    // 3. 过压保护 (仅当启用时)
    if (reg.ovp_enabled && reg.voltage > reg.over_voltage_limit) {
        reg.over_voltage_alarm = true;
        reg.direction = 0; // 停止
    }

    // 4. 回路耦合与过流保护
    auto update_loop = [&](LoopState& loop) {
        if (loop.breaker_closed && reg.breaker_closed) {
            // 简化的物理模型：电流与电压成正比
            // 注意：CT比率在真实PLC中可能用于计算，这里简单模拟
            loop.current = (reg.voltage / 400.0) * 3200.0;

            // 温度跟随电流变化
            float base_temp = 20.0f + (float)(reg.voltage / 400.0) * 80.0f;
            for (int i = 0; i < 16; ++i) {
                if (std::isnan(loop.temperatures[i])) continue;
                // 添加一点随机扰动
                loop.temperatures[i] = base_temp - (float)(rand() % 200 / 100.0);
            }

            // 过流保护
            if (loop.current > loop.max_current_setting) {
                loop.over_current_alarm = true;
                loop.breaker_closed = false; // 跳闸
                loop.current = 0;
            }
        } else {
            loop.current = 0;
            // 冷却回常温
            for (int i = 0; i < 16; ++i) {
                if (!std::isnan(loop.temperatures[i])) {
                    loop.temperatures[i] = loop.temperatures[i] * 0.95f + 20.0f * 0.05f;
                }
            }
        }
    };

    update_loop(circ.test_loop);
    update_loop(circ.ref_loop);

    // 总电流为两支路之和
    reg.current = (circ.test_loop.current + circ.ref_loop.current);
}

// --- 命令处理 ---

void MockDevice::set_regulator_breaker(uint8_t id, bool close) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto& reg = (id == 1) ? reg1 : reg2;
    auto& circ = (id == 1) ? circ1 : circ2;

    reg.breaker_closed = close;
    if (!close) {
        // 主闸断开，回路也断电
        circ.test_loop.breaker_closed = false;
        circ.ref_loop.breaker_closed = false;
        reg.direction = 0;
        reg.current = 0;
    }
}

void MockDevice::set_loop_breaker(uint8_t circ_id, uint8_t command) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto& reg = (circ_id == 1) ? reg1 : reg2;
    auto& circ = (circ_id == 1) ? circ1 : circ2;

    if (!reg.breaker_closed) return; // 主闸没合，支路不能合

    if (command == 1)      circ.test_loop.breaker_closed = true;
    else if (command == 2) circ.test_loop.breaker_closed = false;
    else if (command == 3) circ.ref_loop.breaker_closed = true;
    else if (command == 4) circ.ref_loop.breaker_closed = false;
}

void MockDevice::set_regulator_op(uint8_t id, uint8_t cmd) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto& reg = (id == 1) ? reg1 : reg2;
    if (!reg.breaker_closed) return;
    if (cmd == 1)      reg.direction = 1;
    else if (cmd == 2) reg.direction = -1;
    else               reg.direction = 0;
}

void MockDevice::clear_alarms() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    reg1.over_voltage_alarm = reg1.over_current_alarm = false;
    reg2.over_voltage_alarm = reg2.over_current_alarm = false;
    circ1.test_loop.over_current_alarm = circ1.ref_loop.over_current_alarm = false;
    circ2.test_loop.over_current_alarm = circ2.ref_loop.over_current_alarm = false;
}

void MockDevice::set_plc_mode(uint8_t circ_id, uint8_t mode) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto& circ = (circ_id == 1) ? circ1 : circ2;
    circ.plc_mode = mode;
}

void MockDevice::set_plc_source(uint8_t circ_id, uint8_t source) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto& circ = (circ_id == 1) ? circ1 : circ2;
    circ.plc_source = source;
}

// --- 设置更新 ---

void MockDevice::update_reg_settings(uint8_t id, const RegulatorState& new_settings) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto& target = (id == 1) ? reg1 : reg2;

    target.over_voltage_limit = new_settings.over_voltage_limit;
    target.over_current_limit = new_settings.over_current_limit;
    target.speed_up_percent = std::clamp(new_settings.speed_up_percent, 1, 100);
    target.speed_down_percent = std::clamp(new_settings.speed_down_percent, 1, 100);
    target.ovp_enabled = new_settings.ovp_enabled;
}

void MockDevice::update_circ_settings(uint8_t id, const LoopState& test_s, const LoopState& ref_s) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto& circ = (id == 1) ? circ1 : circ2;

    // 更新试验回路设置
    circ.test_loop.max_current_setting = test_s.max_current_setting;
    circ.test_loop.start_current_setting = test_s.start_current_setting;
    circ.test_loop.current_change_range = test_s.current_change_range;
    circ.test_loop.ct_ratio = test_s.ct_ratio;

    // 更新参考回路设置
    circ.ref_loop.max_current_setting = ref_s.max_current_setting;
    circ.ref_loop.start_current_setting = ref_s.start_current_setting;
    circ.ref_loop.current_change_range = ref_s.current_change_range;
    circ.ref_loop.ct_ratio = ref_s.ct_ratio;
}

// --- 数据获取 ---

MockDevice::RegulatorState MockDevice::get_reg(uint8_t id) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return (id == 1) ? reg1 : reg2;
}

MockDevice::CircuitState MockDevice::get_circ(uint8_t id) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return (id == 1) ? circ1 : circ2;
}
