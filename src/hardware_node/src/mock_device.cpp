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

    // 初始化温度
    for (int i = 0; i < 16; ++i) {
        circ1.test_loop.temperatures[i] = 20.0f;
        circ1.ref_loop.temperatures[i] = 20.0f;

        if (i < 8) {
            circ2.test_loop.temperatures[i] = 20.0f;
            circ2.ref_loop.temperatures[i] = 20.0f;
        } else {
            circ2.test_loop.temperatures[i] = std::nanf("");
            circ2.ref_loop.temperatures[i] = std::nanf("");
        }
    }
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
    // 1. 电压变化逻辑
    if (reg.breaker_closed) {
        if (reg.direction != 0) {
            double step = (40.0 * (reg.speed_percent / 100.0)) * 0.1;
            reg.voltage += reg.direction * step;
        }
    }

    // 2. 物理限位
    reg.upper_limit_on = (reg.voltage >= 400.0);
    reg.lower_limit_on = (reg.voltage <= 0.0);
    if (reg.upper_limit_on) { reg.voltage = 400.0; reg.direction = 0; }
    if (reg.lower_limit_on)   { reg.voltage = 0.0;   reg.direction = 0; }

    // 3. 过压保护
    if (reg.ovp_enabled && reg.voltage > reg.over_voltage_limit) {
        reg.over_voltage_alarm = true;
        reg.direction = 0;
    }

    // 4. 回路耦合
    auto update_loop = [&](LoopState& loop) {
        if (loop.breaker_closed && reg.breaker_closed) {
            loop.current = (reg.voltage / 400.0) * 3200.0;
            float base_temp = 20.0f + (float)(reg.voltage / 400.0) * 80.0f;
            for (int i = 0; i < 16; ++i) {
                if (std::isnan(loop.temperatures[i])) continue;
                loop.temperatures[i] = base_temp - (float)(rand() % 200 / 100.0);
            }
            if (loop.current > loop.max_current_setting) {
                loop.over_current_alarm = true;
                loop.breaker_closed = false;
                loop.current = 0;
            }
        } else {
            loop.current = 0;
            for (int i = 0; i < 16; ++i) {
                if (!std::isnan(loop.temperatures[i])) loop.temperatures[i] = 20.0f;
            }
        }
    };

    update_loop(circ.test_loop);
    update_loop(circ.ref_loop);

    reg.current = (circ.test_loop.current + circ.ref_loop.current) / 1000.0;
}

// --- 命令处理 ---

void MockDevice::set_regulator_breaker(uint8_t id, bool close) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto& reg = (id == 1) ? reg1 : reg2;
    auto& circ = (id == 1) ? circ1 : circ2;

    reg.breaker_closed = close;
    if (!close) {
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

    if (!reg.breaker_closed) return;

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

// [修复] 补全缺失的函数实现
void MockDevice::set_plc_mode(uint8_t circ_id, uint8_t mode) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto& circ = (circ_id == 1) ? circ1 : circ2;
    circ.plc_mode = mode;
}

// [修复] 补全缺失的函数实现
void MockDevice::set_plc_source(uint8_t circ_id, uint8_t source) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto& circ = (circ_id == 1) ? circ1 : circ2;
    circ.plc_source = source;
}

// --- 设置更新 ---

void MockDevice::update_reg_settings(uint8_t id, const RegulatorState& settings) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto& target = (id == 1) ? reg1 : reg2;
    target.over_voltage_limit = settings.over_voltage_limit;
    target.speed_percent = std::clamp(settings.speed_percent, 20, 100);
    target.ovp_enabled = settings.ovp_enabled;
}

void MockDevice::update_circ_settings(uint8_t id, int32_t test_max, int32_t ref_max) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto& target = (id == 1) ? circ1 : circ2;
    target.test_loop.max_current_setting = test_max;
    target.ref_loop.max_current_setting = ref_max;
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
