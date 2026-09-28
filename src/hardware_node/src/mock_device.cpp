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
    reg1.last_cmd_time = std::chrono::steady_clock::now();
    reg2.last_cmd_time = std::chrono::steady_clock::now();

    auto init_loop =[](LoopState& l, int32_t max_curr) {
        l.max_current_setting = max_curr;
        l.start_current_setting = 1000;
        l.current_change_range = 5;
        l.ct_ratio = 1000;
    };

    init_loop(circ1.test_loop, 7200);
    init_loop(circ1.ref_loop, 7200);
    init_loop(circ2.test_loop, 3600);
    init_loop(circ2.ref_loop, 3600);

    for (int i = 0; i < 40; ++i) {
        circ1.temperatures[i] = 20.0f;
        circ2.temperatures[i] = 20.0f;
    }
}

void MockDevice::plc_cycle() {
    while (running_) {
        auto start_time = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            // 调压器1 控制 circ1.test_loop 和 circ2.test_loop
            update_regulator_loops(reg1, circ1.test_loop, circ2.test_loop);
            // 调压器2 控制 circ1.ref_loop 和 circ2.ref_loop
            update_regulator_loops(reg2, circ1.ref_loop, circ2.ref_loop);

            // 更新回路温度仿真 (基于回路是否正在通流加热)
            auto update_circuit_temp = [&](CircuitState& circ) {
                bool heating = (circ.test_loop.breaker_closed || circ.ref_loop.breaker_closed);
                float base_temp = 20.0f;
                if (heating) {
                    float v = std::max(reg1.voltage, reg2.voltage);
                    base_temp = 20.0f + (float)(v / 450.0) * 80.0f;
                }
                for (int i = 0; i < 40; ++i) {
                    if (std::isnan(circ.temperatures[i])) continue;
                    if (heating) {
                        circ.temperatures[i] = base_temp - (float)(rand() % 200 / 100.0);
                    } else {
                        circ.temperatures[i] = circ.temperatures[i] * 0.99f + 20.0f * 0.01f;
                    }
                }
            };
            update_circuit_temp(circ1);
            update_circuit_temp(circ2);
        }
        std::this_thread::sleep_until(start_time + std::chrono::milliseconds(20));
    }
}

void MockDevice::update_regulator_loops(RegulatorState& reg, LoopState& loop1, LoopState& loop2) {
    if (reg.direction != 0 && !reg.auto_reduce_opening) {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - reg.last_cmd_time).count() > 100) {
            reg.direction = 0;
        }
    }

    if (reg.auto_reduce_opening) {
        reg.direction = -1;
        if (reg.voltage <= 0.0) {
            reg.breaker_closed = false;
            reg.auto_reduce_opening = false;
            reg.direction = 0;

            if (reg.id == 1) {
                circ1.test_loop.breaker_closed = false;
                circ2.test_loop.breaker_closed = false;
            } else {
                circ1.ref_loop.breaker_closed = false;
                circ2.ref_loop.breaker_closed = false;
            }
        }
    }

    if (reg.breaker_closed || reg.auto_reduce_opening) {
        double max_step = 0.9;
        if (reg.direction == 1) reg.voltage += max_step * (reg.speed_up_percent / 100.0);
        else if (reg.direction == -1) reg.voltage -= max_step * (reg.speed_down_percent / 100.0);
    }

    reg.upper_limit_on = (reg.voltage >= 450.0);
    reg.lower_limit_on = (reg.voltage <= 0.0);

    if (reg.voltage > 450.0) { reg.voltage = 450.0; if(reg.direction == 1) reg.direction = 0; }
    if (reg.voltage < 0.0)   { reg.voltage = 0.0;   if(reg.direction == -1) reg.direction = 0; }

    if (reg.ovp_enabled && reg.voltage > reg.over_voltage_limit) {
        reg.over_voltage_alarm = true;
        reg.direction = 0;
    }

    auto update_loop = [&](LoopState& loop) {
        if (loop.breaker_closed && reg.breaker_closed) {
            double max_loop_current = (reg.id == 1) ? 7200.0 : 3600.0;
            loop.current = (reg.voltage / 450.0) * max_loop_current;

            if (loop.current > loop.max_current_setting) {
                loop.over_current_alarm = true;
                loop.breaker_closed = false;
                loop.current = 0;
            }
        } else {
            loop.current = 0;
        }
    };

    update_loop(loop1);
    update_loop(loop2);
    reg.current = (loop1.current + loop2.current) / 32.0;
}

void MockDevice::set_regulator_breaker(uint8_t id, uint8_t command) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto& reg = (id == 1) ? reg1 : reg2;

    if (command == 1) {
        reg.breaker_closed = true;
        reg.auto_reduce_opening = false;
    }
    else if (command == 2) {
        reg.breaker_closed = false;
        reg.auto_reduce_opening = false;
        reg.direction = 0;
        reg.current = 0;

        if (id == 1) {
            circ1.test_loop.breaker_closed = false;
            circ2.test_loop.breaker_closed = false;
        } else {
            circ1.ref_loop.breaker_closed = false;
            circ2.ref_loop.breaker_closed = false;
        }
    }
    else if (command == 3) {
        if (reg.breaker_closed) reg.auto_reduce_opening = true;
    }
}

void MockDevice::set_loop_breaker(uint8_t circ_id, uint8_t command) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto& circ = (circ_id == 1) ? circ1 : circ2;

    if (command == 1 || command == 2) {
        if (!reg1.breaker_closed) return;
        circ.test_loop.breaker_closed = (command == 1);
    }
    else if (command == 3 || command == 4) {
        if (!reg2.breaker_closed) return;
        circ.ref_loop.breaker_closed = (command == 3);
    }
}

void MockDevice::set_regulator_op(uint8_t id, uint8_t cmd) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto& reg = (id == 1) ? reg1 : reg2;
    if (!reg.breaker_closed) return;
    if (cmd == 1)      reg.direction = 1;
    else if (cmd == 2) reg.direction = -1;
    else               reg.direction = 0;
    reg.last_cmd_time = std::chrono::steady_clock::now();
}

void MockDevice::clear_alarms() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    reg1.over_voltage_alarm = reg1.over_current_alarm = false;
    reg2.over_voltage_alarm = reg2.over_current_alarm = false;
    circ1.test_loop.over_current_alarm = circ1.ref_loop.over_current_alarm = false;
    circ2.test_loop.over_current_alarm = circ2.ref_loop.over_current_alarm = false;
}

void MockDevice::set_plc_mode(uint8_t circ_id, uint8_t loop_type, uint8_t mode) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto& circ = (circ_id == 1) ? circ1 : circ2;
    if (loop_type == 1) circ.test_loop.plc_mode = mode;
    else circ.ref_loop.plc_mode = mode;
}

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
    circ.test_loop.max_current_setting = test_s.max_current_setting;
    circ.test_loop.start_current_setting = test_s.start_current_setting;
    circ.test_loop.current_change_range = test_s.current_change_range;
    circ.test_loop.ct_ratio = test_s.ct_ratio;

    circ.ref_loop.max_current_setting = ref_s.max_current_setting;
    circ.ref_loop.start_current_setting = ref_s.start_current_setting;
    circ.ref_loop.current_change_range = ref_s.current_change_range;
    circ.ref_loop.ct_ratio = ref_s.ct_ratio;
}

MockDevice::RegulatorState MockDevice::get_reg(uint8_t id) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return (id == 1) ? reg1 : reg2;
}

MockDevice::CircuitState MockDevice::get_circ(uint8_t id) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return (id == 1) ? circ1 : circ2;
}
