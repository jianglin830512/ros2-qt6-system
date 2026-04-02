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

    // [修改] 定义一个通用的初始化 Lambda，增加 max_curr 参数
    auto init_loop =[](LoopState& l, int valid_count, int32_t max_curr) {
        l.max_current_setting = max_curr;
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

    // 回路 1：TEST 和 REF 分别初始化为最大 7200A
    init_loop(circ1.test_loop, 16, 7200);
    init_loop(circ1.ref_loop, 8, 7200);

    // 回路 2：TEST 和 REF 分别初始化为最大 3600A
    init_loop(circ2.test_loop, 16, 3600);
    init_loop(circ2.ref_loop, 8, 3600);
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
        }
        std::this_thread::sleep_until(start_time + std::chrono::milliseconds(100));
    }
}

void MockDevice::update_regulator_loops(RegulatorState& reg, LoopState& loop1, LoopState& loop2) {
    // 1. 电压变化逻辑 (区分升/降压速度)
    if (reg.breaker_closed) {
        double max_step = 4.5; // [修改] 假设100ms内最大变化4.5V (45V/s) 适配 450V 满量程

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

    // 2. 物理限位 [修改为 450V]
    reg.upper_limit_on = (reg.voltage >= 450.0);
    reg.lower_limit_on = (reg.voltage <= 0.0);

    if (reg.voltage > 450.0) { reg.voltage = 450.0; if(reg.direction == 1) reg.direction = 0; }
    if (reg.voltage < 0.0)   { reg.voltage = 0.0;   if(reg.direction == -1) reg.direction = 0; }

    // 3. 过压保护 (仅当启用时)
    if (reg.ovp_enabled && reg.voltage > reg.over_voltage_limit) {
        reg.over_voltage_alarm = true;
        reg.direction = 0; // 停止
    }

    // 4. 回路耦合与过流保护
    auto update_loop = [&](LoopState& loop) {
        if (loop.breaker_closed && reg.breaker_closed) {
            // [修改] 根据调节器 ID 动态设置满载电压对应的电流
            double max_loop_current = (reg.id == 1) ? 7200.0 : 3600.0;

            // [修改] 电流与电压成正比 (按 450V 量程比例计算)
            loop.current = (reg.voltage / 450.0) * max_loop_current;

            // 温度跟随电流变化
            float base_temp = 20.0f + (float)(reg.voltage / 450.0) * 80.0f;
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

    update_loop(loop1);
    update_loop(loop2);

    // 5. [修改] 总电流 = （TEST_LOOP + REF_LOOP）电流 / 32
    // Regulator 1 (7200 + 7200) / 32 -> 最大 450A
    // Regulator 2 (3600 + 3600) / 32 -> 最大 225A
    reg.current = (loop1.current + loop2.current) / 32.0;
}

// --- 命令处理 ---

void MockDevice::set_regulator_breaker(uint8_t id, bool close) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto& reg = (id == 1) ? reg1 : reg2;

    reg.breaker_closed = close;
    if (!close) {
        reg.direction = 0;
        reg.current = 0;
        // 按照拓扑：Reg1 断开 Test回路；Reg2 断开 Ref回路
        if (id == 1) {
            circ1.test_loop.breaker_closed = false;
            circ2.test_loop.breaker_closed = false;
        } else {
            circ1.ref_loop.breaker_closed = false;
            circ2.ref_loop.breaker_closed = false;
        }
    }
}

void MockDevice::set_loop_breaker(uint8_t circ_id, uint8_t command) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto& circ = (circ_id == 1) ? circ1 : circ2;

    // 命令 1,2 操作 Test Loop，需检查 Reg1；命令 3,4 操作 Ref Loop，需检查 Reg2
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
