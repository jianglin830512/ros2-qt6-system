// ============================================================
// FILE: E:\WGS\SRC\CONTROL_NODE\src\control_logic.cpp
// ============================================================
#include "control_node/control_logic.hpp"
#include "control_node/state_manager.hpp"
#include "control_node/i_hardware_coordinator.hpp"
#include "control_node/i_persistence_coordinator.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>

ControlLogic::ControlLogic(
    std::shared_ptr<StateManager> state_manager,
    std::shared_ptr<IHardwareCoordinator> hardware_coordinator,
    std::shared_ptr<IPersistenceCoordinator> persistence_coordinator)
    : state_manager_(state_manager),
    hardware_coordinator_(hardware_coordinator),
    persistence_coordinator_(persistence_coordinator),
    last_plc_cmd_time_(0,0,RCL_SYSTEM_TIME)
{
    initialize_all_default_settings();
}

void ControlLogic::update()
{
    // 如果还没加载完参数，不执行自动/手动控制逻辑
    if (!settings_synced_) return;

    // 1. 无条件处理时间逻辑计算 is_heat
    process_time_calculation();

    // 2. 根据系统配置执行 手动/自动 逻辑
    auto sys_settings = state_manager_->get_system_settings();
    if (sys_settings.auto_on) {
        process_auto_logic();
    } else {
        process_manual_interlock();
    }
}

// ---------------------------------------------------------
// 生命周期与同步状态判定
// ---------------------------------------------------------
void ControlLogic::maintain_lifecycle() {
    auto sys_status = state_manager_->get_system_status();

    // 获取 coordinator 中自带的 3秒超时 判定结果
    bool hw_ok = hardware_coordinator_->is_connected();
    bool db_ok = persistence_coordinator_->is_connected();

    // 获取硬件中的子节点连接状态
    bool plc_ok = sys_status.hardware_system_status.plc_connected;
    bool temp_ok = sys_status.hardware_system_status.temp_monitor_connected;

    // 将连通性状态写入准备广播的数据中
    sys_status.hardware_connected = hw_ok;
    sys_status.database_connected = db_ok;

    // 1. 如果 DB 在线，且还没同步过，发起拉取
    if (db_ok && !settings_synced_) {
        attempt_sync_settings();
    }

    // 2. 检查异步回调是否已经全部获取成功
    if (!settings_synced_ && sys_settings_loaded_ &&
        reg_settings_loaded_[1] && reg_settings_loaded_[2] &&
        cir_settings_loaded_[1] && cir_settings_loaded_[2]) {

        settings_synced_ = true;
        is_syncing_ = false;
        RCLCPP_INFO(rclcpp::get_logger("ControlLogic"), "Initial settings sync from RECORD_NODE completed successfully.");
    }

    // 3. 更新系统综合状态标记 (System State)
    if (!hw_ok || !db_ok || !plc_ok || !temp_ok) {
        sys_status.system_state = ros2_interfaces::msg::SystemStatus::STATE_ERROR;

        // 增加诊断日志：每 3 秒打印一次到底是谁掉线了，避免盲猜
        static rclcpp::Time last_print_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        auto now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        if ((now - last_print_time).seconds() > 3.0) {
            RCLCPP_WARN(rclcpp::get_logger("ControlLogic"),
                        "System is in ERROR/STANDBY state! Diagnostics -> DB Node: %d, HW Node: %d, PLC: %d, Temp Monitor: %d",
                        db_ok, hw_ok, plc_ok, temp_ok);
            last_print_time = now;
        }
    }
    else if (settings_synced_) {
        sys_status.system_state = ros2_interfaces::msg::SystemStatus::STATE_NORMAL;
    }
    else {
        sys_status.system_state = ros2_interfaces::msg::SystemStatus::STATE_INITIALIZING;
    }

    current_lifecycle_state_ = sys_status.system_state;
    state_manager_->update_system_status(sys_status);
}

void ControlLogic::attempt_sync_settings() {
    if (is_syncing_ || settings_synced_) return;
    is_syncing_ = true;

    persistence_coordinator_->get_system_settings([this](bool success, const ros2_interfaces::msg::SystemSettings& s) {
        if(success) { state_manager_->update_system_settings(s); sys_settings_loaded_ = true; }
    });
    for (uint8_t i = 1; i <= 2; ++i) {
        persistence_coordinator_->get_regulator_settings(i, [this, i](bool success, const ros2_interfaces::msg::RegulatorSettings& s) {
            if(success) { state_manager_->update_regulator_settings(i, s); reg_settings_loaded_[i] = true; }
        });
        persistence_coordinator_->get_circuit_settings(i, [this, i](bool success, const ros2_interfaces::msg::CircuitSettings& s) {
            if(success) { state_manager_->update_circuit_settings(i, s); cir_settings_loaded_[i] = true; }
        });
    }
}

bool ControlLogic::is_system_ready() const { return settings_synced_; }
bool ControlLogic::is_settings_synced() const { return settings_synced_; }

// ---------------------------------------------------------
// 核心逻辑保持不变
// ---------------------------------------------------------
bool ControlLogic::check_time_overlap(const ros2_interfaces::msg::CircuitSettings& c1, const ros2_interfaces::msg::CircuitSettings& c2) {
    auto get_intervals = [](const ros2_interfaces::msg::CircuitSettings& circ) {
        std::vector<std::pair<int32_t, int32_t>> ivs;
        auto add_loop = [&ivs](const ros2_interfaces::msg::LoopSettings& l) {
            if (!l.enabled) return;
            int32_t start = l.heating_time.sec % 86400;
            int32_t dur = l.heating_duration.sec;
            if (dur == 0) return;
            int32_t end = start + dur;
            if (end > 86400) { ivs.push_back({start, 86400}); ivs.push_back({0, end % 86400}); }
            else { ivs.push_back({start, end}); }
        };
        add_loop(circ.test_loop); add_loop(circ.ref_loop); return ivs;
    };
    auto c1_intervals = get_intervals(c1); auto c2_intervals = get_intervals(c2);
    for (auto& iv : c1_intervals) {
        int32_t exp_start = iv.first - 300; int32_t exp_end = iv.second + 300;
        for (const auto& c2_iv : c2_intervals) {
            int32_t c2_starts[3] = {c2_iv.first - 86400, c2_iv.first, c2_iv.first + 86400};
            int32_t c2_ends[3]   = {c2_iv.second - 86400, c2_iv.second, c2_iv.second + 86400};
            for (int i=0; i<3; ++i) {
                if (!(exp_end <= c2_starts[i] || exp_start >= c2_ends[i])) return true;
            }
        }
    }
    return false;
}

void ControlLogic::process_time_calculation() {
    rclcpp::Time now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    auto calc_loop = [&](ros2_interfaces::msg::LoopSettings& settings, ros2_interfaces::msg::LoopStatus& status) {
        status.is_heat = false; if (!settings.enabled) return false;
        rclcpp::Time start_time(settings.start_date, RCL_SYSTEM_TIME);
        double seconds_elapsed = (now - start_time).seconds();
        if (seconds_elapsed < 0) return false;
        int32_t days_passed = static_cast<int32_t>(seconds_elapsed / 86400.0);
        status.completed_cycle_count = static_cast<uint16_t>(days_passed);
        status.remaining_cycle_count = (settings.cycle_count > status.completed_cycle_count) ? (settings.cycle_count - status.completed_cycle_count) : 0;
        if (status.remaining_cycle_count == 0) { settings.enabled = false; return true; }
        auto check_window = [&](int32_t day_idx) {
            if (day_idx < 0 || day_idx >= settings.cycle_count) return false;
            rclcpp::Time window_start = start_time + rclcpp::Duration::from_seconds(day_idx * 86400.0) + rclcpp::Duration(settings.heating_time);
            rclcpp::Time window_end = window_start + rclcpp::Duration(settings.heating_duration);
            if (now >= window_start && now < window_end) {
                status.is_heat = true; status.elapsed_heating_time = now - window_start; status.remaining_heating_time = window_end - now; return true;
            }
            return false;
        };
        if (!check_window(days_passed)) check_window(days_passed - 1);
        return false;
    };
    for (uint8_t id = 1; id <= 2; ++id) {
        auto settings = state_manager_->get_circuit_settings(id);
        auto status = state_manager_->get_circuit_status(id);
        bool chg_test = calc_loop(settings.test_loop, status.test_loop);
        bool chg_ref = calc_loop(settings.ref_loop, status.ref_loop);
        state_manager_->update_circuit_status(id, status);
        if (chg_test || chg_ref) state_manager_->update_circuit_settings(id, settings);
    }
}

void ControlLogic::process_manual_interlock() {
    auto sys_status = state_manager_->get_system_status();
    auto c1 = state_manager_->get_circuit_status(1);
    auto c2 = state_manager_->get_circuit_status(2);

    bool c1_working = c1.test_loop.hardware_loop_status.breaker_closed_switch_ack || c1.ref_loop.hardware_loop_status.breaker_closed_switch_ack;
    bool c2_working = c2.test_loop.hardware_loop_status.breaker_closed_switch_ack || c2.ref_loop.hardware_loop_status.breaker_closed_switch_ack;
    if (c1_working) sys_status.circuit_work_status = ros2_interfaces::msg::SystemStatus::CIRCUIT_1_WORKING;
    else if (c2_working) sys_status.circuit_work_status = ros2_interfaces::msg::SystemStatus::CIRCUIT_2_WORKING;
    else sys_status.circuit_work_status = ros2_interfaces::msg::SystemStatus::NONE_CIRCUIT_WORKING;
    state_manager_->update_system_status(sys_status);

    auto now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    if ((now - last_plc_cmd_time_).seconds() > 1.0) {
        auto check_and_set_mode = [this](uint8_t cid, uint8_t ltype, uint8_t target_mode) {
            auto c_status = state_manager_->get_circuit_status(cid);
            uint8_t current_mode = (ltype == 1) ? c_status.test_loop.hardware_loop_status.plc_control_mode : c_status.ref_loop.hardware_loop_status.plc_control_mode;
            if (current_mode != target_mode) {
                hardware_coordinator_->set_circuit_control_mode(cid, ltype, target_mode);
            }
        };
        check_and_set_mode(1, 1, 1);
        check_and_set_mode(1, 2, 1);
        check_and_set_mode(2, 1, 1);
        check_and_set_mode(2, 2, 1);
        last_plc_cmd_time_ = now;
    }
}

void ControlLogic::process_auto_logic() {
    auto sys_status = state_manager_->get_system_status();
    auto c1 = state_manager_->get_circuit_status(1); auto c2 = state_manager_->get_circuit_status(2);
    auto c1_set = state_manager_->get_circuit_settings(1); auto c2_set = state_manager_->get_circuit_settings(2);

    bool c1_heat = c1.test_loop.is_heat || c1.ref_loop.is_heat;
    bool c2_heat = c2.test_loop.is_heat || c2.ref_loop.is_heat;
    if (c1_heat) sys_status.circuit_work_status = ros2_interfaces::msg::SystemStatus::CIRCUIT_1_WORKING;
    else if (c2_heat) sys_status.circuit_work_status = ros2_interfaces::msg::SystemStatus::CIRCUIT_2_WORKING;
    else sys_status.circuit_work_status = ros2_interfaces::msg::SystemStatus::NONE_CIRCUIT_WORKING;
    state_manager_->update_system_status(sys_status);

    auto now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    bool can_send_cmd = (now - last_plc_cmd_time_).seconds() > 1.0;

    if (sys_status.circuit_work_status == ros2_interfaces::msg::SystemStatus::NONE_CIRCUIT_WORKING) {
        if(can_send_cmd) execute_auto_shutdown();
    }
    else if (sys_status.circuit_work_status == ros2_interfaces::msg::SystemStatus::CIRCUIT_1_WORKING) {
        if(can_send_cmd) {
            // 回路2闲置，让它关闭，但不允许它动调压器（传 false）
            manage_loop_auto(2, 1, false, 1, 1, false);
            manage_loop_auto(2, 2, false, 2, 1, false);
            // 回路1工作，拥有调压器控制权（传 true）
            manage_loop_auto(1, 1, c1.test_loop.is_heat, 1, c1_set.test_loop.auto_strategy, true);
            manage_loop_auto(1, 2, c1.ref_loop.is_heat,  2, c1_set.ref_loop.auto_strategy, true);
        }
    }
    else if (sys_status.circuit_work_status == ros2_interfaces::msg::SystemStatus::CIRCUIT_2_WORKING) {
        if(can_send_cmd) {
            // 回路1闲置，不允许它动调压器（传 false）
            manage_loop_auto(1, 1, false, 1, 1, false);
            manage_loop_auto(1, 2, false, 2, 1, false);
            // 回路2工作，拥有调压器控制权（传 true）
            manage_loop_auto(2, 1, c2.test_loop.is_heat, 1, c2_set.test_loop.auto_strategy, true);
            manage_loop_auto(2, 2, c2.ref_loop.is_heat,  2, c2_set.ref_loop.auto_strategy, true);
        }
    }
    if (can_send_cmd) last_plc_cmd_time_ = now;
}

void ControlLogic::execute_auto_shutdown() {
    // 彻底停机时，为了避免两遍代码都给调压器发分闸指令导致冲突
    // 只让回路 1 负责分闸调压器，回路 2 仅分闸自己的支路即可
    manage_loop_auto(1, 1, false, 1, 1, true);
    manage_loop_auto(1, 2, false, 2, 1, true);
    manage_loop_auto(2, 1, false, 1, 1, false);
    manage_loop_auto(2, 2, false, 2, 1, false);
}

void ControlLogic::manage_loop_auto(uint8_t circuit_id, uint8_t loop_type, bool is_heating, uint8_t regulator_id, uint8_t target_plc_mode, bool control_regulator) {
    auto circ_status = state_manager_->get_circuit_status(circuit_id);
    auto loop_hw = (loop_type == 1) ? circ_status.test_loop.hardware_loop_status : circ_status.ref_loop.hardware_loop_status;
    auto reg_status = state_manager_->get_regulator_status(regulator_id);

    if (is_heating) {
        // 【优化】只要是加热状态，绝对不允许发送分闸指令！
        if (!loop_hw.breaker_closed_switch_ack || !reg_status.breaker_closed_switch_ack) {
            if (!is_regulator_at_lower_limit(regulator_id)) {
                // 如果不在零位，先降压到零（只有拥有控制权才允许操作）
                if (control_regulator) {
                    auto op = std::make_shared<ros2_interfaces::msg::RegulatorOperationCommand>();
                    op->regulator_id = regulator_id; op->command = 2; // Move down
                    hardware_coordinator_->send_regulator_operation_command(op);
                }
            } else {
                // 已经在零位。严格按照 PLC 硬件约束：必须先合调压器，再合支路！
                if (!reg_status.breaker_closed_switch_ack) {
                    // 1. 调压器没合，优先合调压器
                    if (control_regulator) {
                        auto cmd = std::make_shared<ros2_interfaces::srv::RegulatorBreakerCommand::Request>();
                        cmd->regulator_id = regulator_id; cmd->command = 1; // Close Regulator
                        hardware_coordinator_->execute_regulator_breaker_command(cmd, nullptr);
                    }
                }
                else if (!loop_hw.breaker_closed_switch_ack) {
                    // 2. 只有确认调压器已经合闸后，才允许发送支路合闸指令（避免 PLC 拒绝）
                    auto cmd = std::make_shared<ros2_interfaces::srv::CircuitBreakerCommand::Request>();
                    cmd->circuit_id = circuit_id; cmd->command = (loop_type == 1) ? 1 : 3; // Close Loop
                    hardware_coordinator_->execute_circuit_breaker_command(cmd, nullptr);
                }
            }
        } else {
            // 都已合闸，且在零位，切换到目标 PLC 自动模式
            if (loop_hw.plc_control_mode != target_plc_mode) {
                hardware_coordinator_->set_circuit_control_mode(circuit_id, loop_type, target_plc_mode);
            }
        }
    } else {
        // 【停止加热的逻辑】
        if (loop_hw.plc_control_mode != 1) {
            hardware_coordinator_->set_circuit_control_mode(circuit_id, loop_type, 1);
        } else {
            if (loop_hw.breaker_closed_switch_ack) {
                // 支路还合着。如果允许控调压器且没归零，先归零以防电弧
                if (control_regulator && !is_regulator_at_lower_limit(regulator_id)) {
                    auto op = std::make_shared<ros2_interfaces::msg::RegulatorOperationCommand>();
                    op->regulator_id = regulator_id; op->command = 2;
                    hardware_coordinator_->send_regulator_operation_command(op);
                } else {
                    // 已归零（或者不允许控调压器），安全分闸支路
                    auto cmd = std::make_shared<ros2_interfaces::srv::CircuitBreakerCommand::Request>();
                    cmd->circuit_id = circuit_id; cmd->command = (loop_type == 1) ? 2 : 4; // Open Loop
                    hardware_coordinator_->execute_circuit_breaker_command(cmd, nullptr);
                }
            } else {
                // 支路已经分闸了。如果拥有控制权，再去分闸调压器
                if (control_regulator && reg_status.breaker_closed_switch_ack) {
                    if (!is_regulator_at_lower_limit(regulator_id)) {
                        auto op = std::make_shared<ros2_interfaces::msg::RegulatorOperationCommand>();
                        op->regulator_id = regulator_id; op->command = 2;
                        hardware_coordinator_->send_regulator_operation_command(op);
                    } else {
                        auto cmd = std::make_shared<ros2_interfaces::srv::RegulatorBreakerCommand::Request>();
                        cmd->regulator_id = regulator_id; cmd->command = 2; // Open Regulator
                        hardware_coordinator_->execute_regulator_breaker_command(cmd, nullptr);
                    }
                }
            }
        }
    }
}

bool ControlLogic::is_regulator_at_lower_limit(uint8_t regulator_id) {
    auto status = state_manager_->get_regulator_status(regulator_id);
    return status.lower_limit_switch_on || (status.voltage_reading < 2.0);
}

void ControlLogic::process_regulator_operation_command(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg) {
    if (!is_system_ready()) return;
    if (state_manager_->get_system_settings().auto_on) return;
    hardware_coordinator_->send_regulator_operation_command(msg);
}

void ControlLogic::handle_regulator_breaker_command_request(const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request>& request, LogicResultCallback callback) {
    if (!is_system_ready()) { if(callback) callback(false, "System not READY"); return; }
    if (state_manager_->get_system_settings().auto_on) { if(callback) callback(false, "Rejected: System in AUTO mode."); return; }
    if (request->command == 1 && !is_regulator_at_lower_limit(request->regulator_id)) { if(callback) callback(false, "Rejected: Regulator not at Lower Limit."); return; }
    hardware_coordinator_->execute_regulator_breaker_command(request, callback);
}

void ControlLogic::handle_circuit_breaker_command_request(const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request>& request, LogicResultCallback callback) {
    if (!is_system_ready()) { if(callback) callback(false, "System not READY"); return; }
    if (state_manager_->get_system_settings().auto_on) { if(callback) callback(false, "Rejected: System in AUTO mode."); return; }
    auto c_set = state_manager_->get_circuit_settings(request->circuit_id);
    bool is_test = (request->command == 1 || request->command == 2);
    if ((is_test && !c_set.test_loop.enabled) || (!is_test && !c_set.ref_loop.enabled)) { if(callback) callback(false, "Rejected: Loop is NOT Enabled."); return; }
    auto sys_status = state_manager_->get_system_status();
    if (request->command == 1 || request->command == 3) {
        if (sys_status.circuit_work_status != 0 && sys_status.circuit_work_status != request->circuit_id) { if(callback) callback(false, "Rejected: Interlock active."); return; }
        uint8_t req_reg = is_test ? 1 : 2;
        if (!is_regulator_at_lower_limit(req_reg)) { if(callback) callback(false, "Rejected: Regulator not at Lower Limit."); return; }
    }
    hardware_coordinator_->execute_circuit_breaker_command(request, callback);
}

void ControlLogic::handle_set_system_settings_request(const std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Request>& request, LogicResultCallback callback) {
    state_manager_->update_system_settings(request->settings);
    if (callback) callback(true, "System settings updated.");
}

void ControlLogic::handle_set_regulator_settings_request(const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request>& request, LogicResultCallback callback) {
    hardware_coordinator_->apply_regulator_settings_to_hardware(request->settings.regulator_id, request->settings, callback);
}

void ControlLogic::handle_set_circuit_settings_request(const std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Request>& request, LogicResultCallback callback) {
    uint8_t id = request->settings.circuit_id;
    uint8_t other_id = (id == 1) ? 2 : 1;
    auto other_settings = state_manager_->get_circuit_settings(other_id);
    bool overlap = (id == 1) ? check_time_overlap(request->settings, other_settings) : check_time_overlap(other_settings, request->settings);
    if (overlap) { if (callback) callback(false, "Time Overlap Error"); return; }
    auto current_settings = state_manager_->get_circuit_settings(id);
    auto temp_new_settings = request->settings;
    temp_new_settings.test_loop.hardware_loop_settings = current_settings.test_loop.hardware_loop_settings;
    temp_new_settings.ref_loop.hardware_loop_settings = current_settings.ref_loop.hardware_loop_settings;
    state_manager_->update_circuit_settings(id, temp_new_settings);

    ros2_interfaces::msg::HardwareCircuitSettings hw_settings_to_send;
    hw_settings_to_send.circuit_id = id;
    hw_settings_to_send.test_loop = request->settings.test_loop.hardware_loop_settings;
    hw_settings_to_send.ref_loop = request->settings.ref_loop.hardware_loop_settings;
    hardware_coordinator_->apply_circuit_settings_to_hardware(id, hw_settings_to_send, callback);
}

void ControlLogic::process_clear_alarm() { hardware_coordinator_->send_clear_alarm(); }

void ControlLogic::initialize_all_default_settings() {
    state_manager_->update_system_settings(create_default_system_settings());
    for (uint8_t i = 1; i <= 2; ++i) {
        state_manager_->update_regulator_settings(i, create_default_regulator_settings(i));
        state_manager_->update_circuit_settings(i, create_default_circuit_settings(i));
        ros2_interfaces::msg::RegulatorStatus r_s; r_s.regulator_id = i; state_manager_->update_regulator_status(i, r_s);
        ros2_interfaces::msg::CircuitStatus c_s; c_s.circuit_id = i; state_manager_->update_circuit_status(i, c_s);
    }
}
ros2_interfaces::msg::SystemSettings ControlLogic::create_default_system_settings() { ros2_interfaces::msg::SystemSettings s; s.auto_on = false; return s; }
ros2_interfaces::msg::RegulatorSettings ControlLogic::create_default_regulator_settings(uint8_t id) { ros2_interfaces::msg::RegulatorSettings s; s.regulator_id = id; return s; }
ros2_interfaces::msg::CircuitSettings ControlLogic::create_default_circuit_settings(uint8_t id) { ros2_interfaces::msg::CircuitSettings s; s.circuit_id = id; return s; }
