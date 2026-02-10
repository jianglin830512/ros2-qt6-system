#include "control_node/control_logic.hpp"
#include "control_node/state_manager.hpp"
// --- 包含接口头文件 ---
#include "control_node/i_hardware_coordinator.hpp"
#include "control_node/i_persistence_coordinator.hpp"
// 包含具体策略
#include "control_node/strategies/manual_strategy.hpp"
#include "control_node/strategies/auto_current_strategy.hpp"
// 包含消息定义
#include "ros2_interfaces/msg/hardware_circuit_status.hpp"
#include "ros2_interfaces/msg/system_status.hpp"
#include "rclcpp/rclcpp.hpp"

// 构造函数：通过依赖注入接收所有依赖项
ControlLogic::ControlLogic(
    std::shared_ptr<StateManager> state_manager,
    std::shared_ptr<IHardwareCoordinator> hardware_coordinator,
    std::shared_ptr<IPersistenceCoordinator> persistence_coordinator)
    : state_manager_(state_manager),
    hardware_coordinator_(hardware_coordinator),
    persistence_coordinator_(persistence_coordinator)
{
    // 1. 初始化内存中的默认值 (防止空数据导致未初始化读取)
    initialize_all_default_settings();

    // 2. 初始化所有回路的默认策略 (手动模式)
    for (uint8_t i = 1; i <= StateManager::NUM_CIRCUITS; ++i) {
        switch_mode(i, ros2_interfaces::msg::HardwareCircuitStatus::PLC_MODE_MANUAL);
    }
}

// ---------------------------------------------------------
// 核心循环与生命周期管理
// ---------------------------------------------------------
// [核心] 50Hz 控制循环
void ControlLogic::update()
{
    // [NEW] 处理所有回路的通用业务逻辑（计数、定时、自动启停）
    // 只要系统不是 ERROR 状态，就进行计算
    if (current_lifecycle_state_ != ros2_interfaces::msg::SystemStatus::STATE_ERROR) {
        for (uint8_t i = 1; i <= StateManager::NUM_CIRCUITS; ++i) {
            // 增加 try-catch 防止业务逻辑计算异常导致节点崩溃
            try {
                process_circuit_logic(i);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("ControlLogic"),
                                      "Exception in process_circuit_logic for circuit %d: %s", i, e.what());
            } catch (...) {
                RCLCPP_ERROR(rclcpp::get_logger("ControlLogic"),
                                      "Unknown exception in process_circuit_logic for circuit %d", i);
            }
        }
    }

    // 根据当前生命周期状态，决定执行逻辑
    if (current_lifecycle_state_ == ros2_interfaces::msg::SystemStatus::STATE_NORMAL) {
        // --- Normal Operation ---

        for (auto const& [id, strategy] : active_strategies_) {
            if (strategy) {
                // [NEW] Feed raw PLC status to strategy
                auto plc_status = state_manager_->get_last_known_plc_status(id);
                strategy->update_plc_status(plc_status.first, plc_status.second);

                // Run strategy logic
                strategy->update();
            }
        }
    }
    else if (current_lifecycle_state_ == ros2_interfaces::msg::SystemStatus::STATE_ERROR) {
        // --- 错误模式 ---
        // 执行安全停机逻辑 (Failsafe)
        execute_safety_shutdown();
    }
    else {
        // --- 初始化模式 (STATE_INITIALIZING) ---
        // 静默状态：不更新策略，不发送指令，等待连接建立
    }
}

// [NEW] 处理单个回路的整体业务逻辑
void ControlLogic::process_circuit_logic(uint8_t circuit_id)
{
    // 使用 System Time 获取当前时间
    rclcpp::Time now = rclcpp::Clock(RCL_SYSTEM_TIME).now();

    // 获取数据副本
    auto settings = state_manager_->get_circuit_settings(circuit_id);
    auto status = state_manager_->get_circuit_status(circuit_id);

    bool settings_changed = false;

    // 处理 Test Loop
    if (process_single_loop_logic(settings.test_loop, status.test_loop, now)) {
        settings_changed = true;
    }

    // 处理 Ref Loop
    if (process_single_loop_logic(settings.ref_loop, status.ref_loop, now)) {
        settings_changed = true;
    }

    // 更新状态
    state_manager_->update_circuit_status(circuit_id, status);

    // 如果逻辑强制修改了设置 (如 enabled 变为 false)，需要回写并可能需要通知
    if (settings_changed) {
        state_manager_->update_circuit_settings(circuit_id, settings);
        // 注意：这里更新了 StateManager，ControlNode 的 Settings Timer 会广播新值。
    }
}

// [NEW] 单个 Loop 的具体计算逻辑
bool ControlLogic::process_single_loop_logic(
    ros2_interfaces::msg::LoopSettings& settings,
    ros2_interfaces::msg::LoopStatus& status,
    const rclcpp::Time& now)
{
    bool settings_modified = false;

    // 1. 获取开始时间 (显式指定 RCL_SYSTEM_TIME)
    rclcpp::Time start_time(settings.start_date, RCL_SYSTEM_TIME);

    // 2. 计算距离开始时间流逝的秒数
    rclcpp::Duration diff = now - start_time;
    double seconds_elapsed = diff.seconds();

    // 重置计算状态
    status.is_heat = false;
    status.elapsed_heating_time = rclcpp::Duration(0, 0);
    status.remaining_heating_time = rclcpp::Duration(0, 0);

    // --- 计算周期数 ---
    // 假设 start_time 是第0天的00:00:00
    int32_t days_passed = 0;

    if (seconds_elapsed < 0) {
        // [规则1] 在试验开启日期之前
        days_passed = -1;
        status.completed_cycle_count = 0;
        status.remaining_cycle_count = (settings.cycle_count > 0) ? settings.cycle_count : 0;

        // 强制 Disable
        if (settings.enabled) {
            settings.enabled = false;
            settings_modified = true;
        }
    } else {
        // 在开始日期之后
        days_passed = static_cast<int32_t>(seconds_elapsed / 86400.0);
        status.completed_cycle_count = static_cast<uint16_t>(days_passed);

        int32_t remaining = settings.cycle_count - status.completed_cycle_count;
        if (remaining < 0) remaining = 0;
        status.remaining_cycle_count = static_cast<uint16_t>(remaining);

        // [规则1] 超过了结束日期 (剩余次数为0且当天时间已过完，这里简化为剩余次数为0即结束)
        if (status.remaining_cycle_count == 0) {
            if (settings.enabled) {
                settings.enabled = false;
                settings_modified = true;
            }
        }
        // [规则1]如果在起始日期和结束日期之间，那么ENABLE可以有QT_NODE设置，
        // 这里不做任何操作，保留 settings.enabled 原值。
    }

    // [规则2] 如果LOOP 的ENABLE被设置为FALSE，必须把is_heat 设置为 FALSE
    // 这一步非常关键，它切断了后续加热逻辑
    if (!settings.enabled) {
        status.is_heat = false;
        return settings_modified; // 直接返回，不再计算加热窗口
    }

    // --- 计算加热状态 (Is Heat) ---
    // 代码执行到这里，说明 enabled == true，且在有效期内

    rclcpp::Duration heat_start_offset(settings.heating_time);
    rclcpp::Duration heat_duration(settings.heating_duration);

    auto check_window = [&](int32_t cycle_idx) -> bool {
        if (cycle_idx < 0) return false;
        if (cycle_idx >= settings.cycle_count) return false;

        rclcpp::Duration day_offset = rclcpp::Duration::from_seconds(cycle_idx * 86400.0);
        rclcpp::Time cycle_start = start_time + day_offset + heat_start_offset;
        rclcpp::Time cycle_end = cycle_start + heat_duration;

        if (now >= cycle_start && now < cycle_end) {
            status.is_heat = true;
            status.elapsed_heating_time = now - cycle_start;
            status.remaining_heating_time = cycle_end - now;
            return true;
        }
        return false;
    };

    // 检查今天和昨天（跨天）
    if (!check_window(days_passed)) {
        check_window(days_passed - 1);
    }

    return settings_modified;
}


// [核心] 1Hz 生命周期维护 (由 ControlNode 定时器调用)
void ControlLogic::maintain_lifecycle()
{
    auto sys_status = state_manager_->get_system_status();
    uint8_t old_state = sys_status.system_state; // 记录上一时刻状态

    // 1. 检查各节点连接状态 (非阻塞检查)
    bool hw_ok = hardware_coordinator_->is_connected();
    bool db_ok = persistence_coordinator_->is_connected();

    sys_status.hardware_connected = hw_ok;
    sys_status.database_connected = db_ok;

    // 数据同步逻辑独立出来，只依赖 db_ok
    if (db_ok && !settings_synced_) {
        attempt_sync_settings();
    }

    // 2. 状态机逻辑
    if (hw_ok && db_ok) {
        // 连接正常，检查数据是否同步
        if (settings_synced_) {
            // 一切就绪 -> 正常状态
            sys_status.system_state = ros2_interfaces::msg::SystemStatus::STATE_NORMAL;
        } else {
            // 连接通了，但数据还没加载完 -> 初始化中
            sys_status.system_state = ros2_interfaces::msg::SystemStatus::STATE_INITIALIZING;
        }
    } else {
        // 有任何一个连接丢失
        if (old_state == ros2_interfaces::msg::SystemStatus::STATE_NORMAL) {
            // 之前是正常的，现在断了 -> 错误状态 (触发停机)
            sys_status.system_state = ros2_interfaces::msg::SystemStatus::STATE_ERROR;
        } else {
            // 还没初始化完就断了，或者刚启动 -> 初始化中
            sys_status.system_state = ros2_interfaces::msg::SystemStatus::STATE_INITIALIZING;
        }

        // 如果数据库断开，强制重置同步标志，确保恢复连接后重新拉取最新数据
        if (!db_ok) {
            settings_synced_ = false;
            // 重置细分标志位
            sys_settings_loaded_ = false;
            for(auto& v : reg_settings_loaded_) v = false;
            for(auto& v : cir_settings_loaded_) v = false;
        }
    }

    // 更新内部缓存和 StateManager
    current_lifecycle_state_ = sys_status.system_state;
    state_manager_->update_system_status(sys_status);

    // 3. 状态切换动作 (Transition Actions)

    // [Recovery] 从 非Normal -> Normal：重置所有策略(清除积分)
    if (old_state != ros2_interfaces::msg::SystemStatus::STATE_NORMAL &&
        current_lifecycle_state_ == ros2_interfaces::msg::SystemStatus::STATE_NORMAL)
    {
        RCLCPP_INFO(rclcpp::get_logger("ControlLogic"), "System recovered to NORMAL. Resetting strategies.");
        for (auto& [id, strategy] : active_strategies_) {
            if(strategy) strategy->reset();
        }
    }
    // [Failure] 从 Normal -> Error：打印报警
    else if (old_state == ros2_interfaces::msg::SystemStatus::STATE_NORMAL &&
             current_lifecycle_state_ == ros2_interfaces::msg::SystemStatus::STATE_ERROR)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ControlLogic"), "System entered ERROR state! Engaging safety shutdown.");
    }
}
// 尝试从 RecordNode 同步设置
void ControlLogic::attempt_sync_settings()
{
    // 防止在一次异步请求还没回来时发起下一次请求 (简单的防抖)
    if (is_syncing_) return;
    is_syncing_ = true;

    // 定义一个检查是否全部完成的 Lambda
    auto check_completion = [this]() {
        bool all_reg = true;
        for(uint8_t i=1; i<=StateManager::NUM_REGULATORS; ++i) if(!reg_settings_loaded_[i]) all_reg = false;

        bool all_cir = true;
        for(uint8_t i=1; i<=StateManager::NUM_CIRCUITS; ++i) if(!cir_settings_loaded_[i]) all_cir = false;

        if (sys_settings_loaded_ && all_reg && all_cir) {
            settings_synced_ = true;
            RCLCPP_INFO(rclcpp::get_logger("ControlLogic"), "All settings synchronized from RecordNode.");
        }
        // 本轮尝试结束
        is_syncing_ = false;
    };

    // 1. 请求系统设置
    if (!sys_settings_loaded_) {
        persistence_coordinator_->get_system_settings(
            [this, check_completion](bool success, const ros2_interfaces::msg::SystemSettings& settings) {
                if (success) {
                    state_manager_->update_system_settings(settings);
                    sys_settings_loaded_ = true;
                }
                check_completion();
            });
    } else {
        // 如果已经加载过，直接检查后续
    }

    // 2. 请求调压器设置
    for (uint8_t i = 1; i <= StateManager::NUM_REGULATORS; ++i) {
        if (!reg_settings_loaded_[i]) {
            persistence_coordinator_->get_regulator_settings(i,
                                                             [this, i, check_completion](bool success, const ros2_interfaces::msg::RegulatorSettings& settings) {
                                                                 if (success) {
                                                                     state_manager_->update_regulator_settings(i, settings);
                                                                     reg_settings_loaded_[i] = true;
                                                                 }
                                                                 check_completion();
                                                             });
        }
    }

    // 3. 请求回路设置
    for (uint8_t i = 1; i <= StateManager::NUM_CIRCUITS; ++i) {
        if (!cir_settings_loaded_[i]) {
            persistence_coordinator_->get_circuit_settings(i,
                                                           [this, i, check_completion](bool success, const ros2_interfaces::msg::CircuitSettings& settings) {
                                                               if (success) {
                                                                   state_manager_->update_circuit_settings(i, settings);
                                                                   cir_settings_loaded_[i] = true;
                                                               }
                                                               check_completion();
                                                           });
        }
    }
}
// 辅助函数：判断系统是否就绪
bool ControlLogic::is_system_ready() const {
    return current_lifecycle_state_ == ros2_interfaces::msg::SystemStatus::STATE_NORMAL;
}

// [NEW] 辅助函数：判断设置是否已同步
bool ControlLogic::is_settings_synced() const {
    return settings_synced_;
}

// 安全停机逻辑 (Failsafe)
void ControlLogic::execute_safety_shutdown()
{
    // 只有当硬件连接还在时，发送停机指令才有意义
    if (hardware_coordinator_->is_connected()) {
        static rclcpp::Time last_safety_cmd_time(0, 0, RCL_SYSTEM_TIME);
        auto now = rclcpp::Clock().now();

        // 限制发送频率 (10Hz)，避免在错误状态下淹没网络
        if ((now - last_safety_cmd_time).seconds() > 0.1) {
            auto stop_cmd = std::make_shared<ros2_interfaces::msg::RegulatorOperationCommand>();
            stop_cmd->command = ros2_interfaces::msg::RegulatorOperationCommand::CMD_VOLTAGE_STOP;

            for (uint8_t i = 1; i <= StateManager::NUM_REGULATORS; ++i) {
                stop_cmd->regulator_id = i;
                // 注意：绕过 Strategy，直接调用 HardwareCoordinator
                hardware_coordinator_->send_regulator_operation_command(stop_cmd);
            }
            last_safety_cmd_time = now;
        }
    }
}
// ---------------------------------------------------------
// 命令处理 (Gatekeeper Pattern)
// ---------------------------------------------------------
void ControlLogic::process_regulator_operation_command(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg)
{
    if (!is_system_ready()) {
        RCLCPP_WARN(rclcpp::get_logger("ControlLogic"), "Ignored regulator op command: System not READY.");
        return;
    }

    uint8_t circuit_id = msg->regulator_id; // Assuming 1:1 mapping
    if (active_strategies_.count(circuit_id)) {
        // [CHANGED] Call renamed method
        active_strategies_[circuit_id]->handle_regulator_operation_command(msg);
    }
}

void ControlLogic::handle_regulator_breaker_command_request(const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request>& request, LogicResultCallback callback)
{
    if (!is_system_ready()) {
        if(callback) callback(false, "System not READY");
        return;
    }

    // [CHANGED] Delegate to strategy instead of calling hardware directly
    uint8_t circuit_id = request->regulator_id; // Assuming 1:1
    if (active_strategies_.count(circuit_id)) {
        active_strategies_[circuit_id]->handle_regulator_breaker_command(request, callback);
    } else {
        if(callback) callback(false, "No active strategy for this circuit.");
    }
}

void ControlLogic::handle_circuit_breaker_command_request(const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request>& request, LogicResultCallback callback)
{
    if (!is_system_ready()) {
        if(callback) callback(false, "System not READY");
        return;
    }

    // [CHANGED] Delegate to strategy
    uint8_t circuit_id = request->circuit_id;
    if (active_strategies_.count(circuit_id)) {
        active_strategies_[circuit_id]->handle_circuit_breaker_command(request, callback);
    } else {
        if(callback) callback(false, "No active strategy for this circuit.");
    }
}

void ControlLogic::handle_circuit_mode_command_request(const std::shared_ptr<ros2_interfaces::srv::CircuitModeCommand::Request>& request, LogicResultCallback callback)
{
    // [Gatekeeper]
    if (!is_system_ready()) {
        if(callback) callback(false, "System not READY");
        return;
    }

    try {
        this->switch_mode(request->circuit_id, request->command);
        if (callback) callback(true, "Strategy switched.");
    } catch (const std::exception& e) {
        if (callback) callback(false, std::string("Failed to switch strategy: ") + e.what());
    }
}
void ControlLogic::process_clear_alarm()
{
    // 允许在 ERROR 状态下清除报警，这可能是恢复手段的一部分
    hardware_coordinator_->send_clear_alarm();
}
// ---------------------------------------------------------
// 参数设置处理 (Gatekeeper + Chain)
// ---------------------------------------------------------
void ControlLogic::handle_set_system_settings_request(
    const std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Request>& request,
    LogicResultCallback callback)
{
    // [Gatekeeper]
    if (!is_system_ready()) {
        if(callback) callback(false, "System not READY. Cannot set settings.");
        return;
    }

    // 1. 直接更新到 StateManager (RecordNode 会通过广播监听)
    state_manager_->update_system_settings(request->settings);

    // 2. 回调成功 (不再调用 save service)
    if (callback) {
        callback(true, "System settings updated locally.");
    }
}
void ControlLogic::handle_set_regulator_settings_request(
    const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request>& request,
    LogicResultCallback callback)
{
    // [Gatekeeper]
    if (!is_system_ready()) {
        if(callback) callback(false, "System not READY.");
        return;
    }

    uint8_t id = request->settings.regulator_id;
    const auto& settings = request->settings;

    // 1. STATE_MANAGER 不做任何更新 (等待 Hardware 广播)

    // 2. 调用 Hardware Service
    hardware_coordinator_->apply_regulator_settings_to_hardware(
        id, settings,
        [callback](bool hw_success, const std::string& hw_message) {
            // 3. 仅回调结果
            if (callback) {
                callback(hw_success, hw_message);
            }
        });
}
void ControlLogic::handle_set_circuit_settings_request(
    const std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Request>& request,
    LogicResultCallback callback)
{
    // [Gatekeeper]
    if (!is_system_ready()) {
        if(callback) callback(false, "System not READY.");
        return;
    }

    uint8_t id = request->settings.circuit_id;
    const auto& request_settings = request->settings;

    // 1. 更新 StateManager 中的非Hardware部分
    //    逻辑：读取当前 -> 覆盖非HW字段 -> 恢复旧HW字段 -> 写入
    auto current_settings = state_manager_->get_circuit_settings(id);
    auto temp_new_settings = request_settings;

    // 保持 HardwareLoopSettings 不变 (使用旧值覆盖请求中的值)
    temp_new_settings.test_loop.hardware_loop_settings = current_settings.test_loop.hardware_loop_settings;
    temp_new_settings.ref_loop.hardware_loop_settings = current_settings.ref_loop.hardware_loop_settings;

    state_manager_->update_circuit_settings(id, temp_new_settings);


    // 2. 准备发送给 Hardware 的 HardwareCircuitSettings (使用请求中的值)
    ros2_interfaces::msg::HardwareCircuitSettings hw_settings_to_send;
    hw_settings_to_send.circuit_id = id;
    hw_settings_to_send.test_loop = request_settings.test_loop.hardware_loop_settings;
    hw_settings_to_send.ref_loop = request_settings.ref_loop.hardware_loop_settings;

    // 3. 调用 Hardware Service
    hardware_coordinator_->apply_circuit_settings_to_hardware(
        id, hw_settings_to_send,
        [callback](bool hw_success, const std::string& hw_message) {
            // 4. 仅回调结果 (HW 成功后不更新 StateManager, 等待 HW 广播)
            if (callback) {
                callback(hw_success, hw_message);
            }
        });
}
// ---------------------------------------------------------
// 辅助函数与默认设置
// ---------------------------------------------------------
void ControlLogic::switch_mode(uint8_t circuit_id, uint8_t new_mode)
{
    // 清除旧策略
    active_strategies_.erase(circuit_id);

    // 根据消息常量进行切换
    switch (new_mode) {
    case ros2_interfaces::msg::HardwareCircuitStatus::PLC_MODE_MANUAL:
        active_strategies_[circuit_id] = std::make_unique<ManualStrategy>(
            circuit_id, state_manager_, hardware_coordinator_);
        break;
    case ros2_interfaces::msg::HardwareCircuitStatus::PLC_MODE_AUTO_CURRENT:
        active_strategies_[circuit_id] = std::make_unique<AutoCurrentStrategy>(
            circuit_id, state_manager_, hardware_coordinator_);
        break;
    default:
        // 默认回落到手动模式，并打印警告
        RCLCPP_WARN(rclcpp::get_logger("ControlLogic"), "Unknown mode %d requested for circuit %d. Defaulting to Manual.", new_mode, circuit_id);
        active_strategies_[circuit_id] = std::make_unique<ManualStrategy>(
            circuit_id, state_manager_, hardware_coordinator_);
        break;
    }

    // 更新 StateManager 中的控制模式字段
    auto status = state_manager_->get_circuit_status(circuit_id);
    status.control_mode = new_mode;
    state_manager_->update_circuit_status(circuit_id, status);

    RCLCPP_INFO(rclcpp::get_logger("ControlLogic"), "Switched Circuit %u to Mode %u. Strategy activated.", circuit_id, new_mode);
}
// --- 系统默认设置 ---
ros2_interfaces::msg::SystemSettings ControlLogic::create_default_system_settings() {
    ros2_interfaces::msg::SystemSettings settings;
    settings.sample_interval_sec = 1; // 1秒采样一次
    settings.record_interval_min = 5; // 5分钟存一次盘
    settings.keep_record_on_shutdown = true;
    return settings;
}
// --- 调压器默认设置 ---
ros2_interfaces::msg::RegulatorSettings ControlLogic::create_default_regulator_settings(uint8_t id) {
    ros2_interfaces::msg::RegulatorSettings settings;
    settings.regulator_id = id;
    settings.over_voltage_v = 450; // 安全电压阈值
    settings.over_current_a = 100; // 安全电流阈值
    settings.voltage_up_speed_percent = 10;
    settings.voltage_down_speed_percent = 20;
    return settings;
}
// --- 回路默认设置 ---
ros2_interfaces::msg::CircuitSettings ControlLogic::create_default_circuit_settings(uint8_t id) {
    ros2_interfaces::msg::CircuitSettings settings;
    settings.circuit_id = id;
    settings.test_loop.enabled = false; // 默认不启动
    settings.ref_loop.enabled = false;
    settings.curr_mode_use_ref = false;
    // 初始化内部的硬件特定参数（防止为空导致的溢出或异常）
    settings.test_loop.hardware_loop_settings.ct_ratio = 1000;
    settings.test_loop.hardware_loop_settings.start_current_a = 0;
    settings.ref_loop.hardware_loop_settings.ct_ratio = 1000;
    settings.ref_loop.hardware_loop_settings.start_current_a = 0;

    return settings;
}
// --- 汇总初始化方法 ---
void ControlLogic::initialize_all_default_settings() {
    // 1. 初始化 Settings (原有代码)
    state_manager_->update_system_settings(create_default_system_settings());
    for (uint8_t i = 1; i <= StateManager::NUM_REGULATORS; ++i) {
        state_manager_->update_regulator_settings(i, create_default_regulator_settings(i));
    }
    for (uint8_t i = 1; i <= StateManager::NUM_CIRCUITS; ++i) {
        state_manager_->update_circuit_settings(i, create_default_circuit_settings(i));
    }
    // 2. 初始化 Status 的 ID
    // 即使没有硬件数据，ID 也应该是正确的，方便 QT 识别
    for (uint8_t i = 1; i <= StateManager::NUM_REGULATORS; ++i) {
        ros2_interfaces::msg::RegulatorStatus default_status;
        default_status.regulator_id = i;
        state_manager_->update_regulator_status(i, default_status);
    }

    for (uint8_t i = 1; i <= StateManager::NUM_CIRCUITS; ++i) {
        ros2_interfaces::msg::CircuitStatus default_status;
        default_status.circuit_id = i;
        state_manager_->update_circuit_status(i, default_status);
    }
}
