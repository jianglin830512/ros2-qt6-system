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

    // 注意：构造函数不进行网络IO或阻塞等待。
    // 数据同步和连接检查完全交给 maintain_lifecycle 由外部定时器驱动。
}

// ---------------------------------------------------------
//                核心循环与生命周期管理
// ---------------------------------------------------------

// [核心] 50Hz 控制循环
void ControlLogic::update()
{
    // 根据当前生命周期状态，决定执行逻辑
    if (current_lifecycle_state_ == ros2_interfaces::msg::SystemStatus::STATE_NORMAL) {
        // --- 正常运行模式 ---

        // 1. 处理调压器数据 (硬件 -> 常规状态)
        for (uint8_t id = 1; id <= StateManager::NUM_REGULATORS; ++id)
        {
            auto hw_status = state_manager_->get_hardware_regulator_status(id);
            hw_status.regulator_id = id;  // 防止硬件发的ID是错的
            state_manager_->update_regulator_status(id, hw_status);
        }

        // 2. 处理回路数据 (硬件 -> 常规状态)
        for (uint8_t id = 1; id <= StateManager::NUM_CIRCUITS; ++id)
        {
            auto hw_status = state_manager_->get_hardware_circuit_status(id);
            hw_status.circuit_id = id; // 防止硬件发的ID是错的
            auto conventional_status = state_manager_->get_circuit_status(id);

            // 映射底层状态
            conventional_status.test_loop.hardware_loop_status = hw_status.test_loop;
            conventional_status.ref_loop.hardware_loop_status = hw_status.ref_loop;
            conventional_status.header.stamp = hw_status.header.stamp;
            conventional_status.circuit_id = hw_status.circuit_id;

            state_manager_->update_circuit_status(id, conventional_status);
        }

        // 3. 执行策略更新 (计算PID、状态对账等)
        for (auto const& [id, strategy] : active_strategies_) {
            if (strategy) {
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
        // 注意：此处不直接调用 check_completion 以免递归过深，仅在回调链中处理即可
        // 实际上由于 is_syncing_ 保护，下一轮循环会处理
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

    // 如果所有标志位其实都已经是 true (极少情况)，这里需要手动释放锁
    // 为了简单起见，我们假设只要有任何一项没加载，都会触发回调来解锁 is_syncing_
    // 如果全部已加载，settings_synced_ 早就为 true，不会进入此函数。
}

// 辅助函数：判断系统是否就绪
bool ControlLogic::is_system_ready() const {
    return current_lifecycle_state_ == ros2_interfaces::msg::SystemStatus::STATE_NORMAL;
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
//                命令处理 (Gatekeeper Pattern)
// ---------------------------------------------------------

void ControlLogic::process_regulator_operation_command(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg)
{
    // [Gatekeeper] 非正常状态下，拒绝手动操作
    if (!is_system_ready()) {
        RCLCPP_WARN(rclcpp::get_logger("ControlLogic"), "Ignored regulator op command: System not READY.");
        return;
    }

    uint8_t circuit_id = msg->regulator_id; // 假设 1:1 映射
    if (active_strategies_.count(circuit_id)) {
        active_strategies_[circuit_id]->handle_manual_command(msg);
    }
}

void ControlLogic::handle_regulator_breaker_command_request(const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request>& request, LogicResultCallback callback)
{
    // [Gatekeeper]
    if (!is_system_ready()) {
        if(callback) callback(false, "System not READY (Initializing or Error)");
        return;
    }
    hardware_coordinator_->execute_regulator_breaker_command(request, callback);
}

void ControlLogic::handle_circuit_breaker_command_request(const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request>& request, LogicResultCallback callback)
{
    // [Gatekeeper]
    if (!is_system_ready()) {
        if(callback) callback(false, "System not READY");
        return;
    }
    hardware_coordinator_->execute_circuit_breaker_command(request, callback);
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
//                参数设置处理 (Gatekeeper + Chain)
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

    state_manager_->update_system_settings(request->settings);

    persistence_coordinator_->save_system_settings(
        [this, callback](bool success, const std::string& message) {
            if (callback) {
                std::string final_message = success ?
                                                "System settings updated and saved." :
                                                "Settings updated in memory, but failed to save: " + message;
                callback(success, final_message);
            }
        });
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

    // Chain: Hardware -> Memory -> Persistence
    hardware_coordinator_->apply_regulator_settings_to_hardware(
        id, settings,
        [this, id, settings, callback](bool hw_success, const std::string& hw_message) {
            if (!hw_success) {
                if (callback) callback(false, "Hardware rejected settings: " + hw_message);
                return;
            }
            state_manager_->update_regulator_settings(id, settings);

            persistence_coordinator_->save_regulator_settings(
                id,
                [this, id, callback](bool ps_success, const std::string& ps_message) {
                    if (callback) {
                        std::string final_message = ps_success ?
                                                        "Regulator settings applied and saved." :
                                                        "Hardware updated, but failed to save: " + ps_message;
                        callback(ps_success, final_message);
                    }
                });
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
    const auto& settings = request->settings;

    ros2_interfaces::msg::HardwareCircuitSettings hw_settings;
    hw_settings.circuit_id = id;
    hw_settings.test_loop = settings.test_loop.hardware_loop_settings;
    hw_settings.ref_loop = settings.ref_loop.hardware_loop_settings;

    // Chain: Hardware -> Memory -> Persistence
    hardware_coordinator_->apply_circuit_settings_to_hardware(
        id, hw_settings,
        [this, id, settings, callback](bool hw_success, const std::string& hw_message) {
            if (!hw_success) {
                if (callback) callback(false, "Hardware rejected settings: " + hw_message);
                return;
            }
            state_manager_->update_circuit_settings(id, settings);

            persistence_coordinator_->save_circuit_settings(
                id,
                [this, id, callback](bool ps_success, const std::string& ps_message) {
                    if (callback) {
                        std::string final_message = ps_success ?
                                                        "Circuit settings applied and saved." :
                                                        "Hardware updated, but failed to save: " + ps_message;
                        callback(ps_success, final_message);
                    }
                });
        });
}

// ---------------------------------------------------------
//                辅助函数与默认设置
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
    settings.sample_interval_sec = 1;     // 1秒采样一次
    settings.record_interval_min = 5;     // 5分钟存一次盘
    settings.keep_record_on_shutdown = true;
    return settings;
}

// --- 调压器默认设置 ---
ros2_interfaces::msg::RegulatorSettings ControlLogic::create_default_regulator_settings(uint8_t id) {
    ros2_interfaces::msg::RegulatorSettings settings;
    settings.regulator_id = id;
    settings.over_voltage_v = 450;        // 安全电压阈值
    settings.over_current_a = 100;        // 安全电流阈值
    settings.voltage_up_speed_percent = 10;
    settings.voltage_down_speed_percent = 20;
    return settings;
}

// --- 回路默认设置 ---
ros2_interfaces::msg::CircuitSettings ControlLogic::create_default_circuit_settings(uint8_t id) {
    ros2_interfaces::msg::CircuitSettings settings;
    settings.circuit_id = id;
    settings.test_loop.enabled = false;   // 默认不启动
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
        default_status.regulator_id = i; // <--- 关键修正
        state_manager_->update_regulator_status(i, default_status);
    }

    for (uint8_t i = 1; i <= StateManager::NUM_CIRCUITS; ++i) {
        ros2_interfaces::msg::CircuitStatus default_status;
        default_status.circuit_id = i; // <--- 关键修正
        state_manager_->update_circuit_status(i, default_status);
    }
}
