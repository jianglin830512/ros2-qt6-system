#include "control_node/strategies/auto_current_strategy.hpp"
#include "ros2_interfaces/msg/hardware_circuit_status.hpp"
#include "ros2_interfaces/msg/regulator_status.hpp"
#include "ros2_interfaces/srv/circuit_breaker_command.hpp"
#include "ros2_interfaces/srv/regulator_breaker_command.hpp"
#include "ros2_interfaces/msg/regulator_operation_command.hpp"

using HwStatus = ros2_interfaces::msg::HardwareCircuitStatus;
using CircuitBreakerCmd = ros2_interfaces::srv::CircuitBreakerCommand;
using RegulatorBreakerCmd = ros2_interfaces::srv::RegulatorBreakerCommand;
using RegulatorOpCmd = ros2_interfaces::msg::RegulatorOperationCommand;

AutoCurrentStrategy::AutoCurrentStrategy(
    uint8_t circuit_id,
    std::shared_ptr<StateManager> state_manager,
    std::shared_ptr<IHardwareCoordinator> hardware_coordinator)
    : IControllerStrategy(circuit_id, state_manager, hardware_coordinator),
    last_sync_time_(0, 0, RCL_SYSTEM_TIME),
    last_plc_sync_time_(0, 0, RCL_SYSTEM_TIME) // [NEW] 初始化
{
}

void AutoCurrentStrategy::reset()
{
    last_sync_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
    last_plc_sync_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
}

void AutoCurrentStrategy::update()
{
    auto now = rclcpp::Clock().now();

    // 频率限制 (10Hz)，避免过于频繁发送指令
    if (now - last_sync_time_ < rclcpp::Duration::from_seconds(0.1)) {
        return;
    }
    last_sync_time_ = now;

    // 1. 获取最新状态和配置
    auto circuit_status = state_manager_->get_circuit_status(circuit_id_);
    auto regulator_status = state_manager_->get_regulator_status(circuit_id_);
    auto settings = state_manager_->get_circuit_settings(circuit_id_);

    bool test_heating = circuit_status.test_loop.is_heat;
    bool ref_heating = circuit_status.ref_loop.is_heat;

    // 全局加热状态：只要有一个支路在加热，系统就是激活状态
    bool system_active = test_heating || ref_heating;

    // 获取开关和限位反馈
    bool test_breaker_closed = circuit_status.test_loop.hardware_loop_status.breaker_closed_switch_ack;
    bool ref_breaker_closed = circuit_status.ref_loop.hardware_loop_status.breaker_closed_switch_ack;
    bool reg_breaker_closed = regulator_status.breaker_closed_switch_ack;
    bool at_lower_limit = regulator_status.lower_limit_switch_on || (regulator_status.voltage_reading < 2.0);

    // =========================================================
    // [NEW] PLC 状态一致性检查与强制同步 (Mode & Source)
    // =========================================================

    // 决定目标 PLC 反馈源：只要处于 AUTO 模式，就强制保持一致
    uint8_t target_plc_source = settings.curr_mode_use_ref ?
                                    HwStatus::PLC_SOURCE_REF_LOOP :
                                    HwStatus::PLC_SOURCE_TEST_LOOP;

    // 决定目标 PLC 控制模式：默认安全状态为手动
    uint8_t target_plc_mode = HwStatus::PLC_MODE_MANUAL;

    // 判断开关条件是否准备好（决定了是否允许切到 AUTO_CURRENT）
    bool breakers_ready = false;
    if (system_active) {
        bool t_ready = (!test_heating || test_breaker_closed);
        bool r_ready = (!ref_heating || ref_breaker_closed);
        if (t_ready && r_ready && reg_breaker_closed) {
            breakers_ready = true;
            target_plc_mode = HwStatus::PLC_MODE_AUTO_CURRENT;
        }
    }

    // 判断是否与硬件当前状态存在不一致
    bool source_mismatch = (current_plc_control_source_ != target_plc_source);
    bool mode_mismatch = (current_plc_control_mode_ != target_plc_mode);

    // 如果不一致，且距离上次下发同步指令超过了 1 秒（防止 Service 被频繁请求淹没）
    if ((source_mismatch || mode_mismatch) && (now - last_plc_sync_time_ >= rclcpp::Duration::from_seconds(1.0))) {
        if (source_mismatch) {
            RCLCPP_INFO(rclcpp::get_logger("AutoStrategy"), "[Circuit %d] PLC Source mismatch (Current: %d, Target: %d). Enforcing...",
                        circuit_id_, current_plc_control_source_, target_plc_source);
            hardware_coordinator_->set_circuit_control_source(circuit_id_, target_plc_source);
        }
        if (mode_mismatch) {
            RCLCPP_INFO(rclcpp::get_logger("AutoStrategy"), "[Circuit %d] PLC Mode mismatch (Current: %d, Target: %d). Enforcing...",
                        circuit_id_, current_plc_control_mode_, target_plc_mode);
            hardware_coordinator_->set_circuit_control_mode(circuit_id_, target_plc_mode);
        }
        last_plc_sync_time_ = now; // 重置冷却定时器
    }


    // =========================================================
    // 核心状态机：根据全局加热状态决定时序
    // =========================================================

    if (system_active) {
        // ---------------------------------------------------------
        // 【加热阶段】
        // 逻辑：先确保需要的开关全部合闸 -> 确认无误后 -> 下发PLC控制字
        // (注: PLC控制字下发已经统一在上面的同步检查中了，此处仅关注开关)
        // ---------------------------------------------------------

        // 1. 控制试验回路支路开关
        if (test_heating && !test_breaker_closed) {
            auto req = std::make_shared<CircuitBreakerCmd::Request>();
            req->circuit_id = circuit_id_;
            req->command = CircuitBreakerCmd::Request::CMD_TEST_BREAKER_CLOSE;
            hardware_coordinator_->execute_circuit_breaker_command(req, nullptr);
            RCLCPP_INFO(rclcpp::get_logger("AutoStrategy"), "Heating: Closing Test Loop Breaker...");
        } else if (!test_heating && test_breaker_closed) {
            auto req = std::make_shared<CircuitBreakerCmd::Request>();
            req->circuit_id = circuit_id_;
            req->command = CircuitBreakerCmd::Request::CMD_TEST_BREAKER_OPEN;
            hardware_coordinator_->execute_circuit_breaker_command(req, nullptr);
            RCLCPP_INFO(rclcpp::get_logger("AutoStrategy"), "Heating: Opening inactive Test Loop Breaker...");
        }

        // 2. 控制参考(模拟)回路支路开关
        if (ref_heating && !ref_breaker_closed) {
            auto req = std::make_shared<CircuitBreakerCmd::Request>();
            req->circuit_id = circuit_id_;
            req->command = CircuitBreakerCmd::Request::CMD_SIM_BREAKER_CLOSE;
            hardware_coordinator_->execute_circuit_breaker_command(req, nullptr);
            RCLCPP_INFO(rclcpp::get_logger("AutoStrategy"), "Heating: Closing Ref Loop Breaker...");
        } else if (!ref_heating && ref_breaker_closed) {
            auto req = std::make_shared<CircuitBreakerCmd::Request>();
            req->circuit_id = circuit_id_;
            req->command = CircuitBreakerCmd::Request::CMD_SIM_BREAKER_OPEN;
            hardware_coordinator_->execute_circuit_breaker_command(req, nullptr);
            RCLCPP_INFO(rclcpp::get_logger("AutoStrategy"), "Heating: Opening inactive Ref Loop Breaker...");
        }

        // 3. 控制调压器总开关
        if (!reg_breaker_closed) {
            auto req = std::make_shared<RegulatorBreakerCmd::Request>();
            req->regulator_id = circuit_id_;
            req->command = RegulatorBreakerCmd::Request::CMD_BREAKER_CLOSE;
            hardware_coordinator_->execute_regulator_breaker_command(req, nullptr);
            RCLCPP_INFO(rclcpp::get_logger("AutoStrategy"), "Heating: Closing Regulator Breaker...");
        }

    } else {
        // ---------------------------------------------------------
        // 【停止/冷却阶段】
        // 逻辑：先降压至 0 -> 确认到底后 -> 同时断开所有支路和调压器开关 -> 切回手动模式
        // (注: 切回手动模式已经统一在上面的同步检查中完成)
        // ---------------------------------------------------------

        if (!at_lower_limit) {
            // 第一步：尚未降到底，下发降压指令（此时保持开关合闸状态，以释放残存电能或避免电弧）
            auto cmd = std::make_shared<RegulatorOpCmd>();
            cmd->regulator_id = circuit_id_;
            cmd->command = RegulatorOpCmd::CMD_VOLTAGE_DOWN;
            hardware_coordinator_->send_regulator_operation_command(cmd);
        }
        else {
            // 第二步：已经降到底，开始处理分闸
            // 断开调压器开关
            if (reg_breaker_closed) {
                auto req = std::make_shared<RegulatorBreakerCmd::Request>();
                req->regulator_id = circuit_id_;
                req->command = RegulatorBreakerCmd::Request::CMD_BREAKER_OPEN;
                hardware_coordinator_->execute_regulator_breaker_command(req, nullptr);
                RCLCPP_INFO(rclcpp::get_logger("AutoStrategy"), "Voltage at 0: Opening Regulator Breaker.");
            }

            // 断开试验回路支路开关
            if (test_breaker_closed) {
                auto req = std::make_shared<CircuitBreakerCmd::Request>();
                req->circuit_id = circuit_id_;
                req->command = CircuitBreakerCmd::Request::CMD_TEST_BREAKER_OPEN;
                hardware_coordinator_->execute_circuit_breaker_command(req, nullptr);
                RCLCPP_INFO(rclcpp::get_logger("AutoStrategy"), "Voltage at 0: Opening Test Loop Breaker.");
            }

            // 断开参考回路支路开关
            if (ref_breaker_closed) {
                auto req = std::make_shared<CircuitBreakerCmd::Request>();
                req->circuit_id = circuit_id_;
                req->command = CircuitBreakerCmd::Request::CMD_SIM_BREAKER_OPEN;
                hardware_coordinator_->execute_circuit_breaker_command(req, nullptr);
                RCLCPP_INFO(rclcpp::get_logger("AutoStrategy"), "Voltage at 0: Opening Ref Loop Breaker.");
            }
        }
    }
}

// --- Command Handlers: Blocked in Auto Mode ---

void AutoCurrentStrategy::handle_regulator_operation_command(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg)
{
    (void)msg;
    RCLCPP_WARN(rclcpp::get_logger("AutoCurrentStrategy"),
                "Blocked regulator operation command for ID %d. System is in AUTO mode.", circuit_id_);
}

void AutoCurrentStrategy::handle_regulator_breaker_command(
    const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request>& request,
    StrategyCallback callback)
{
    (void)request;
    if (callback) {
        callback(false, "Command BLOCKED: Circuit is in AUTO mode. Switch to Manual to operate breakers.");
    }
}

void AutoCurrentStrategy::handle_circuit_breaker_command(
    const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request>& request,
    StrategyCallback callback)
{
    (void)request;
    if (callback) {
        callback(false, "Command BLOCKED: Circuit is in AUTO mode. Switch to Manual to operate breakers.");
    }
}
