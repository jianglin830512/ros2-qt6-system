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
    last_sync_time_(0, 0, RCL_SYSTEM_TIME)
{
}

void AutoCurrentStrategy::reset()
{
    last_sync_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
}

void AutoCurrentStrategy::update()
{
    auto now = rclcpp::Clock().now();

    // 频率限制 (例如 10Hz)，避免过于频繁发送指令
    if (now - last_sync_time_ < rclcpp::Duration::from_seconds(0.1)) {
        return;
    }
    last_sync_time_ = now;

    // 1. 获取最新状态
    auto circuit_status = state_manager_->get_circuit_status(circuit_id_);
    // 假设 Regulator ID 与 Circuit ID 一一对应
    auto regulator_status = state_manager_->get_regulator_status(circuit_id_);
    auto settings = state_manager_->get_circuit_settings(circuit_id_);

    bool test_heating = circuit_status.test_loop.is_heat;
    bool ref_heating = circuit_status.ref_loop.is_heat;
    bool system_active = test_heating || ref_heating;

    // =========================================================
    // [规则3] 自动合/分闸逻辑
    // =========================================================

    // --- 试验回路 (Test Loop) ---
    // 字段: breaker_closed_switch_ack
    bool test_breaker_closed = circuit_status.test_loop.hardware_loop_status.breaker_closed_switch_ack;

    if (test_heating && !test_breaker_closed) {
        // 需要加热但开关开着 -> 合闸
        auto req = std::make_shared<CircuitBreakerCmd::Request>();
        req->circuit_id = circuit_id_;
        // 根据 CircuitBreakerCommand.srv 定义，使用 CMD_TEST_BREAKER_CLOSE
        req->command = CircuitBreakerCmd::Request::CMD_TEST_BREAKER_CLOSE;
        hardware_coordinator_->execute_circuit_breaker_command(req, nullptr);
        RCLCPP_INFO(rclcpp::get_logger("AutoStrategy"), "Auto-Closing Test Loop Breaker");
    }
    else if (!test_heating && test_breaker_closed) {
        // 不需要加热但开关合着 -> 分闸
        auto req = std::make_shared<CircuitBreakerCmd::Request>();
        req->circuit_id = circuit_id_;
        req->command = CircuitBreakerCmd::Request::CMD_TEST_BREAKER_OPEN;
        hardware_coordinator_->execute_circuit_breaker_command(req, nullptr);
        RCLCPP_INFO(rclcpp::get_logger("AutoStrategy"), "Auto-Opening Test Loop Breaker");
    }

    // --- 参考回路 (Ref Loop) ---
    // 假设: Ref Loop 对应接口定义中的 CMD_SIM_BREAKER (模拟回路)
    bool ref_breaker_closed = circuit_status.ref_loop.hardware_loop_status.breaker_closed_switch_ack;

    if (ref_heating && !ref_breaker_closed) {
        auto req = std::make_shared<CircuitBreakerCmd::Request>();
        req->circuit_id = circuit_id_;
        req->command = CircuitBreakerCmd::Request::CMD_SIM_BREAKER_CLOSE;
        hardware_coordinator_->execute_circuit_breaker_command(req, nullptr);
        RCLCPP_INFO(rclcpp::get_logger("AutoStrategy"), "Auto-Closing Ref Loop Breaker");
    }
    else if (!ref_heating && ref_breaker_closed) {
        auto req = std::make_shared<CircuitBreakerCmd::Request>();
        req->circuit_id = circuit_id_;
        req->command = CircuitBreakerCmd::Request::CMD_SIM_BREAKER_OPEN;
        hardware_coordinator_->execute_circuit_breaker_command(req, nullptr);
        RCLCPP_INFO(rclcpp::get_logger("AutoStrategy"), "Auto-Opening Ref Loop Breaker");
    }

    // =========================================================
    // [规则3 & 规则2] 调压器与整机状态控制
    // =========================================================

    // 字段: breaker_closed_switch_ack (来自 RegulatorStatus.msg)
    bool reg_breaker_closed = regulator_status.breaker_closed_switch_ack;

    if (system_active) {
        // --- 处于加热状态 ---

        // 1. 确保 PLC 在 Auto Current 模式
        if (current_plc_control_mode_ != HwStatus::PLC_MODE_AUTO_CURRENT) {
            hardware_coordinator_->set_circuit_control_mode(circuit_id_, HwStatus::PLC_MODE_AUTO_CURRENT);
        }

        // 2. 确定目标源 (Source)
        uint8_t target_source = settings.curr_mode_use_ref ?
                                    HwStatus::PLC_SOURCE_REF_LOOP :
                                    HwStatus::PLC_SOURCE_TEST_LOOP;
        if (current_plc_control_source_ != target_source) {
            hardware_coordinator_->set_circuit_control_source(circuit_id_, target_source);
        }

        // 3. 自动合调压器闸
        if (!reg_breaker_closed) {
            auto req = std::make_shared<RegulatorBreakerCmd::Request>();
            req->regulator_id = circuit_id_;
            req->command = RegulatorBreakerCmd::Request::CMD_BREAKER_CLOSE;
            hardware_coordinator_->execute_regulator_breaker_command(req, nullptr);
            RCLCPP_INFO(rclcpp::get_logger("AutoStrategy"), "Auto-Closing Regulator Breaker");
        }

    } else {
        // --- 处于停止状态 (Idle/Cooling/Disabled) ---

        // 字段: lower_limit_switch_on 和 voltage_reading (来自 RegulatorStatus.msg)
        bool at_lower_limit = regulator_status.lower_limit_switch_on || (regulator_status.voltage_reading < 2.0);

        if (!at_lower_limit) {
            // [规则3] 尚未降到底 -> 发送降压指令
            auto cmd = std::make_shared<RegulatorOpCmd>();
            cmd->regulator_id = circuit_id_;
            cmd->command = RegulatorOpCmd::CMD_VOLTAGE_DOWN;
            hardware_coordinator_->send_regulator_operation_command(cmd);
        }
        else {
            // [规则3] 已降到底 -> 立刻关闭对应调压器开关
            if (reg_breaker_closed) {
                // 发送分闸指令
                auto req = std::make_shared<RegulatorBreakerCmd::Request>();
                req->regulator_id = circuit_id_;
                req->command = RegulatorBreakerCmd::Request::CMD_BREAKER_OPEN;
                hardware_coordinator_->execute_regulator_breaker_command(req, nullptr);
                RCLCPP_INFO(rclcpp::get_logger("AutoStrategy"), "System Idle & Low Voltage: Opening Regulator Breaker");
            }
            else {
                // [规则2] 确认PLC的控制模式是手动模式
                // 只有当调压器分闸且系统不加热时，才安全切换回手动
                if (current_plc_control_mode_ != HwStatus::PLC_MODE_MANUAL) {
                    hardware_coordinator_->set_circuit_control_mode(circuit_id_, HwStatus::PLC_MODE_MANUAL);
                    RCLCPP_INFO(rclcpp::get_logger("AutoStrategy"), "System Safely Stopped. Switching PLC to MANUAL.");
                }
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
