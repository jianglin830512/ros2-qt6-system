#include "control_node/strategies/auto_current_strategy.hpp"
#include "ros2_interfaces/msg/hardware_circuit_status.hpp"

// 使用别名简化代码
using HwStatus = ros2_interfaces::msg::HardwareCircuitStatus;

AutoCurrentStrategy::AutoCurrentStrategy(
    uint8_t circuit_id,
    std::shared_ptr<StateManager> state_manager,
    std::shared_ptr<IHardwareCoordinator> hardware_coordinator)
    : IControllerStrategy(circuit_id, state_manager, hardware_coordinator),
    last_command_time_(0, 0, RCL_SYSTEM_TIME)
{
}

void AutoCurrentStrategy::reset()
{
    // 清除 PID 积分项和中间状态
    integral_error_ = 0.0;
    last_error_ = 0.0;
    // 重置命令冷却时间，允许立即发送指令
    last_command_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
}

void AutoCurrentStrategy::update()
{
    auto now = rclcpp::Clock().now();

    // 1. 获取最新状态和设置
    auto hw_status = state_manager_->get_hardware_circuit_status(circuit_id_);
    auto settings = state_manager_->get_circuit_settings(circuit_id_);

    // 假设 Regulator ID 与 Circuit ID 一一对应
    uint8_t regulator_id = circuit_id_;

    // =========================================================
    // Phase 1: 状态对账 (State Reconciliation)
    // 确保 PLC 处于正确的模式和源
    // =========================================================

    // 1.1 检查模式是否为自动恒流
    if (hw_status.plc_control_mode != HwStatus::PLC_MODE_AUTO_CURRENT) {
        if (now - last_command_time_ > command_cooldown_) {
            // 发送切换命令
            hardware_coordinator_->set_circuit_control_mode(circuit_id_, HwStatus::PLC_MODE_AUTO_CURRENT);
            last_command_time_ = now;
        }
        return; // 硬件未就绪，不执行控制算法
    }

    // 1.2 检查控制源 (根据设置决定是用 试验回路 还是 参考回路)
    uint8_t target_source = settings.curr_mode_use_ref ? HwStatus::PLC_SOURCE_REF_LOOP : HwStatus::PLC_SOURCE_TEST_LOOP;

    if (hw_status.plc_control_source != target_source) {
        if (now - last_command_time_ > command_cooldown_) {
            hardware_coordinator_->set_circuit_control_source(circuit_id_, target_source);
            last_command_time_ = now;
        }
        return; // 硬件未就绪
    }

    // =========================================================
    // Phase 2: 控制算法 (Control Loop)
    // 只有当 Phase 1 通过后，才执行闭环控制
    // =========================================================

    // 获取目标电流 (单位为 A)
    double target_current = 0.0;
    if (settings.curr_mode_use_ref) {
        target_current = static_cast<double>(settings.ref_loop.hardware_loop_settings.start_current_a);
    } else {
        target_current = static_cast<double>(settings.test_loop.hardware_loop_settings.start_current_a);
    }

    // 获取实际电流
    double actual_current = 0.0;
    if (settings.curr_mode_use_ref) {
        actual_current = hw_status.ref_loop.current;
    } else {
        actual_current = hw_status.test_loop.current;
    }

    // 检查是否应该停止
    // 如果目标电流极小，或者回路未启用(逻辑上)，应停止
    bool loop_enabled = settings.curr_mode_use_ref ? settings.ref_loop.enabled : settings.test_loop.enabled;
    if (!loop_enabled || target_current < 0.1) {
        // 发送停止指令以确保安全
        auto cmd = std::make_shared<ros2_interfaces::msg::RegulatorOperationCommand>();
        cmd->regulator_id = regulator_id;
        cmd->command = ros2_interfaces::msg::RegulatorOperationCommand::CMD_VOLTAGE_STOP;
        hardware_coordinator_->send_regulator_operation_command(cmd);
        return;
    }

    // --- 简单的 Bang-Bang 控制示例 (实际项目中可替换为 PID) ---
    double error = target_current - actual_current;

    // 死区设置 (例如 1A)
    if (std::abs(error) < 1.0) {
        auto cmd = std::make_shared<ros2_interfaces::msg::RegulatorOperationCommand>();
        cmd->regulator_id = regulator_id;
        cmd->command = ros2_interfaces::msg::RegulatorOperationCommand::CMD_VOLTAGE_STOP;
        hardware_coordinator_->send_regulator_operation_command(cmd);
    }
    else if (error > 0) {
        // 电流偏小 -> 升压
        auto cmd = std::make_shared<ros2_interfaces::msg::RegulatorOperationCommand>();
        cmd->regulator_id = regulator_id;
        cmd->command = ros2_interfaces::msg::RegulatorOperationCommand::CMD_VOLTAGE_UP;
        hardware_coordinator_->send_regulator_operation_command(cmd);
    }
    else {
        // 电流偏大 -> 降压
        auto cmd = std::make_shared<ros2_interfaces::msg::RegulatorOperationCommand>();
        cmd->regulator_id = regulator_id;
        cmd->command = ros2_interfaces::msg::RegulatorOperationCommand::CMD_VOLTAGE_DOWN;
        hardware_coordinator_->send_regulator_operation_command(cmd);
    }
}
