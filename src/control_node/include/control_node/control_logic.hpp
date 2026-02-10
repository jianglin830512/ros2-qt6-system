#ifndef CONTROL_LOGIC_HPP
#define CONTROL_LOGIC_HPP

#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp" // IWYU pragma: keep
// --- Include all necessary message and service types ---
#include "ros2_interfaces/srv/set_system_settings.hpp"
#include "ros2_interfaces/srv/set_regulator_settings.hpp"
#include "ros2_interfaces/srv/set_circuit_settings.hpp"
#include "ros2_interfaces/msg/regulator_operation_command.hpp"
#include "ros2_interfaces/srv/regulator_breaker_command.hpp"
#include "ros2_interfaces/srv/circuit_mode_command.hpp"
#include "ros2_interfaces/srv/circuit_breaker_command.hpp"
#include "control_node/i_controller_strategy.hpp"

// === Forward Declarations (using interfaces) ===
class StateManager;
class IHardwareCoordinator;
class IPersistenceCoordinator;
class ControlNode; // Forward declare to avoid circular dependency

// Callback type for reporting results back to ControlNode
using LogicResultCallback = std::function<void(bool success, const std::string& message)>;

/**
 * @class ControlLogic
 * @brief Encapsulates the core business logic of the system.
 *
 * This class is decoupled from ROS APIs. It makes decisions based on state
 * and delegates external actions (hardware, persistence) through interfaces.
 */
class ControlLogic
{
public:
    ControlLogic(
        std::shared_ptr<StateManager> state_manager,
        std::shared_ptr<IHardwareCoordinator> hardware_coordinator,
        std::shared_ptr<IPersistenceCoordinator> persistence_coordinator);

    void update();
    void maintain_lifecycle();  // 生命周期维护函数，由 ControlNode 的低频定时器调用
    void switch_mode(uint8_t circuit_id, uint8_t new_mode);
    bool is_system_ready() const;

    // [NEW] 暴露设置同步状态，用于防止广播默认值覆盖数据库
    bool is_settings_synced() const;

    // --- Handlers for commands from ControlNode ---
    void process_regulator_operation_command(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg);
    void handle_regulator_breaker_command_request(const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request>& request, LogicResultCallback callback);
    void handle_circuit_mode_command_request(const std::shared_ptr<ros2_interfaces::srv::CircuitModeCommand::Request>& request, LogicResultCallback callback);
    void handle_circuit_breaker_command_request(const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request>& request, LogicResultCallback callback);
    void process_clear_alarm();

    // --- Handlers for service requests from ControlNode ---
    void handle_set_system_settings_request(
        const std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Request>& request,
        LogicResultCallback callback);

    void handle_set_regulator_settings_request(
        const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request>& request,
        LogicResultCallback callback);

    void handle_set_circuit_settings_request(
        const std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Request>& request,
        LogicResultCallback callback);

    void handle_update_keep_record_on_shutdown(
        bool keep_record,
        LogicResultCallback callback);

private:

    void initialize_all_default_settings();  // 直接初始化参数，无需读取数据库内容
    ros2_interfaces::msg::SystemSettings    create_default_system_settings();
    ros2_interfaces::msg::RegulatorSettings create_default_regulator_settings(uint8_t id);
    ros2_interfaces::msg::CircuitSettings   create_default_circuit_settings(uint8_t id);
    void attempt_sync_settings();   // 尝试同步设置的内部函数
    void execute_safety_shutdown(); // 安全停机逻辑

    // [NEW] 业务逻辑处理 helper
    void process_circuit_logic(uint8_t circuit_id);
    bool process_single_loop_logic(
        ros2_interfaces::msg::LoopSettings& settings,
        ros2_interfaces::msg::LoopStatus& status,
        const rclcpp::Time& now);

    // Use raw pointers for dependencies managed by ControlNode
    std::shared_ptr<StateManager> state_manager_;
    std::shared_ptr<IHardwareCoordinator> hardware_coordinator_;
    std::shared_ptr<IPersistenceCoordinator> persistence_coordinator_;

    // Each circuit can have its own strategy
    std::map<uint8_t, std::unique_ptr<IControllerStrategy>> active_strategies_;

    bool settings_synced_ = false;       // 所有设置是否已成功从数据库加载
    bool is_syncing_ = false;            // 是否正在等待回调（防止重入/请求风暴）

    // 细分标志位，用于判断是否全部完成
    bool sys_settings_loaded_ = false;
    bool reg_settings_loaded_[StateManager::NUM_REGULATORS + 1] = {false}; // +1 为了方便下标从1开始
    bool cir_settings_loaded_[StateManager::NUM_CIRCUITS + 1] = {false};

    // 缓存当前的生命周期状态，用于高频循环中的判断
    uint8_t current_lifecycle_state_ = 0; // 默认为 INITIALIZING
};

#endif // CONTROL_LOGIC_HPP
