#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp" // IWYU pragma: keep
//#include <memory>

// --- 消息和服务的头文件 ---
// Status & Settings (发布用)
#include "ros2_interfaces/msg/regulator_status.hpp"
#include "ros2_interfaces/msg/circuit_status.hpp"
#include "ros2_interfaces/msg/system_settings.hpp"
#include "ros2_interfaces/msg/regulator_settings.hpp"
#include "ros2_interfaces/msg/circuit_settings.hpp"
// Commands (订阅用)
#include "ros2_interfaces/msg/regulator_operation_command.hpp"
#include "std_msgs/msg/empty.hpp"
// Command Services (Server用)
#include "ros2_interfaces/srv/regulator_breaker_command.hpp"
#include "ros2_interfaces/srv/circuit_mode_command.hpp"
#include "ros2_interfaces/srv/circuit_breaker_command.hpp"
// Settings Services (Server用)
#include "ros2_interfaces/srv/set_system_settings.hpp"
#include "ros2_interfaces/srv/set_regulator_settings.hpp"
#include "ros2_interfaces/srv/set_circuit_settings.hpp"


// --- 前向声明 ---
class StateManager;
class IHardwareCoordinator;
class IPersistenceCoordinator;
class ControlLogic;

/**
 * @class ControlNode
 * @brief 项目的主ROS节点，负责创建和管理核心组件，并处理所有ROS通信。
 *
 * 此类继承自 rclcpp::Node 和 std::enable_shared_from_this，
 * 以便能够将自身的共享指针传递给它所创建的组件。
 * 它拥有核心组件的所有实例，并负责依赖注入。
 */
class ControlNode : public rclcpp::Node
{
public:
    ControlNode();
    ~ControlNode();

    // 新的初始化方法，在构造后调用
    void initialize_components();

private:
    // === 回调函数声明 ===
    void regulator_operation_command_callback(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg);
    void clear_alarm_callback(const std_msgs::msg::Empty::SharedPtr msg);
    void regulator_breaker_command_callback(
        const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request> request,
        std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Response> response);
    void circuit_mode_command_callback(
        const std::shared_ptr<ros2_interfaces::srv::CircuitModeCommand::Request> request,
        std::shared_ptr<ros2_interfaces::srv::CircuitModeCommand::Response> response);
    void circuit_breaker_command_callback(
        const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request> request,
        std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Response> response);
    void set_system_settings_callback(
        const std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Request> request,
        std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Response> response);
    void set_regulator_settings_callback(
        const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request> request,
        std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Response> response);
    void set_circuit_settings_callback(
        const std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Request> request,
        std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Response> response);
    void broadcast_status_callback(); // 广播回调
    void broadcast_settings_callback();

    // === ROS 句柄 (API层) ===
    // Publishers
    rclcpp::Publisher<ros2_interfaces::msg::CircuitStatus>::SharedPtr circuit_status_pub_;
    rclcpp::Publisher<ros2_interfaces::msg::RegulatorStatus>::SharedPtr regulator_status_pub_;
    // Settings Publishers
    rclcpp::Publisher<ros2_interfaces::msg::SystemSettings>::SharedPtr system_settings_pub_;
    rclcpp::Publisher<ros2_interfaces::msg::RegulatorSettings>::SharedPtr regulator_settings_pub_;
    rclcpp::Publisher<ros2_interfaces::msg::CircuitSettings>::SharedPtr circuit_settings_pub_;

    // Subscribers
    rclcpp::Subscription<ros2_interfaces::msg::RegulatorOperationCommand>::SharedPtr regulator_operation_command_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_alarm_sub_;

    // Service Servers
    rclcpp::Service<ros2_interfaces::srv::SetSystemSettings>::SharedPtr set_system_settings_service_;
    rclcpp::Service<ros2_interfaces::srv::SetRegulatorSettings>::SharedPtr set_regulator_settings_service_;
    rclcpp::Service<ros2_interfaces::srv::SetCircuitSettings>::SharedPtr set_circuit_settings_service_;
    // Command Service Servers
    rclcpp::Service<ros2_interfaces::srv::RegulatorBreakerCommand>::SharedPtr regulator_breaker_command_service_;
    rclcpp::Service<ros2_interfaces::srv::CircuitModeCommand>::SharedPtr circuit_mode_command_service_;
    rclcpp::Service<ros2_interfaces::srv::CircuitBreakerCommand>::SharedPtr circuit_breaker_command_service_;

    // 驱动控制逻辑的核心定时器
    rclcpp::TimerBase::SharedPtr control_logic_timer_; // 50Hz 逻辑更新
    rclcpp::TimerBase::SharedPtr status_broadcast_timer_;     // 较慢的广播
    rclcpp::TimerBase::SharedPtr settings_broadcast_timer_;     // 较慢的广播
    rclcpp::TimerBase::SharedPtr lifecycle_check_timer_;

    // === 核心组件实例 (由本节点拥有) ===
    std::shared_ptr<StateManager> state_manager_;
    std::shared_ptr<IHardwareCoordinator> hardware_coordinator_;
    std::shared_ptr<IPersistenceCoordinator> persistence_coordinator_;
    std::unique_ptr<ControlLogic> control_logic_;

    // === 多线程的组 ===
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr server_cb_group_;
};

#endif // CONTROL_NODE_HPP_
