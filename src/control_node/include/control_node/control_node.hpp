#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "ros2_interfaces/msg/regulator_status.hpp"
#include "ros2_interfaces/msg/circuit_status.hpp"
#include "ros2_interfaces/msg/system_settings.hpp"
#include "ros2_interfaces/msg/regulator_settings.hpp"
#include "ros2_interfaces/msg/circuit_settings.hpp"
#include "ros2_interfaces/msg/system_status.hpp"
#include "ros2_interfaces/msg/regulator_operation_command.hpp"
#include "std_msgs/msg/empty.hpp"
#include "ros2_interfaces/srv/regulator_breaker_command.hpp"
#include "ros2_interfaces/srv/circuit_breaker_command.hpp"
#include "ros2_interfaces/srv/set_system_settings.hpp"
#include "ros2_interfaces/srv/set_regulator_settings.hpp"
#include "ros2_interfaces/srv/set_circuit_settings.hpp"
#include "ros2_interfaces/srv/get_cable_info_batch.hpp" // [NEW] 引入全局电缆信息服务

class StateManager;
class IHardwareCoordinator;
class IPersistenceCoordinator;
class ControlLogic;

class ControlNode : public rclcpp::Node
{
public:
    ControlNode();
    ~ControlNode();
    void initialize_components();

private:
    void regulator_operation_command_callback(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg);
    void clear_alarm_callback(const std_msgs::msg::Empty::SharedPtr msg);
    void regulator_breaker_command_callback(
        const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request> request,
        std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Response> response);
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
    void broadcast_status_callback();
    void broadcast_settings_callback();

    // [NEW] 电缆信息定时同步回调
    void cable_sync_timer_callback();

    rclcpp::Publisher<ros2_interfaces::msg::CircuitStatus>::SharedPtr circuit_status_pub_;
    rclcpp::Publisher<ros2_interfaces::msg::RegulatorStatus>::SharedPtr regulator_status_pub_;
    rclcpp::Publisher<ros2_interfaces::msg::SystemStatus>::SharedPtr system_status_pub_;
    rclcpp::Publisher<ros2_interfaces::msg::SystemSettings>::SharedPtr system_settings_pub_;
    rclcpp::Publisher<ros2_interfaces::msg::RegulatorSettings>::SharedPtr regulator_settings_pub_;
    rclcpp::Publisher<ros2_interfaces::msg::CircuitSettings>::SharedPtr circuit_settings_pub_;

    rclcpp::Subscription<ros2_interfaces::msg::RegulatorOperationCommand>::SharedPtr regulator_operation_command_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_alarm_sub_;

    rclcpp::Service<ros2_interfaces::srv::SetSystemSettings>::SharedPtr set_system_settings_service_;
    rclcpp::Service<ros2_interfaces::srv::SetRegulatorSettings>::SharedPtr set_regulator_settings_service_;
    rclcpp::Service<ros2_interfaces::srv::SetCircuitSettings>::SharedPtr set_circuit_settings_service_;
    rclcpp::Service<ros2_interfaces::srv::RegulatorBreakerCommand>::SharedPtr regulator_breaker_command_service_;
    rclcpp::Service<ros2_interfaces::srv::CircuitBreakerCommand>::SharedPtr circuit_breaker_command_service_;

    // [NEW] Global Node 获取电缆信息Client
    rclcpp::Client<ros2_interfaces::srv::GetCableInfoBatch>::SharedPtr get_cable_info_batch_client_;

    rclcpp::TimerBase::SharedPtr control_logic_timer_;
    rclcpp::TimerBase::SharedPtr status_broadcast_timer_;
    rclcpp::TimerBase::SharedPtr settings_broadcast_timer_;
    rclcpp::TimerBase::SharedPtr lifecycle_check_timer_;
    rclcpp::TimerBase::SharedPtr cable_sync_timer_; // [NEW] 定时同步计时器

    std::shared_ptr<StateManager> state_manager_;
    std::shared_ptr<IHardwareCoordinator> hardware_coordinator_;
    std::shared_ptr<IPersistenceCoordinator> persistence_coordinator_;
    std::unique_ptr<ControlLogic> control_logic_;

    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr server_cb_group_;
};
#endif
