#ifndef HARDWARE_COORDINATOR_HPP
#define HARDWARE_COORDINATOR_HPP

#include "rclcpp/rclcpp.hpp"  // IWYU pragma: keep
#include "control_node/i_hardware_coordinator.hpp"
#include "control_node/state_manager.hpp"

// Service header needed for clients
#include "ros2_interfaces/srv/set_hardware_circuit_settings.hpp"
#include "ros2_interfaces/srv/set_regulator_settings.hpp"
// Include status/settings messages for subscriptions
#include "ros2_interfaces/msg/hardware_circuit_status.hpp"
#include "ros2_interfaces/msg/hardware_circuit_settings.hpp"
#include "ros2_interfaces/msg/regulator_status.hpp"
#include "ros2_interfaces/msg/regulator_settings.hpp"
#include "ros2_interfaces/srv/set_hardware_circuit_control_mode.hpp"
#include "ros2_interfaces/srv/set_hardware_circuit_control_source.hpp"
// Include command messages for publishers
#include "std_msgs/msg/empty.hpp"


/**
 * @class HardwareCoordinator
 * @brief Implements the concrete logic for interacting with the hardware_node.
 */
class HardwareCoordinator : public IHardwareCoordinator
{
public:
    HardwareCoordinator(StateManager* state_manager, rclcpp::Node::SharedPtr node);

    bool is_connected() const override;

    // --- Synchronous Interface Methods ---
    void apply_regulator_settings_to_hardware(
        uint8_t id,
        const ros2_interfaces::msg::RegulatorSettings& settings,
        HardwareCallback callback) override;
    void apply_circuit_settings_to_hardware(
        uint8_t id,
        const ros2_interfaces::msg::HardwareCircuitSettings& settings,
        HardwareCallback callback) override;

    // --- Asynchronous Interface Methods ---
    void send_regulator_operation_command(
        const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr command_msg) override;
    void execute_regulator_breaker_command(
        const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request> request,
        HardwareCallback callback) override;
    void execute_circuit_breaker_command(
        const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request> request,
        HardwareCallback callback) override;
    void send_clear_alarm() override;
    void set_circuit_control_mode(uint8_t circuit_id, uint8_t mode) override;
    void set_circuit_control_source(uint8_t circuit_id, uint8_t source) override;

private:
    // --- Internal Subscription Callbacks ---
    void hardware_circuit_status_callback(const ros2_interfaces::msg::HardwareCircuitStatus::SharedPtr msg);
    void hardware_regulator_status_callback(const ros2_interfaces::msg::RegulatorStatus::SharedPtr msg);
    void hardware_circuit_settings_callback(const ros2_interfaces::msg::HardwareCircuitSettings::SharedPtr msg);
    void hardware_regulator_settings_callback(const ros2_interfaces::msg::RegulatorSettings::SharedPtr msg);

    StateManager* state_manager_;
    rclcpp::Node::SharedPtr node_;

    // === ROS Handles (communication with hardware_node) ===
    // Subscriptions (receiving data from hardware_node)
    rclcpp::Subscription<ros2_interfaces::msg::HardwareCircuitStatus>::SharedPtr hw_circuit_status_sub_;
    rclcpp::Subscription<ros2_interfaces::msg::RegulatorStatus>::SharedPtr hw_regulator_status_sub_;
    rclcpp::Subscription<ros2_interfaces::msg::HardwareCircuitSettings>::SharedPtr hw_circuit_settings_sub_;
    rclcpp::Subscription<ros2_interfaces::msg::RegulatorSettings>::SharedPtr hw_regulator_settings_sub_;

    // Publishers (sending commands to hardware_node)
    rclcpp::Publisher<ros2_interfaces::msg::RegulatorOperationCommand>::SharedPtr hw_regulator_operation_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr hw_clear_alarm_pub_;

    // Service Clients (for settings and commands)
    rclcpp::Client<ros2_interfaces::srv::SetRegulatorSettings>::SharedPtr set_hw_regulator_settings_client_;
    rclcpp::Client<ros2_interfaces::srv::SetHardwareCircuitSettings>::SharedPtr set_hw_circuit_settings_client_;
    rclcpp::Client<ros2_interfaces::srv::RegulatorBreakerCommand>::SharedPtr hw_regulator_breaker_client_;
    rclcpp::Client<ros2_interfaces::srv::CircuitBreakerCommand>::SharedPtr hw_circuit_breaker_client_;
    rclcpp::Client<ros2_interfaces::srv::SetHardwareCircuitControlMode>::SharedPtr hw_set_control_mode_client_;
    rclcpp::Client<ros2_interfaces::srv::SetHardwareCircuitControlSource>::SharedPtr hw_set_control_source_client_;
};

#endif // HARDWARE_COORDINATOR_HPP

