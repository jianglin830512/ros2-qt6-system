// hardware_node/hardware_node.hpp

#ifndef HARDWARE_NODE_HPP_
#define HARDWARE_NODE_HPP_

#include "rclcpp/rclcpp.hpp" // IWYU pragma: keep

// [新增] 引入节流所需的头文件
#include <map>
#include <string>
#include <mutex>
#include <chrono>

// --- 消息和服务的头文件 ---
// ... (原有头文件不变)
#include "ros2_interfaces/msg/hardware_circuit_settings.hpp"
#include "ros2_interfaces/msg/hardware_circuit_status.hpp"
#include "ros2_interfaces/msg/regulator_operation_command.hpp"
#include "ros2_interfaces/msg/regulator_settings.hpp"
#include "ros2_interfaces/msg/regulator_status.hpp"
#include "ros2_interfaces/srv/circuit_breaker_command.hpp"
#include "ros2_interfaces/srv/regulator_breaker_command.hpp"
#include "ros2_interfaces/srv/set_hardware_circuit_settings.hpp"
#include "ros2_interfaces/srv/set_regulator_settings.hpp"
#include "ros2_interfaces/srv/set_hardware_circuit_control_mode.hpp"
#include "ros2_interfaces/srv/set_hardware_circuit_control_source.hpp"
#include "std_msgs/msg/empty.hpp"


// --- 前向声明 ---
class IHardwareDriver;

using namespace std::chrono_literals;

class HardwareNode : public rclcpp::Node
{
public:
    HardwareNode();
    ~HardwareNode();

    void initialize_components();

private:
    // === 回调函数声明 ===
    // ... (原有回调声明不变)
    void hardware_regulator_operation_command_callback(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg);
    void hardware_clear_alarm_callback(const std_msgs::msg::Empty::SharedPtr msg);
    void set_hardware_regulator_settings_callback(const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request> request, std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Response> response);
    void set_hardware_circuit_settings_callback(const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitSettings::Request> request, std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitSettings::Response> response);
    void hardware_regulator_breaker_command_callback(const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request> request, std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Response> response);
    void hardware_circuit_breaker_command_callback(const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request> request, std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Response> response);
    void hardware_set_control_mode_callback(const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitControlMode::Request> request, std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitControlMode::Response> response);
    void hardware_set_control_source_callback(const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitControlSource::Request> request, std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitControlSource::Response> response);
    void poll_hardware_data();

    // [新增] 优雅的节流辅助函数
    template <typename ResponseT>
    bool is_request_throttled(const std::string& service_name, std::shared_ptr<ResponseT> response,
                              const std::chrono::milliseconds& throttle_duration = 100ms)
    {
        std::lock_guard<std::mutex> lock(service_throttle_mutex_);
        auto now = std::chrono::steady_clock::now();

        if (service_call_timestamps_.count(service_name))
        {
            auto last_call = service_call_timestamps_[service_name];
            if (now - last_call < throttle_duration)
            {
                response->success = false;
                response->message = "Too many requests. Please try again shortly.";
                RCLCPP_WARN(this->get_logger(), "Service '%s' call rejected due to rate limit.", service_name.c_str());
                return true; // 被节流
            }
        }

        service_call_timestamps_[service_name] = now;
        return false; // 未被节流
    }

    // === ROS 句柄 (API层) ===
    // ... (原有句柄不变)
    rclcpp::Publisher<ros2_interfaces::msg::HardwareCircuitStatus>::SharedPtr hardware_circuit_status_pub_;
    rclcpp::Publisher<ros2_interfaces::msg::RegulatorStatus>::SharedPtr hardware_regulator_status_pub_;
    rclcpp::Publisher<ros2_interfaces::msg::HardwareCircuitSettings>::SharedPtr hardware_circuit_settings_pub_;
    rclcpp::Publisher<ros2_interfaces::msg::RegulatorSettings>::SharedPtr hardware_regulator_settings_pub_;
    rclcpp::Subscription<ros2_interfaces::msg::RegulatorOperationCommand>::SharedPtr hardware_regulator_operation_command_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr hardware_clear_alarm_sub_;
    rclcpp::Service<ros2_interfaces::srv::SetRegulatorSettings>::SharedPtr set_hardware_regulator_settings_service_;
    rclcpp::Service<ros2_interfaces::srv::SetHardwareCircuitSettings>::SharedPtr set_hardware_circuit_settings_service_;
    rclcpp::Service<ros2_interfaces::srv::RegulatorBreakerCommand>::SharedPtr hardware_regulator_breaker_command_service_;
    rclcpp::Service<ros2_interfaces::srv::CircuitBreakerCommand>::SharedPtr hardware_circuit_breaker_command_service_;
    rclcpp::Service<ros2_interfaces::srv::SetHardwareCircuitControlMode>::SharedPtr hardware_set_control_mode_service_;
    rclcpp::Service<ros2_interfaces::srv::SetHardwareCircuitControlSource>::SharedPtr hardware_set_control_source_service_;
    rclcpp::TimerBase::SharedPtr hardware_poll_timer_;

    // === 核心组件实例 ===
    std::unique_ptr<IHardwareDriver> hardware_driver_;

    // [新增] 服务节流所需成员
    std::mutex service_throttle_mutex_;
    std::map<std::string, std::chrono::steady_clock::time_point> service_call_timestamps_;
};

#endif // HARDWARE_NODE_HPP_
