#include "control_node/hardware_coordinator.hpp"
#include "control_node/state_manager.hpp"
#include "control_node/control_node_constants.hpp"
using namespace std::chrono_literals;
HardwareCoordinator::HardwareCoordinator(StateManager* state_manager, rclcpp::Node::SharedPtr node)
    : state_manager_(state_manager), node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "Initializing Hardware Coordinator...");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    // === 1. 创建订阅器，接收来自 hardware_node 的数据 ===
    auto hw_circuit_status_topic = node_->declare_parameter<std::string>(
        control_node_constants::HARDWARE_CIRCUIT_STATUS_TOPIC_PARAM,
        control_node_constants::DEFAULT_HARDWARE_CIRCUIT_STATUS_TOPIC);
    RCLCPP_INFO(node_->get_logger(), "Subscribing to Hardware Circuit Status Topic: '%s'", hw_circuit_status_topic.c_str());
    hw_circuit_status_sub_ = node_->create_subscription<ros2_interfaces::msg::HardwareCircuitStatus>(
        hw_circuit_status_topic, qos, std::bind(&HardwareCoordinator::hardware_circuit_status_callback, this, std::placeholders::_1));

    auto hw_regulator_status_topic = node_->declare_parameter<std::string>(
        control_node_constants::HARDWARE_REGULATOR_STATUS_TOPIC_PARAM,
        control_node_constants::DEFAULT_HARDWARE_REGULATOR_STATUS_TOPIC);
    RCLCPP_INFO(node_->get_logger(), "Subscribing to Hardware Regulator Status Topic: '%s'", hw_regulator_status_topic.c_str());
    hw_regulator_status_sub_ = node_->create_subscription<ros2_interfaces::msg::RegulatorStatus>(
        hw_regulator_status_topic, qos, std::bind(&HardwareCoordinator::hardware_regulator_status_callback, this, std::placeholders::_1));

    auto hw_circuit_settings_topic = node_->declare_parameter<std::string>(
        control_node_constants::HARDWARE_CIRCUIT_SETTINGS_TOPIC_PARAM,
        control_node_constants::DEFAULT_HARDWARE_CIRCUIT_SETTINGS_TOPIC);
    RCLCPP_INFO(node_->get_logger(), "Subscribing to Hardware Circuit Settings Topic: '%s'", hw_circuit_settings_topic.c_str());
    hw_circuit_settings_sub_ = node_->create_subscription<ros2_interfaces::msg::HardwareCircuitSettings>(
        hw_circuit_settings_topic, qos, std::bind(&HardwareCoordinator::hardware_circuit_settings_callback, this, std::placeholders::_1));

    auto hw_regulator_settings_topic = node_->declare_parameter<std::string>(
        control_node_constants::HARDWARE_REGULATOR_SETTINGS_TOPIC_PARAM,
        control_node_constants::DEFAULT_HARDWARE_REGULATOR_SETTINGS_TOPIC);
    RCLCPP_INFO(node_->get_logger(), "Subscribing to Hardware Regulator Settings Topic: '%s'", hw_regulator_settings_topic.c_str());
    hw_regulator_settings_sub_ = node_->create_subscription<ros2_interfaces::msg::RegulatorSettings>(
        hw_regulator_settings_topic, qos, std::bind(&HardwareCoordinator::hardware_regulator_settings_callback, this, std::placeholders::_1));

    // === 2. 创建发布器，向 hardware_node 发送命令 ===
    auto hw_regulator_op_topic = node_->declare_parameter<std::string>(
        control_node_constants::HARDWARE_REGULATOR_OPERATION_COMMAND_TOPIC_PARAM,
        control_node_constants::DEFAULT_HARDWARE_REGULATOR_OPERATION_COMMAND_TOPIC);
    RCLCPP_INFO(node_->get_logger(), "Publishing to Hardware Regulator Operation Command Topic: '%s'", hw_regulator_op_topic.c_str());
    hw_regulator_operation_pub_ = node_->create_publisher<ros2_interfaces::msg::RegulatorOperationCommand>(hw_regulator_op_topic, qos);

    auto hw_clear_alarm_topic = node_->declare_parameter<std::string>(
        control_node_constants::HARDWARE_CLEAR_ALARM_TOPIC_PARAM,
        control_node_constants::DEFAULT_HARDWARE_CLEAR_ALARM_TOPIC);
    RCLCPP_INFO(node_->get_logger(), "Publishing to Hardware Clear Alarm Topic: '%s'", hw_clear_alarm_topic.c_str());
    hw_clear_alarm_pub_ = node_->create_publisher<std_msgs::msg::Empty>(hw_clear_alarm_topic, qos);

    // === 3. 创建服务客户端 ===
    // 3.1 向 hardware_node 发送设置
    auto set_hw_regulator_service = node_->declare_parameter<std::string>(
        control_node_constants::SET_HARDWARE_REGULATOR_SETTINGS_SERVICE_PARAM,
        control_node_constants::DEFAULT_SET_HARDWARE_REGULATOR_SETTINGS_SERVICE);
    RCLCPP_INFO(node_->get_logger(), "Using SetHardwareRegulatorSettings service: '%s'", set_hw_regulator_service.c_str());
    set_hw_regulator_settings_client_ = node_->create_client<ros2_interfaces::srv::SetRegulatorSettings>(set_hw_regulator_service);

    auto set_hw_circuit_service = node_->declare_parameter<std::string>(
        control_node_constants::SET_HARDWARE_CIRCUIT_SETTINGS_SERVICE_PARAM,
        control_node_constants::DEFAULT_SET_HARDWARE_CIRCUIT_SETTINGS_SERVICE);
    RCLCPP_INFO(node_->get_logger(), "Using SetHardwareCircuitSettings service: '%s'", set_hw_circuit_service.c_str());
    set_hw_circuit_settings_client_ = node_->create_client<ros2_interfaces::srv::SetHardwareCircuitSettings>(set_hw_circuit_service);

    // 3.2 向 hardware_node 发送命令
    auto hw_regulator_breaker_service = node_->declare_parameter<std::string>(
        control_node_constants::HARDWARE_REGULATOR_BREAKER_COMMAND_SERVICE_PARAM,
        control_node_constants::DEFAULT_HARDWARE_REGULATOR_BREAKER_COMMAND_SERVICE);
    RCLCPP_INFO(node_->get_logger(), "Using HardwareRegulatorBreakerCommand service: '%s'", hw_regulator_breaker_service.c_str());
    hw_regulator_breaker_client_ = node_->create_client<ros2_interfaces::srv::RegulatorBreakerCommand>(hw_regulator_breaker_service);

    auto hw_circuit_breaker_service = node_->declare_parameter<std::string>(
        control_node_constants::HARDWARE_CIRCUIT_BREAKER_COMMAND_SERVICE_PARAM,
        control_node_constants::DEFAULT_HARDWARE_CIRCUIT_BREAKER_COMMAND_SERVICE);
    RCLCPP_INFO(node_->get_logger(), "Using HardwareCircuitBreakerCommand service: '%s'", hw_circuit_breaker_service.c_str());
    hw_circuit_breaker_client_ = node_->create_client<ros2_interfaces::srv::CircuitBreakerCommand>(hw_circuit_breaker_service);

    // Set Control Mode Client
    auto hw_mode_service_name = node_->declare_parameter<std::string>(
        control_node_constants::HARDWARE_SET_CONTROL_MODE_SERVICE_PARAM,
        control_node_constants::DEFAULT_HARDWARE_SET_CONTROL_MODE_SERVICE);
    RCLCPP_INFO(node_->get_logger(), "Using Hardware SetControlMode service: '%s'", hw_mode_service_name.c_str());
    hw_set_control_mode_client_ = node_->create_client<ros2_interfaces::srv::SetHardwareCircuitControlMode>(
        hw_mode_service_name);

    // Set Control Source Client
    auto hw_source_service_name = node_->declare_parameter<std::string>(
        control_node_constants::HARDWARE_SET_CONTROL_SOURCE_SERVICE_PARAM,
        control_node_constants::DEFAULT_HARDWARE_SET_CONTROL_SOURCE_SERVICE);
    RCLCPP_INFO(node_->get_logger(), "Using Hardware SetControlSource service: '%s'", hw_source_service_name.c_str());
    hw_set_control_source_client_ = node_->create_client<ros2_interfaces::srv::SetHardwareCircuitControlSource>(
        hw_source_service_name);

    RCLCPP_INFO(node_->get_logger(), "Hardware Coordinator initialization complete.");
}
bool HardwareCoordinator::is_connected() const
{
    // 检查关键服务是否就绪。
    return set_hw_circuit_settings_client_->service_is_ready();
}
// --- 订阅回调实现: 收到硬件数据，直接更新 StateManager (Single Source of Truth) ---
void HardwareCoordinator::hardware_circuit_status_callback(const ros2_interfaces::msg::HardwareCircuitStatus::SharedPtr msg)
{
    // 将硬件状态合并到主 CircuitStatus 中
    state_manager_->update_circuit_status_from_hardware(msg->circuit_id, *msg);
    //RCLCPP_INFO(node_->get_logger(), "msg:\n%s", ros2_interfaces::msg::to_yaml(*msg).c_str());
}
void HardwareCoordinator::hardware_regulator_status_callback(const ros2_interfaces::msg::RegulatorStatus::SharedPtr msg)
{
    // 调压器状态是完整的 RegulatorStatus，直接更新即可
    // 确保ID正确
    state_manager_->update_regulator_status(msg->regulator_id, *msg);
}
void HardwareCoordinator::hardware_circuit_settings_callback(const ros2_interfaces::msg::HardwareCircuitSettings::SharedPtr msg)
{
    // 收到 Hardware Settings 广播，更新到 StateManager 的 CircuitSettings 中
    state_manager_->update_circuit_settings_from_hardware(msg->circuit_id, *msg);
}
void HardwareCoordinator::hardware_regulator_settings_callback(const ros2_interfaces::msg::RegulatorSettings::SharedPtr msg)
{
    // 收到 Hardware Settings 广播，直接更新 StateManager 的 RegulatorSettings
    state_manager_->update_regulator_settings(msg->regulator_id, *msg);
}
// --- 接口实现: 从上层接收指令，发送给硬件 ---
void HardwareCoordinator::apply_regulator_settings_to_hardware(
    uint8_t id,
    const ros2_interfaces::msg::RegulatorSettings& settings,
    HardwareCallback callback)
{
    if (!set_hw_regulator_settings_client_->service_is_ready()) {
        RCLCPP_WARN(node_->get_logger(), "Set hardware regulator settings service for ID %u is not available.", id);
        if (callback) {
            callback(false, "Service not available.");
        }
        return;
    }

    auto request = std::make_shared<ros2_interfaces::srv::SetRegulatorSettings::Request>();
    request->settings.regulator_id = id;
    request->settings = settings;

    auto response_callback = [this, callback, id](rclcpp::Client<ros2_interfaces::srv::SetRegulatorSettings>::SharedFuture future) {
        if (callback) {
            auto response = future.get();
            if (response) {
                callback(response->success, response->message);
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Failed to get response for SetRegulatorSettings service for ID %u.", id);
                callback(false, "Failed to get service response.");
            }
        }
    };

    set_hw_regulator_settings_client_->async_send_request(request, response_callback);
    RCLCPP_INFO(node_->get_logger(), "[HW Coordinator] Asynchronously applying regulator settings for ID %u.", id);
}
void HardwareCoordinator::apply_circuit_settings_to_hardware(
    uint8_t id,
    const ros2_interfaces::msg::HardwareCircuitSettings& settings,
    HardwareCallback callback)
{
    if (!set_hw_circuit_settings_client_->service_is_ready()) {
        RCLCPP_WARN(node_->get_logger(), "Set hardware circuit settings service for ID %u is not available.", id);
        if (callback) {
            callback(false, "Service not available.");
        }
        return;
    }

    auto request = std::make_shared<ros2_interfaces::srv::SetHardwareCircuitSettings::Request>();
    request->settings.circuit_id = id;
    request->settings = settings;

    auto response_callback = [this, callback, id](rclcpp::Client<ros2_interfaces::srv::SetHardwareCircuitSettings>::SharedFuture future) {
        if (callback) {
            auto response = future.get();
            if (response) {
                callback(response->success, response->message);
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Failed to get response for SetHardwareCircuitSettings service for ID %u.", id);
                callback(false, "Failed to get service response.");
            }
        }
    };

    set_hw_circuit_settings_client_->async_send_request(request, response_callback);
    RCLCPP_INFO(node_->get_logger(), "[HW Coordinator] Asynchronously applying circuit settings for ID %u.", id);
}
// 5个Command (2 TOPIC PUB , 3 SERVICE CLIENT)
void HardwareCoordinator::send_regulator_operation_command(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr command_msg)
{
    if (hw_regulator_operation_pub_->get_subscription_count() > 0) {
        hw_regulator_operation_pub_->publish(*command_msg);
        RCLCPP_DEBUG(node_->get_logger(), "[HW Coordinator] Published regulator operation command for ID %u: %u", command_msg->regulator_id, command_msg->command);
    } else {
        RCLCPP_WARN(node_->get_logger(), "[HW Coordinator] No subscribers for hardware regulator operation command topic. Command for ID %u: %u not sent.", command_msg->regulator_id, command_msg->command);
    }
}
void HardwareCoordinator::send_clear_alarm()
{
    if (hw_clear_alarm_pub_->get_subscription_count() > 0) {
        hw_clear_alarm_pub_->publish(std_msgs::msg::Empty());
        RCLCPP_DEBUG(node_->get_logger(), "[HW Coordinator] Published clear alarm command.");
    } else {
        RCLCPP_WARN(node_->get_logger(), "[HW Coordinator] No subscribers for hardware clear alarm topic. Command not sent.");
    }
}
void HardwareCoordinator::execute_regulator_breaker_command(
    const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request> request,
    HardwareCallback callback)
{
    if (!hw_regulator_breaker_client_->service_is_ready()) {
        RCLCPP_WARN(node_->get_logger(), "Hardware regulator breaker service for ID %u is not available.", request->regulator_id);
        if (callback) {
            callback(false, "Service not available.");
        }
        return;
    }

    auto response_callback = [this, callback, id = request->regulator_id](rclcpp::Client<ros2_interfaces::srv::RegulatorBreakerCommand>::SharedFuture future) {
        if (callback) {
            auto response = future.get();
            if (response) {
                callback(response->success, response->message);
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Failed to get response for Hardware Regulator Breaker Command for ID %u.", id);
                callback(false, "Failed to get service response.");
            }
        }
    };

    hw_regulator_breaker_client_->async_send_request(request, response_callback);
    RCLCPP_INFO(node_->get_logger(), "[HW Coordinator] Asynchronously executing regulator breaker command for ID %u.", request->regulator_id);
}
void HardwareCoordinator::execute_circuit_breaker_command(
    const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request> request,
    HardwareCallback callback)
{
    if (!hw_circuit_breaker_client_->service_is_ready()) {
        RCLCPP_WARN(node_->get_logger(), "Hardware circuit breaker service for ID %u is not available.", request->circuit_id);
        if (callback) {
            callback(false, "Service not available.");
        }
        return;
    }

    auto response_callback = [this, callback, id = request->circuit_id](rclcpp::Client<ros2_interfaces::srv::CircuitBreakerCommand>::SharedFuture future) {
        if (callback) {
            auto response = future.get();
            if (response) {
                callback(response->success, response->message);
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Failed to get response for Hardware Circuit Breaker Command for ID %u.", id);
                callback(false, "Failed to get service response.");
            }
        }
    };

    hw_circuit_breaker_client_->async_send_request(request, response_callback);
    RCLCPP_INFO(node_->get_logger(), "[HW Coordinator] Asynchronously executing circuit breaker command for ID %u.", request->circuit_id);
}
void HardwareCoordinator::set_circuit_control_mode(uint8_t circuit_id, uint8_t mode)
{
    if (!hw_set_control_mode_client_) return;

    if (!hw_set_control_mode_client_->service_is_ready()) {
        RCLCPP_WARN(node_->get_logger(),
                    "Hardware SetControlMode service not ready. Cannot set mode %d for circuit %d.", mode, circuit_id);
        return;
    }

    auto request = std::make_shared<ros2_interfaces::srv::SetHardwareCircuitControlMode::Request>();
    request->circuit_id = circuit_id;
    request->mode = mode;

    auto future_callback = [this, circuit_id, mode](rclcpp::Client<ros2_interfaces::srv::SetHardwareCircuitControlMode>::SharedFuture future) {
        try {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(node_->get_logger(), "HW-ACK: Circuit %d mode set to %d.", circuit_id, mode);
            } else {
                RCLCPP_ERROR(node_->get_logger(), "HW-NACK: Failed to set circuit %d mode: %s", circuit_id, response->message.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Exception setting circuit %d mode: %s", circuit_id, e.what());
        }
    };

    hw_set_control_mode_client_->async_send_request(request, future_callback);
}
void HardwareCoordinator::set_circuit_control_source(uint8_t circuit_id, uint8_t source)
{
    if (!hw_set_control_source_client_->service_is_ready()) {
        RCLCPP_WARN(node_->get_logger(),
                    "Hardware SetControlSource service not ready. Cannot set source %d for circuit %d.", source, circuit_id);
        return;
    }

    auto request = std::make_shared<ros2_interfaces::srv::SetHardwareCircuitControlSource::Request>();
    request->circuit_id = circuit_id;
    request->source = source;

    auto future_callback = [this, circuit_id, source](rclcpp::Client<ros2_interfaces::srv::SetHardwareCircuitControlSource>::SharedFuture future) {
        try {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(node_->get_logger(), "HW-ACK: Circuit %d source set to %d.", circuit_id, source);
            } else {
                RCLCPP_ERROR(node_->get_logger(), "HW-NACK: Failed to set circuit %d source: %s", circuit_id, response->message.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Exception setting circuit %d source: %s", circuit_id, e.what());
        }
    };

    hw_set_control_source_client_->async_send_request(request, future_callback);
}
