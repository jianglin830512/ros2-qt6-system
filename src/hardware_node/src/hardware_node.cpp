#include "hardware_node/hardware_node.hpp"
#include "hardware_node/hardware_node_constants.hpp"
#include "hardware_node/tcp_hardware_driver.hpp"
#include "hardware_node/mock_hardware_driver.hpp"
#include <functional>
#include <future>
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
HardwareNode::HardwareNode() : Node("hardware_node")
{
    RCLCPP_INFO(this->get_logger(), "HardwareNode is starting up.");
}
HardwareNode::~HardwareNode()
{
    RCLCPP_INFO(this->get_logger(), "HardwareNode is shutting down.");
}
void HardwareNode::initialize_components()
{
    RCLCPP_INFO(this->get_logger(), "Initializing HardwareNode components...");

    // --- 实例化驱动 ---
    bool use_mock = this->declare_parameter<bool>(hardware_node_constants::USE_MOCK_DRIVER, true);
    if (use_mock)
    {
        hardware_driver_ = std::make_unique<MockHardwareDriver>(this->get_logger());
        RCLCPP_INFO(this->get_logger(), "Using MockHardwareDriver.");
    }
    else
    {
        hardware_driver_ = std::make_unique<TcpHardwareDriver>(this->get_logger());
        RCLCPP_WARN(this->get_logger(), "Using TcpHardwareDriver. This assumes a connection to physical hardware.");
    }

    // --- 初始化 Publishers ---
    auto circuit_status_topic = this->declare_parameter<std::string>(
        hardware_node_constants::HARDWARE_CIRCUIT_STATUS_TOPIC_PARAM,
        hardware_node_constants::DEFAULT_HARDWARE_CIRCUIT_STATUS_TOPIC);
    hardware_circuit_status_pub_ = this->create_publisher<ros2_interfaces::msg::HardwareCircuitStatus>(circuit_status_topic, 10);

    auto regulator_status_topic = this->declare_parameter<std::string>(
        hardware_node_constants::HARDWARE_REGULATOR_STATUS_TOPIC_PARAM,
        hardware_node_constants::DEFAULT_HARDWARE_REGULATOR_STATUS_TOPIC);
    hardware_regulator_status_pub_ = this->create_publisher<ros2_interfaces::msg::RegulatorStatus>(regulator_status_topic, 10);

    auto circuit_settings_topic = this->declare_parameter<std::string>(
        hardware_node_constants::HARDWARE_CIRCUIT_SETTINGS_TOPIC_PARAM,
        hardware_node_constants::DEFAULT_HARDWARE_CIRCUIT_SETTINGS_TOPIC);
    hardware_circuit_settings_pub_ = this->create_publisher<ros2_interfaces::msg::HardwareCircuitSettings>(circuit_settings_topic, 10);

    auto regulator_settings_topic = this->declare_parameter<std::string>(
        hardware_node_constants::HARDWARE_REGULATOR_SETTINGS_TOPIC_PARAM,
        hardware_node_constants::DEFAULT_HARDWARE_REGULATOR_SETTINGS_TOPIC);
    hardware_regulator_settings_pub_ = this->create_publisher<ros2_interfaces::msg::RegulatorSettings>(regulator_settings_topic, 10);

    // --- 初始化 Subscribers ---
    auto regulator_operation_command_topic = this->declare_parameter<std::string>(
        hardware_node_constants::HARDWARE_REGULATOR_OPERATION_COMMAND_TOPIC_PARAM,
        hardware_node_constants::DEFAULT_HARDWARE_REGULATOR_OPERATION_COMMAND_TOPIC);
    hardware_regulator_operation_command_sub_ = this->create_subscription<ros2_interfaces::msg::RegulatorOperationCommand>(
        regulator_operation_command_topic, 10, std::bind(&HardwareNode::hardware_regulator_operation_command_callback, this, _1));

    auto clear_alarm_topic = this->declare_parameter<std::string>(
        hardware_node_constants::HARDWARE_CLEAR_ALARM_TOPIC_PARAM,
        hardware_node_constants::DEFAULT_HARDWARE_CLEAR_ALARM_TOPIC);
    hardware_clear_alarm_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        clear_alarm_topic, 10, std::bind(&HardwareNode::hardware_clear_alarm_callback, this, _1));

    // --- 初始化 Services ---
    // 1. Settings
    auto set_regulator_settings_service = this->declare_parameter<std::string>(
        hardware_node_constants::SET_HARDWARE_REGULATOR_SETTINGS_SERVICE_PARAM,
        hardware_node_constants::DEFAULT_SET_HARDWARE_REGULATOR_SETTINGS_SERVICE);
    set_hardware_regulator_settings_service_ = this->create_service<ros2_interfaces::srv::SetRegulatorSettings>(
        set_regulator_settings_service, std::bind(&HardwareNode::set_hardware_regulator_settings_callback, this, _1, _2));

    auto set_circuit_settings_service = this->declare_parameter<std::string>(
        hardware_node_constants::SET_HARDWARE_CIRCUIT_SETTINGS_SERVICE_PARAM,
        hardware_node_constants::DEFAULT_SET_HARDWARE_CIRCUIT_SETTINGS_SERVICE);
    set_hardware_circuit_settings_service_ = this->create_service<ros2_interfaces::srv::SetHardwareCircuitSettings>(
        set_circuit_settings_service, std::bind(&HardwareNode::set_hardware_circuit_settings_callback, this, _1, _2));

    // 2. Commands
    auto regulator_breaker_command_service = this->declare_parameter<std::string>(
        hardware_node_constants::HARDWARE_REGULATOR_BREAKER_COMMAND_SERVICE_PARAM,
        hardware_node_constants::DEFAULT_HARDWARE_REGULATOR_BREAKER_COMMAND_SERVICE);
    hardware_regulator_breaker_command_service_ = this->create_service<ros2_interfaces::srv::RegulatorBreakerCommand>(
        regulator_breaker_command_service, std::bind(&HardwareNode::hardware_regulator_breaker_command_callback, this, _1, _2));

    auto circuit_breaker_command_service = this->declare_parameter<std::string>(
        hardware_node_constants::HARDWARE_CIRCUIT_BREAKER_COMMAND_SERVICE_PARAM,
        hardware_node_constants::DEFAULT_HARDWARE_CIRCUIT_BREAKER_COMMAND_SERVICE);
    hardware_circuit_breaker_command_service_ = this->create_service<ros2_interfaces::srv::CircuitBreakerCommand>(
        circuit_breaker_command_service, std::bind(&HardwareNode::hardware_circuit_breaker_command_callback, this, _1, _2));

    // 3. PLC Mode/Source (New)
    auto set_mode_service = this->declare_parameter<std::string>(
        hardware_node_constants::HARDWARE_SET_CONTROL_MODE_SERVICE_PARAM,
        hardware_node_constants::DEFAULT_HARDWARE_SET_CONTROL_MODE_SERVICE);
    hardware_set_control_mode_service_ = this->create_service<ros2_interfaces::srv::SetHardwareCircuitControlMode>(
        set_mode_service, std::bind(&HardwareNode::hardware_set_control_mode_callback, this, _1, _2));

    auto set_source_service = this->declare_parameter<std::string>(
        hardware_node_constants::HARDWARE_SET_CONTROL_SOURCE_SERVICE_PARAM,
        hardware_node_constants::DEFAULT_HARDWARE_SET_CONTROL_SOURCE_SERVICE);
    hardware_set_control_source_service_ = this->create_service<ros2_interfaces::srv::SetHardwareCircuitControlSource>(
        set_source_service, std::bind(&HardwareNode::hardware_set_control_source_callback, this, _1, _2));


    // --- 初始化 Timer ---
    int polling_rate_ms = this->declare_parameter<int>(
        hardware_node_constants::POLLING_RATE_MS_PARAM, 200);
    hardware_poll_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(polling_rate_ms),
        std::bind(&HardwareNode::poll_hardware_data, this));

    RCLCPP_INFO(this->get_logger(), "HardwareNode initialization complete. System is running.");
}
// --- Topic Callbacks ---
void HardwareNode::hardware_regulator_operation_command_callback(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received regulator operation command for ID %u", msg->regulator_id);
    hardware_driver_->handle_regulator_operation_command(msg);
}
void HardwareNode::hardware_clear_alarm_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    (void)msg;
    RCLCPP_INFO(this->get_logger(), "Received clear alarm command.");
    hardware_driver_->handle_clear_alarm();
}
// --- Service Callbacks: Settings ---
void HardwareNode::set_hardware_regulator_settings_callback(
    const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request> request,
    std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Response> response)
{
    if (is_request_throttled("set_regulator_settings", response)) {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "SetRegulatorSettings service called for ID %u.", request->settings.regulator_id);
    auto promise = std::make_shared<std::promise<void>>();
    auto future = promise->get_future();
    hardware_driver_->handle_set_hardware_regulator_settings_request(request, [response, promise](bool success, const std::string& message) {
        response->success = success;
        response->message = message;
        promise->set_value();
    });
    if (future.wait_for(3s) == std::future_status::timeout) {
        response->success = false;
        response->message = "Timeout waiting for driver.";
    }
}

void HardwareNode::set_hardware_circuit_settings_callback(
    const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitSettings::Request> request,
    std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitSettings::Response> response)
{
    if (is_request_throttled("set_circuit_settings", response)) {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "SetCircuitSettings service called for ID %u.", request->settings.circuit_id);
    auto promise = std::make_shared<std::promise<void>>();
    auto future = promise->get_future();
    hardware_driver_->handle_set_hardware_circuit_settings_request(request, [response, promise](bool success, const std::string& message) {
        response->success = success;
        response->message = message;
        promise->set_value();
    });
    if (future.wait_for(3s) == std::future_status::timeout) {
        response->success = false;
        response->message = "Timeout waiting for driver.";
    }
}

void HardwareNode::hardware_regulator_breaker_command_callback(
    const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request> request,
    std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Response> response)
{
    if (is_request_throttled("regulator_breaker_command", response)) {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "RegulatorBreakerCommand service called for ID %u.", request->regulator_id);
    auto promise = std::make_shared<std::promise<void>>();
    auto future = promise->get_future();
    hardware_driver_->handle_regulator_breaker_command(request, [response, promise](bool success, const std::string& message) {
        response->success = success;
        response->message = message;
        promise->set_value();
    });
    if (future.wait_for(3s) == std::future_status::timeout) {
        response->success = false;
        response->message = "Timeout waiting for driver.";
    }
}

void HardwareNode::hardware_circuit_breaker_command_callback(
    const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request> request,
    std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Response> response)
{
    if (is_request_throttled("circuit_breaker_command", response)) {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "CircuitBreakerCommand service called for ID %u.", request->circuit_id);
    auto promise = std::make_shared<std::promise<void>>();
    auto future = promise->get_future();
    hardware_driver_->handle_circuit_breaker_command(request, [response, promise](bool success, const std::string& message) {
        response->success = success;
        response->message = message;
        promise->set_value();
    });
    if (future.wait_for(3s) == std::future_status::timeout) {
        response->success = false;
        response->message = "Timeout waiting for driver.";
    }
}

void HardwareNode::hardware_set_control_mode_callback(
    const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitControlMode::Request> request,
    std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitControlMode::Response> response)
{
    if (is_request_throttled("set_control_mode", response)) {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "SetControlMode service called for Circuit %u, Mode %u.", request->circuit_id, request->mode);
    auto promise = std::make_shared<std::promise<void>>();
    auto future = promise->get_future();
    hardware_driver_->handle_set_control_mode(request, [response, promise](bool success, const std::string& message) {
        response->success = success;
        response->message = message;
        promise->set_value();
    });
    if (future.wait_for(3s) == std::future_status::timeout) {
        response->success = false;
        response->message = "Timeout waiting for driver.";
    }
}

void HardwareNode::hardware_set_control_source_callback(
    const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitControlSource::Request> request,
    std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitControlSource::Response> response)
{
    if (is_request_throttled("set_control_source", response)) {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "SetControlSource service called for Circuit %u, Source %u.", request->circuit_id, request->source);
    auto promise = std::make_shared<std::promise<void>>();
    auto future = promise->get_future();
    hardware_driver_->handle_set_control_source(request, [response, promise](bool success, const std::string& message) {
        response->success = success;
        response->message = message;
        promise->set_value();
    });
    if (future.wait_for(3s) == std::future_status::timeout) {
        response->success = false;
        response->message = "Timeout waiting for driver.";
    }
}
void HardwareNode::poll_hardware_data()
{
    // Rclcpp logging omitted for high frequency polling unless debug
    // 假设我们有 ID 为 1 和 2 的两个回路和两个调压器
    for (uint8_t id = 1; id <= 2; ++id)
    {
        // 轮询并发布调压器数据
        ros2_interfaces::msg::RegulatorStatus regulator_status;
        if (hardware_driver_->get_regulator_status(id, regulator_status)) {
            regulator_status.header.stamp = this->get_clock()->now();
            hardware_regulator_status_pub_->publish(regulator_status);
        }

        ros2_interfaces::msg::RegulatorSettings regulator_settings;
        if (hardware_driver_->get_regulator_settings(id, regulator_settings)) {
            hardware_regulator_settings_pub_->publish(regulator_settings);
        }

        // 轮询并发布回路数据
        ros2_interfaces::msg::HardwareCircuitStatus circuit_status;
        if (hardware_driver_->get_circuit_status(id, circuit_status)) {
            circuit_status.header.stamp = this->get_clock()->now();
            hardware_circuit_status_pub_->publish(circuit_status);
        }

        ros2_interfaces::msg::HardwareCircuitSettings circuit_settings;
        if (hardware_driver_->get_circuit_settings(id, circuit_settings)) {
            hardware_circuit_settings_pub_->publish(circuit_settings);
        }
    }
}
