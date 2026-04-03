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

    bool use_mock = this->declare_parameter<bool>(hardware_node_constants::USE_MOCK_DRIVER, false);
    if (use_mock)
    {
        hardware_driver_ = std::make_unique<MockHardwareDriver>(this->get_logger());
        RCLCPP_INFO(this->get_logger(), "Using MockHardwareDriver.");
    }
    else
    {
        std::string plc_ip = this->declare_parameter<std::string>(
            hardware_node_constants::PLC_IP_ADDRESS_PARAM, "192.168.2.100");
        int plc_port = this->declare_parameter<int>(
            hardware_node_constants::PLC_PORT_PARAM, 502);

        std::string temp_ip = this->declare_parameter<std::string>(
            hardware_node_constants::TEMP_MONITOR_IP_ADDRESS_PARAM, "192.168.2.95");
        int temp_port = this->declare_parameter<int>(
            hardware_node_constants::TEMP_MONITOR_PORT_PARAM, 3000);

        hardware_driver_ = std::make_unique<TcpHardwareDriver>(
            this->get_logger(), plc_ip, plc_port, temp_ip, temp_port);

        RCLCPP_WARN(this->get_logger(), "Using TcpHardwareDriver. PLC[%s:%d], TempMon[%s:%d]",
                    plc_ip.c_str(), plc_port, temp_ip.c_str(), temp_port);
    }

    // 定时器任务自身不能重入，保持互斥
    timer_update_cb_group_       = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_pub_cb_group_          = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Sub 和 Srv 允许并发重入，由底层的 mutex 和节流逻辑保证安全
    sub_cb_group_                = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    srv_reg_settings_cb_group_   = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    srv_circ_settings_cb_group_  = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    srv_reg_breaker_cb_group_    = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    srv_circ_breaker_cb_group_   = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    srv_mode_cb_group_           = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // --- 初始化 Publishers ---
    auto circuit_status_topic = this->declare_parameter<std::string>(hardware_node_constants::HARDWARE_CIRCUIT_STATUS_TOPIC_PARAM, hardware_node_constants::DEFAULT_HARDWARE_CIRCUIT_STATUS_TOPIC);
    hardware_circuit_status_pub_ = this->create_publisher<ros2_interfaces::msg::HardwareCircuitStatus>(circuit_status_topic, 10);

    auto regulator_status_topic = this->declare_parameter<std::string>(hardware_node_constants::HARDWARE_REGULATOR_STATUS_TOPIC_PARAM, hardware_node_constants::DEFAULT_HARDWARE_REGULATOR_STATUS_TOPIC);
    hardware_regulator_status_pub_ = this->create_publisher<ros2_interfaces::msg::RegulatorStatus>(regulator_status_topic, 10);

    auto circuit_settings_topic = this->declare_parameter<std::string>(hardware_node_constants::HARDWARE_CIRCUIT_SETTINGS_TOPIC_PARAM, hardware_node_constants::DEFAULT_HARDWARE_CIRCUIT_SETTINGS_TOPIC);
    hardware_circuit_settings_pub_ = this->create_publisher<ros2_interfaces::msg::HardwareCircuitSettings>(circuit_settings_topic, 10);

    auto regulator_settings_topic = this->declare_parameter<std::string>(hardware_node_constants::HARDWARE_REGULATOR_SETTINGS_TOPIC_PARAM, hardware_node_constants::DEFAULT_HARDWARE_REGULATOR_SETTINGS_TOPIC);
    hardware_regulator_settings_pub_ = this->create_publisher<ros2_interfaces::msg::RegulatorSettings>(regulator_settings_topic, 10);

    auto system_status_topic = this->declare_parameter<std::string>(hardware_node_constants::HARDWARE_SYSTEM_STATUS_TOPIC_PARAM, hardware_node_constants::DEFAULT_HARDWARE_SYSTEM_STATUS_TOPIC);
    hardware_system_status_pub_ = this->create_publisher<ros2_interfaces::msg::HardwareSystemStatus>(system_status_topic, 10);

    // --- 初始化 Subscribers ---
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = sub_cb_group_;

    auto regulator_operation_command_topic = this->declare_parameter<std::string>(hardware_node_constants::HARDWARE_REGULATOR_OPERATION_COMMAND_TOPIC_PARAM, hardware_node_constants::DEFAULT_HARDWARE_REGULATOR_OPERATION_COMMAND_TOPIC);
    hardware_regulator_operation_command_sub_ = this->create_subscription<ros2_interfaces::msg::RegulatorOperationCommand>(
        regulator_operation_command_topic, 10, std::bind(&HardwareNode::hardware_regulator_operation_command_callback, this, _1), sub_options);

    auto clear_alarm_topic = this->declare_parameter<std::string>(hardware_node_constants::HARDWARE_CLEAR_ALARM_TOPIC_PARAM, hardware_node_constants::DEFAULT_HARDWARE_CLEAR_ALARM_TOPIC);
    hardware_clear_alarm_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        clear_alarm_topic, 10, std::bind(&HardwareNode::hardware_clear_alarm_callback, this, _1), sub_options);

    // --- 初始化 Services ---
    auto set_regulator_settings_service = this->declare_parameter<std::string>(hardware_node_constants::SET_HARDWARE_REGULATOR_SETTINGS_SERVICE_PARAM, hardware_node_constants::DEFAULT_SET_HARDWARE_REGULATOR_SETTINGS_SERVICE);
    set_hardware_regulator_settings_service_ = this->create_service<ros2_interfaces::srv::SetRegulatorSettings>(
        set_regulator_settings_service, std::bind(&HardwareNode::set_hardware_regulator_settings_callback, this, _1, _2), rmw_qos_profile_services_default, srv_reg_settings_cb_group_);

    auto set_circuit_settings_service = this->declare_parameter<std::string>(hardware_node_constants::SET_HARDWARE_CIRCUIT_SETTINGS_SERVICE_PARAM, hardware_node_constants::DEFAULT_SET_HARDWARE_CIRCUIT_SETTINGS_SERVICE);
    set_hardware_circuit_settings_service_ = this->create_service<ros2_interfaces::srv::SetHardwareCircuitSettings>(
        set_circuit_settings_service, std::bind(&HardwareNode::set_hardware_circuit_settings_callback, this, _1, _2), rmw_qos_profile_services_default, srv_circ_settings_cb_group_);

    auto regulator_breaker_command_service = this->declare_parameter<std::string>(hardware_node_constants::HARDWARE_REGULATOR_BREAKER_COMMAND_SERVICE_PARAM, hardware_node_constants::DEFAULT_HARDWARE_REGULATOR_BREAKER_COMMAND_SERVICE);
    hardware_regulator_breaker_command_service_ = this->create_service<ros2_interfaces::srv::RegulatorBreakerCommand>(
        regulator_breaker_command_service, std::bind(&HardwareNode::hardware_regulator_breaker_command_callback, this, _1, _2), rmw_qos_profile_services_default, srv_reg_breaker_cb_group_);

    auto circuit_breaker_command_service = this->declare_parameter<std::string>(hardware_node_constants::HARDWARE_CIRCUIT_BREAKER_COMMAND_SERVICE_PARAM, hardware_node_constants::DEFAULT_HARDWARE_CIRCUIT_BREAKER_COMMAND_SERVICE);
    hardware_circuit_breaker_command_service_ = this->create_service<ros2_interfaces::srv::CircuitBreakerCommand>(
        circuit_breaker_command_service, std::bind(&HardwareNode::hardware_circuit_breaker_command_callback, this, _1, _2), rmw_qos_profile_services_default, srv_circ_breaker_cb_group_);

    auto set_mode_service = this->declare_parameter<std::string>(hardware_node_constants::HARDWARE_SET_CONTROL_MODE_SERVICE_PARAM, hardware_node_constants::DEFAULT_HARDWARE_SET_CONTROL_MODE_SERVICE);
    hardware_set_control_mode_service_ = this->create_service<ros2_interfaces::srv::SetHardwareCircuitControlMode>(
        set_mode_service, std::bind(&HardwareNode::hardware_set_control_mode_callback, this, _1, _2), rmw_qos_profile_services_default, srv_mode_cb_group_);

    // --- 初始化 Timers ---
    int polling_rate_ms = this->declare_parameter<int>(hardware_node_constants::POLLING_RATE_MS_PARAM, 200);

    // Timer 1: 负责底层可能阻塞的 TCP 抓取 (独立线程执行)
    hardware_update_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(polling_rate_ms),
        std::bind(&HardwareNode::update_hardware_data, this), timer_update_cb_group_);

    // Timer 2: 负责从缓存读取数据并极速发布 (独立线程执行，100ms保证心跳永不断)
    hardware_publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&HardwareNode::publish_hardware_data, this), timer_pub_cb_group_);

    RCLCPP_INFO(this->get_logger(), "HardwareNode initialization complete. System is running.");
}

// --- Topic Callbacks ---
void HardwareNode::hardware_regulator_operation_command_callback(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg) {
    hardware_driver_->handle_regulator_operation_command(msg);
}
void HardwareNode::hardware_clear_alarm_callback(const std_msgs::msg::Empty::SharedPtr msg) {
    (void)msg;
    hardware_driver_->handle_clear_alarm();
}

// --- Service Callbacks: 折中设置超时为 2s，兼容 TCP 排队场景 ---
void HardwareNode::set_hardware_regulator_settings_callback(const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request> request, std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Response> response) {
    std::string key = "set_regulator_settings_" + std::to_string(request->settings.regulator_id);
    if (is_request_throttled(key, response)) { return; }
    auto promise = std::make_shared<std::promise<void>>();
    auto future = promise->get_future();
    hardware_driver_->handle_set_hardware_regulator_settings_request(request,[response, promise](bool success, const std::string& message) {
        response->success = success; response->message = message; promise->set_value();
    });
    if (future.wait_for(2s) == std::future_status::timeout) { // 改为2s，防止网络瞬时排队导致误报
        response->success = false; response->message = "Timeout waiting for driver.";
    }
}

void HardwareNode::set_hardware_circuit_settings_callback(const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitSettings::Request> request, std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitSettings::Response> response) {
    std::string key = "set_circuit_settings_" + std::to_string(request->settings.circuit_id);
    if (is_request_throttled(key, response)) { return; }
    auto promise = std::make_shared<std::promise<void>>();
    auto future = promise->get_future();
    hardware_driver_->handle_set_hardware_circuit_settings_request(request,[response, promise](bool success, const std::string& message) {
        response->success = success; response->message = message; promise->set_value();
    });
    if (future.wait_for(2s) == std::future_status::timeout) {
        response->success = false; response->message = "Timeout waiting for driver.";
    }
}

void HardwareNode::hardware_regulator_breaker_command_callback(const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request> request, std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Response> response) {
    std::string key = "regulator_breaker_" + std::to_string(request->regulator_id);
    if (is_request_throttled(key, response)) { return; }
    auto promise = std::make_shared<std::promise<void>>();
    auto future = promise->get_future();
    hardware_driver_->handle_regulator_breaker_command(request,[response, promise](bool success, const std::string& message) {
        response->success = success; response->message = message; promise->set_value();
    });
    if (future.wait_for(2s) == std::future_status::timeout) {
        response->success = false; response->message = "Timeout waiting for driver.";
    }
}

void HardwareNode::hardware_circuit_breaker_command_callback(const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request> request, std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Response> response) {
    std::string key = "circuit_breaker_" + std::to_string(request->circuit_id);
    if (is_request_throttled(key, response)) { return; }
    auto promise = std::make_shared<std::promise<void>>();
    auto future = promise->get_future();
    hardware_driver_->handle_circuit_breaker_command(request, [response, promise](bool success, const std::string& message) {
        response->success = success; response->message = message; promise->set_value();
    });
    if (future.wait_for(2s) == std::future_status::timeout) {
        response->success = false; response->message = "Timeout waiting for driver.";
    }
}

void HardwareNode::hardware_set_control_mode_callback(const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitControlMode::Request> request, std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitControlMode::Response> response) {
    std::string key = "set_mode_c" + std::to_string(request->circuit_id) + "_l" + std::to_string(request->loop_type);
    if (is_request_throttled(key, response)) { return; }
    auto promise = std::make_shared<std::promise<void>>();
    auto future = promise->get_future();
    hardware_driver_->handle_set_control_mode(request, [response, promise](bool success, const std::string& message) {
        response->success = success; response->message = message; promise->set_value();
    });
    if (future.wait_for(2s) == std::future_status::timeout) {
        response->success = false; response->message = "Timeout waiting for driver.";
    }
}

// --- 数据抓取动作 (可能被阻塞) ---
void HardwareNode::update_hardware_data()
{
    hardware_driver_->update();
}

// --- 纯发布动作 (极速内存读取，不受TCP断线阻塞影响) ---
void HardwareNode::publish_hardware_data()
{
    ros2_interfaces::msg::HardwareSystemStatus system_status;
    if (hardware_driver_->get_system_status(system_status)) {
        system_status.header.stamp = this->get_clock()->now();
        hardware_system_status_pub_->publish(system_status);
    }

    if (!system_status.plc_connected) {
        return; // PLC 断线时，只保活系统状态，不发回路状态
    }

    for (uint8_t id = 1; id <= 2; ++id)
    {
        ros2_interfaces::msg::RegulatorStatus regulator_status;
        if (hardware_driver_->get_regulator_status(id, regulator_status)) {
            regulator_status.header.stamp = this->get_clock()->now();
            hardware_regulator_status_pub_->publish(regulator_status);
        }

        ros2_interfaces::msg::RegulatorSettings regulator_settings;
        if (hardware_driver_->get_regulator_settings(id, regulator_settings)) {
            hardware_regulator_settings_pub_->publish(regulator_settings);
        }

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
