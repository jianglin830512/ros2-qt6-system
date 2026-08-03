#include "control_node/control_node.hpp"
#include "control_node/control_node_constants.hpp"
#include "control_node/state_manager.hpp"
#include "control_node/hardware_coordinator.hpp"
#include "control_node/persistence_coordinator.hpp"
#include "control_node/control_logic.hpp"
#include <future>

using namespace std::chrono_literals;

ControlNode::ControlNode() : Node("control_node") {}

void ControlNode::initialize_components()
{
    RCLCPP_INFO(this->get_logger(), "Initializing Control Node Components...");

    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    server_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    state_manager_ = std::make_shared<StateManager>();
    hardware_coordinator_ = std::make_shared<HardwareCoordinator>(state_manager_.get(), shared_from_this());
    persistence_coordinator_ = std::make_shared<PersistenceCoordinator>(state_manager_.get(), shared_from_this(), client_cb_group_);
    control_logic_ = std::make_unique<ControlLogic>(state_manager_, hardware_coordinator_, persistence_coordinator_);

    // 声明并获取超时时间参数，传给 ControlLogic
    this->declare_parameter(control_node_constants::PLC_COMMAND_TIMEOUT_PARAM, control_node_constants::DEFAULT_PLC_COMMAND_TIMEOUT);
    double plc_cmd_timeout = this->get_parameter(control_node_constants::PLC_COMMAND_TIMEOUT_PARAM).as_double();
    control_logic_->set_command_timeout(plc_cmd_timeout);

    // ============================================================
    // [Publishers] (与 QT_NODE 通信的发布者)
    // ============================================================
    auto circuit_status_topic = this->declare_parameter<std::string>(
        control_node_constants::CIRCUIT_STATUS_TOPIC_PARAM, control_node_constants::DEFAULT_CIRCUIT_STATUS_TOPIC);
    RCLCPP_INFO(this->get_logger(), "Publishing Circuit Status to: '%s'", circuit_status_topic.c_str());
    circuit_status_pub_ = this->create_publisher<ros2_interfaces::msg::CircuitStatus>(circuit_status_topic, 10);

    auto regulator_status_topic = this->declare_parameter<std::string>(
        control_node_constants::REGULATOR_STATUS_TOPIC_PARAM, control_node_constants::DEFAULT_REGULATOR_STATUS_TOPIC);
    RCLCPP_INFO(this->get_logger(), "Publishing Regulator Status to: '%s'", regulator_status_topic.c_str());
    regulator_status_pub_ = this->create_publisher<ros2_interfaces::msg::RegulatorStatus>(regulator_status_topic, 10);

    auto system_status_topic = this->declare_parameter<std::string>(
        control_node_constants::SYSTEM_STATUS_TOPIC_PARAM, control_node_constants::DEFAULT_SYSTEM_STATUS_TOPIC);
    RCLCPP_INFO(this->get_logger(), "Publishing System Status to: '%s'", system_status_topic.c_str());
    system_status_pub_ = this->create_publisher<ros2_interfaces::msg::SystemStatus>(system_status_topic, 10);

    auto system_settings_topic = this->declare_parameter<std::string>(
        control_node_constants::SYSTEM_SETTINGS_TOPIC_PARAM, control_node_constants::DEFAULT_SYSTEM_SETTINGS_TOPIC);
    RCLCPP_INFO(this->get_logger(), "Publishing System Settings to: '%s'", system_settings_topic.c_str());
    system_settings_pub_ = this->create_publisher<ros2_interfaces::msg::SystemSettings>(system_settings_topic, rclcpp::QoS(10).transient_local());

    auto regulator_settings_topic = this->declare_parameter<std::string>(
        control_node_constants::REGULATOR_SETTINGS_TOPIC_PARAM, control_node_constants::DEFAULT_REGULATOR_SETTINGS_TOPIC);
    RCLCPP_INFO(this->get_logger(), "Publishing Regulator Settings to: '%s'", regulator_settings_topic.c_str());
    regulator_settings_pub_ = this->create_publisher<ros2_interfaces::msg::RegulatorSettings>(regulator_settings_topic, rclcpp::QoS(10).transient_local());

    auto circuit_settings_topic = this->declare_parameter<std::string>(
        control_node_constants::CIRCUIT_SETTINGS_TOPIC_PARAM, control_node_constants::DEFAULT_CIRCUIT_SETTINGS_TOPIC);
    RCLCPP_INFO(this->get_logger(), "Publishing Circuit Settings to: '%s'", circuit_settings_topic.c_str());
    circuit_settings_pub_ = this->create_publisher<ros2_interfaces::msg::CircuitSettings>(circuit_settings_topic, rclcpp::QoS(10).transient_local());

    // ============================================================
    // [Subscribers] (与 QT_NODE 通信的订阅者)
    // ============================================================
    auto reg_op_command_topic = this->declare_parameter<std::string>(
        control_node_constants::REGULATOR_OPERATION_COMMAND_TOPIC_PARAM, control_node_constants::DEFAULT_REGULATOR_OPERATION_COMMAND_TOPIC);
    RCLCPP_INFO(this->get_logger(), "Subscribing Regulator Operation Command on: '%s'", reg_op_command_topic.c_str());
    regulator_operation_command_sub_ = this->create_subscription<ros2_interfaces::msg::RegulatorOperationCommand>(
        reg_op_command_topic, 10, std::bind(&ControlNode::regulator_operation_command_callback, this, std::placeholders::_1));

    auto clear_alarm_topic = this->declare_parameter<std::string>(
        control_node_constants::CLEAR_ALARM_TOPIC_PARAM, control_node_constants::DEFAULT_CLEAR_ALARM_TOPIC);
    RCLCPP_INFO(this->get_logger(), "Subscribing Clear Alarm on: '%s'", clear_alarm_topic.c_str());
    clear_alarm_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        clear_alarm_topic, 10, std::bind(&ControlNode::clear_alarm_callback, this, std::placeholders::_1));

    // ============================================================
    // [Services] (与 QT_NODE 通信的服务提供方)
    // ============================================================
    auto set_system_settings_srv = this->declare_parameter<std::string>(
        control_node_constants::SET_SYSTEM_SETTINGS_SERVICE_PARAM, control_node_constants::DEFAULT_SET_SYSTEM_SETTINGS_SERVICE);
    RCLCPP_INFO(this->get_logger(), "Providing Set System Settings Service: '%s'", set_system_settings_srv.c_str());
    set_system_settings_service_ = this->create_service<ros2_interfaces::srv::SetSystemSettings>(
        set_system_settings_srv, std::bind(&ControlNode::set_system_settings_callback, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), server_cb_group_);

    auto set_regulator_settings_srv = this->declare_parameter<std::string>(
        control_node_constants::SET_REGULATOR_SETTINGS_SERVICE_PARAM, control_node_constants::DEFAULT_SET_REGULATOR_SETTINGS_SERVICE);
    RCLCPP_INFO(this->get_logger(), "Providing Set Regulator Settings Service: '%s'", set_regulator_settings_srv.c_str());
    set_regulator_settings_service_ = this->create_service<ros2_interfaces::srv::SetRegulatorSettings>(
        set_regulator_settings_srv, std::bind(&ControlNode::set_regulator_settings_callback, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), server_cb_group_);

    auto set_circuit_settings_srv = this->declare_parameter<std::string>(
        control_node_constants::SET_CIRCUIT_SETTINGS_SERVICE_PARAM, control_node_constants::DEFAULT_SET_CIRCUIT_SETTINGS_SERVICE);
    RCLCPP_INFO(this->get_logger(), "Providing Set Circuit Settings Service: '%s'", set_circuit_settings_srv.c_str());
    set_circuit_settings_service_ = this->create_service<ros2_interfaces::srv::SetCircuitSettings>(
        set_circuit_settings_srv, std::bind(&ControlNode::set_circuit_settings_callback, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), server_cb_group_);

    auto reg_breaker_command_srv = this->declare_parameter<std::string>(
        control_node_constants::REGULATOR_BREAKER_COMMAND_SERVICE_PARAM, control_node_constants::DEFAULT_REGULATOR_BREAKER_COMMAND_SERVICE);
    RCLCPP_INFO(this->get_logger(), "Providing Regulator Breaker Command Service: '%s'", reg_breaker_command_srv.c_str());
    regulator_breaker_command_service_ = this->create_service<ros2_interfaces::srv::RegulatorBreakerCommand>(
        reg_breaker_command_srv, std::bind(&ControlNode::regulator_breaker_command_callback, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), server_cb_group_);

    auto circuit_breaker_command_srv = this->declare_parameter<std::string>(
        control_node_constants::CIRCUIT_BREAKER_COMMAND_SERVICE_PARAM, control_node_constants::DEFAULT_CIRCUIT_BREAKER_COMMAND_SERVICE);
    RCLCPP_INFO(this->get_logger(), "Providing Circuit Breaker Command Service: '%s'", circuit_breaker_command_srv.c_str());
    circuit_breaker_command_service_ = this->create_service<ros2_interfaces::srv::CircuitBreakerCommand>(
        circuit_breaker_command_srv, std::bind(&ControlNode::circuit_breaker_command_callback, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), server_cb_group_);

    // ============================================================
    // [Timers] (定时器)
    // ============================================================
    control_logic_timer_ = this->create_wall_timer(20ms, std::bind(&ControlLogic::update, control_logic_.get()));
    status_broadcast_timer_ = this->create_wall_timer(200ms, std::bind(&ControlNode::broadcast_status_callback, this));
    settings_broadcast_timer_ = this->create_wall_timer(1s, std::bind(&ControlNode::broadcast_settings_callback, this));
    lifecycle_check_timer_ = this->create_wall_timer(1s, std::bind(&ControlLogic::maintain_lifecycle, control_logic_.get()));

    RCLCPP_INFO(this->get_logger(), "Control Node Components Initialized Successfully.");
}

ControlNode::~ControlNode() {}

void ControlNode::regulator_operation_command_callback(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg) {
    control_logic_->process_regulator_operation_command(msg);
}

void ControlNode::clear_alarm_callback(const std_msgs::msg::Empty::SharedPtr) {
    control_logic_->process_clear_alarm();
}

void ControlNode::regulator_breaker_command_callback(const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request> request, std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Response> response) {
    control_logic_->handle_regulator_breaker_command_request(request, [response](bool success, const std::string& message) {
        response->success = success; response->message = message;
    });
}

void ControlNode::circuit_breaker_command_callback(const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request> request, std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Response> response) {
    control_logic_->handle_circuit_breaker_command_request(request, [response](bool success, const std::string& message) {
        response->success = success; response->message = message;
    });
}

void ControlNode::set_system_settings_callback(const std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Request> request, std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Response> response) {
    auto promise = std::make_shared<std::promise<void>>(); auto future = promise->get_future();
    control_logic_->handle_set_system_settings_request(request, [this, response, promise](bool success, const std::string& message) {
        response->success = success; response->message = message;
        if (success) system_settings_pub_->publish(state_manager_->get_system_settings());
        promise->set_value();
    });
    if (future.wait_for(8s) == std::future_status::timeout) response->success = false;
}

void ControlNode::set_regulator_settings_callback(const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request> request, std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Response> response) {
    auto promise = std::make_shared<std::promise<void>>(); auto future = promise->get_future();
    control_logic_->handle_set_regulator_settings_request(request, [this, request, response, promise](bool success, const std::string& message) {
        response->success = success; response->message = message;
        if (success) regulator_settings_pub_->publish(state_manager_->get_regulator_settings(request->settings.regulator_id));
        promise->set_value();
    });
    if (future.wait_for(8s) == std::future_status::timeout) response->success = false;
}

void ControlNode::set_circuit_settings_callback(const std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Request> request, std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Response> response) {
    auto promise = std::make_shared<std::promise<void>>(); auto future = promise->get_future();
    control_logic_->handle_set_circuit_settings_request(request, [this, request, response, promise](bool success, const std::string& message) {
        response->success = success; response->message = message;
        if (success) circuit_settings_pub_->publish(state_manager_->get_circuit_settings(request->settings.circuit_id));
        promise->set_value();
    });
    if (future.wait_for(8s) == std::future_status::timeout) response->success = false;
}

void ControlNode::broadcast_status_callback() {
    auto sys_status_msg = state_manager_->get_system_status();
    sys_status_msg.header.stamp = this->now();
    system_status_pub_->publish(sys_status_msg);

    for(uint8_t id = 1; id <= StateManager::NUM_CIRCUITS; ++id)
        circuit_status_pub_->publish(state_manager_->get_circuit_status(id));

    for(uint8_t id = 1; id <= StateManager::NUM_REGULATORS; ++id)
        regulator_status_pub_->publish(state_manager_->get_regulator_status(id));
}

void ControlNode::broadcast_settings_callback() {
    if (!control_logic_->is_settings_synced()) return;
    system_settings_pub_->publish(state_manager_->get_system_settings());
    for (uint8_t id = 1; id <= StateManager::NUM_REGULATORS; ++id)
        regulator_settings_pub_->publish(state_manager_->get_regulator_settings(id));
    for (uint8_t id = 1; id <= StateManager::NUM_CIRCUITS; ++id)
        circuit_settings_pub_->publish(state_manager_->get_circuit_settings(id));
}
