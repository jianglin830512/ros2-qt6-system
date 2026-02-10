#include "control_node/control_node.hpp"
#include "control_node/control_node_constants.hpp"
#include "control_node/state_manager.hpp"
#include "control_node/hardware_coordinator.hpp"
#include "control_node/persistence_coordinator.hpp"
#include "control_node/control_logic.hpp"
#include <future>

    using namespace std::chrono_literals;

ControlNode::ControlNode() : Node("control_node")
{
    RCLCPP_INFO(this->get_logger(), "ControlNode constructed. Call initializeComponents() to proceed.");
    // 构造函数现在只做最少的工作
}

void ControlNode::initialize_components()
{
    RCLCPP_INFO(this->get_logger(), "Initializing ControlNode with layered architecture...");

    // === 1. 创建回调组 ===
    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    server_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // === 2. 创建核心组件 (严格按照依赖顺序) ===
    state_manager_ = std::make_shared<StateManager>();
    RCLCPP_INFO(this->get_logger(), "StateManager created.");
    hardware_coordinator_ = std::make_shared<HardwareCoordinator>(state_manager_.get(), shared_from_this());
    RCLCPP_INFO(this->get_logger(), "HardwareCoordinator created.");
    persistence_coordinator_ = std::make_shared<PersistenceCoordinator>(state_manager_.get(), shared_from_this(), client_cb_group_);
    RCLCPP_INFO(this->get_logger(), "PersistenceCoordinator created.");
    control_logic_ = std::make_unique<ControlLogic>(state_manager_, hardware_coordinator_, persistence_coordinator_);
    RCLCPP_INFO(this->get_logger(), "ControlLogic created.");

    // === 3. 创建本节点的PUB, SUB, SERVER (同时声明并获取参数) ===

    // --- Publishers ---
    auto circuit_status_topic = this->declare_parameter<std::string>(
        control_node_constants::CIRCUIT_STATUS_TOPIC_PARAM,
        control_node_constants::DEFAULT_CIRCUIT_STATUS_TOPIC);
    RCLCPP_INFO(this->get_logger(), "Publishing to Circuit Status Topic: '%s'", circuit_status_topic.c_str());
    circuit_status_pub_ = this->create_publisher<ros2_interfaces::msg::CircuitStatus>(circuit_status_topic, 10);

    auto regulator_status_topic = this->declare_parameter<std::string>(
        control_node_constants::REGULATOR_STATUS_TOPIC_PARAM,
        control_node_constants::DEFAULT_REGULATOR_STATUS_TOPIC);
    RCLCPP_INFO(this->get_logger(), "Publishing to Regulator Status Topic: '%s'", regulator_status_topic.c_str());
    regulator_status_pub_ = this->create_publisher<ros2_interfaces::msg::RegulatorStatus>(regulator_status_topic, 10);

    auto system_settings_topic = this->declare_parameter<std::string>(
        control_node_constants::SYSTEM_SETTINGS_TOPIC_PARAM,
        control_node_constants::DEFAULT_SYSTEM_SETTINGS_TOPIC);
    RCLCPP_INFO(this->get_logger(), "Publishing to System Settings Topic: '%s'", system_settings_topic.c_str());
    system_settings_pub_ = this->create_publisher<ros2_interfaces::msg::SystemSettings>(system_settings_topic, rclcpp::QoS(10).transient_local());

    auto regulator_settings_topic = this->declare_parameter<std::string>(
        control_node_constants::REGULATOR_SETTINGS_TOPIC_PARAM,
        control_node_constants::DEFAULT_REGULATOR_SETTINGS_TOPIC);
    RCLCPP_INFO(this->get_logger(), "Publishing to Regulator Settings Topic: '%s'", regulator_settings_topic.c_str());
    regulator_settings_pub_ = this->create_publisher<ros2_interfaces::msg::RegulatorSettings>(regulator_settings_topic, rclcpp::QoS(10).transient_local());

    auto circuit_settings_topic = this->declare_parameter<std::string>(
        control_node_constants::CIRCUIT_SETTINGS_TOPIC_PARAM,
        control_node_constants::DEFAULT_CIRCUIT_SETTINGS_TOPIC);
    RCLCPP_INFO(this->get_logger(), "Publishing to Circuit Settings Topic: '%s'", circuit_settings_topic.c_str());
    circuit_settings_pub_ = this->create_publisher<ros2_interfaces::msg::CircuitSettings>(circuit_settings_topic, rclcpp::QoS(10).transient_local());

    // --- Subscribers ---
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    auto regulator_operation_command_topic = this->declare_parameter<std::string>(
        control_node_constants::REGULATOR_OPERATION_COMMAND_TOPIC_PARAM,
        control_node_constants::DEFAULT_REGULATOR_OPERATION_COMMAND_TOPIC);
    RCLCPP_INFO(this->get_logger(), "Subscribing to Regulator Operation Command Topic: '%s'", regulator_operation_command_topic.c_str());
    regulator_operation_command_sub_ = this->create_subscription<ros2_interfaces::msg::RegulatorOperationCommand>(
        regulator_operation_command_topic, qos,
        std::bind(&ControlNode::regulator_operation_command_callback, this, std::placeholders::_1));

    auto clear_alarm_topic = this->declare_parameter<std::string>(
        control_node_constants::CLEAR_ALARM_TOPIC_PARAM,
        control_node_constants::DEFAULT_CLEAR_ALARM_TOPIC);
    RCLCPP_INFO(this->get_logger(), "Subscribing to Clear Alarm Topic: '%s'", clear_alarm_topic.c_str());
    clear_alarm_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        clear_alarm_topic, qos, std::bind(&ControlNode::clear_alarm_callback, this, std::placeholders::_1));

    // --- Setting Servers ---
    auto set_system_settings_service_name = this->declare_parameter<std::string>(
        control_node_constants::SET_SYSTEM_SETTINGS_SERVICE_PARAM,
        control_node_constants::DEFAULT_SET_SYSTEM_SETTINGS_SERVICE);
    RCLCPP_INFO(this->get_logger(), "Providing SetSystemSettings service: '%s'", set_system_settings_service_name.c_str());
    set_system_settings_service_ = this->create_service<ros2_interfaces::srv::SetSystemSettings>(
        set_system_settings_service_name,
        std::bind(&ControlNode::set_system_settings_callback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), server_cb_group_);

    auto set_regulator_settings_service_name = this->declare_parameter<std::string>(
        control_node_constants::SET_REGULATOR_SETTINGS_SERVICE_PARAM,
        control_node_constants::DEFAULT_SET_REGULATOR_SETTINGS_SERVICE);
    RCLCPP_INFO(this->get_logger(), "Providing SetRegulatorSettings service: '%s'", set_regulator_settings_service_name.c_str());
    set_regulator_settings_service_ = this->create_service<ros2_interfaces::srv::SetRegulatorSettings>(
        set_regulator_settings_service_name,
        std::bind(&ControlNode::set_regulator_settings_callback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), server_cb_group_);

    auto set_circuit_settings_service_name = this->declare_parameter<std::string>(
        control_node_constants::SET_CIRCUIT_SETTINGS_SERVICE_PARAM,
        control_node_constants::DEFAULT_SET_CIRCUIT_SETTINGS_SERVICE);
    RCLCPP_INFO(this->get_logger(), "Providing SetCircuitSettings service: '%s'", set_circuit_settings_service_name.c_str());
    set_circuit_settings_service_ = this->create_service<ros2_interfaces::srv::SetCircuitSettings>(
        set_circuit_settings_service_name,
        std::bind(&ControlNode::set_circuit_settings_callback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), server_cb_group_);

    // --- Command Servers ---
    auto regulator_breaker_service_name = this->declare_parameter<std::string>(
        control_node_constants::REGULATOR_BREAKER_COMMAND_SERVICE_PARAM,
        control_node_constants::DEFAULT_REGULATOR_BREAKER_COMMAND_SERVICE);
    RCLCPP_INFO(this->get_logger(), "Providing RegulatorBreakerCommand service: '%s'", regulator_breaker_service_name.c_str());
    regulator_breaker_command_service_ = this->create_service<ros2_interfaces::srv::RegulatorBreakerCommand>(
        regulator_breaker_service_name, std::bind(&ControlNode::regulator_breaker_command_callback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), server_cb_group_);

    auto circuit_mode_service_name = this->declare_parameter<std::string>(
        control_node_constants::CIRCUIT_MODE_COMMAND_SERVICE_PARAM,
        control_node_constants::DEFAULT_CIRCUIT_MODE_COMMAND_SERVICE);
    RCLCPP_INFO(this->get_logger(), "Providing CircuitModeCommand service: '%s'", circuit_mode_service_name.c_str());
    circuit_mode_command_service_ = this->create_service<ros2_interfaces::srv::CircuitModeCommand>(
        circuit_mode_service_name, std::bind(&ControlNode::circuit_mode_command_callback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), server_cb_group_);

    auto circuit_breaker_service_name = this->declare_parameter<std::string>(
        control_node_constants::CIRCUIT_BREAKER_COMMAND_SERVICE_PARAM,
        control_node_constants::DEFAULT_CIRCUIT_BREAKER_COMMAND_SERVICE);
    RCLCPP_INFO(this->get_logger(), "Providing CircuitBreakerCommand service: '%s'", circuit_breaker_service_name.c_str());
    circuit_breaker_command_service_ = this->create_service<ros2_interfaces::srv::CircuitBreakerCommand>(
        circuit_breaker_service_name, std::bind(&ControlNode::circuit_breaker_command_callback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), server_cb_group_);

    // === 4. 创建定时器 ===
    control_logic_timer_ = this->create_wall_timer(
        20ms, std::bind(&ControlLogic::update, control_logic_.get()));
    RCLCPP_INFO(this->get_logger(), "ControlLogic core timer started at 50Hz.");

    status_broadcast_timer_ = this->create_wall_timer(
        200ms, std::bind(&ControlNode::broadcast_status_callback, this));
    RCLCPP_INFO(this->get_logger(), "Status broadcast timer started at 5Hz.");

    settings_broadcast_timer_ = this->create_wall_timer(
        1s, std::bind(&ControlNode::broadcast_settings_callback, this));
    RCLCPP_INFO(this->get_logger(), "Settings broadcast timer started at 1Hz.");

    lifecycle_check_timer_ = this->create_wall_timer(
        1s, std::bind(&ControlLogic::maintain_lifecycle, control_logic_.get()));

    RCLCPP_INFO(this->get_logger(), "Lifecycle check timer started at 1Hz.");
    RCLCPP_INFO(this->get_logger(), "ControlNode initialization complete. System is running.");
}

ControlNode::~ControlNode()
{
    RCLCPP_INFO(this->get_logger(), "ControlNode destroyed.");
}


// =======================================================================
//               回调函数的实现 (保持简洁，仅做委托)
// =======================================================================

// --- [其余所有回调函数保持不变，此处省略] ---

// --- Topic Callbacks ---
void ControlNode::regulator_operation_command_callback(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received regulator operation command for ID %u, command: %u", msg->regulator_id, msg->command);
    control_logic_->process_regulator_operation_command(msg);
}

void ControlNode::clear_alarm_callback(const std_msgs::msg::Empty::SharedPtr)
{
    RCLCPP_INFO(this->get_logger(), "Received clear alarm command.");
    control_logic_->process_clear_alarm();
}

// --- Command Service Callbacks ---
void ControlNode::regulator_breaker_command_callback(
    const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request> request,
    std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "RegulatorBreakerCommand service called for ID %u, command %u", request->regulator_id, request->command);
    control_logic_->handle_regulator_breaker_command_request(request,
                                                             [response](bool success, const std::string& message) {
                                                                 response->success = success;
                                                                 response->message = message;
                                                             });
}

void ControlNode::circuit_mode_command_callback(
    const std::shared_ptr<ros2_interfaces::srv::CircuitModeCommand::Request> request,
    std::shared_ptr<ros2_interfaces::srv::CircuitModeCommand::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "CircuitModeCommand service called for ID %u, command %u", request->circuit_id, request->command);
    control_logic_->handle_circuit_mode_command_request(request,
                                                        [response](bool success, const std::string& message) {
                                                            response->success = success;
                                                            response->message = message;
                                                        });
}

void ControlNode::circuit_breaker_command_callback(
    const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request> request,
    std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "CircuitBreakerCommand service called for ID %u, command %u", request->circuit_id, request->command);
    control_logic_->handle_circuit_breaker_command_request(request,
                                                           [response](bool success, const std::string& message) {
                                                               response->success = success;
                                                               response->message = message;
                                                           });
}

// --- Settings Service Callbacks ---
void ControlNode::set_system_settings_callback(
    const std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Request> request,
    std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "SetSystemSettings service called. Processing...");
    auto promise = std::make_shared<std::promise<void>>();
    auto future = promise->get_future();

    auto final_callback = [this, response, promise](bool success, const std::string& message) {
        response->success = success;
        response->message = message;
        if (success) {
            auto settings_msg = state_manager_->get_system_settings();
            system_settings_pub_->publish(settings_msg);
        }
        promise->set_value();
    };

    control_logic_->handle_set_system_settings_request(request, final_callback);
    if (future.wait_for(3s) == std::future_status::timeout)
    {
        RCLCPP_ERROR(this->get_logger(), "Timeout: set_system_settings_callback.");
        response->success = false;
        response->message = "Timeout: A downstream service (e.g., persistence) is not responding.";
    }
}

void ControlNode::set_regulator_settings_callback(
    const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request> request,
    std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "SetRegulatorSettings service called for ID %u. Processing...", request->settings.regulator_id);
    auto promise = std::make_shared<std::promise<void>>();
    auto future = promise->get_future();

    auto final_callback = [this, response, promise, id = request->settings.regulator_id](bool success, const std::string& message) {
        response->success = success;
        response->message = message;
        if (success) {
            auto settings_msg = state_manager_->get_regulator_settings(id);
            regulator_settings_pub_->publish(settings_msg);
        }
        promise->set_value();
    };

    control_logic_->handle_set_regulator_settings_request(request, final_callback);
    if (future.wait_for(3s) == std::future_status::timeout)
    {
        RCLCPP_ERROR(this->get_logger(), "Timeout: set_regulator_settings_callback, regulator ID %u.", request->settings.regulator_id);
        response->success = false;
        response->message = "Timeout: A downstream service (e.g., persistence) is not responding.";
    }
}

void ControlNode::set_circuit_settings_callback(
    const std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Request> request,
    std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "SetCircuitSettings service called for ID %u. Processing...", request->settings.circuit_id);
    auto promise = std::make_shared<std::promise<void>>();
    auto future = promise->get_future();

    auto final_callback = [this, response, promise, id = request->settings.circuit_id](bool success, const std::string& message) {
        response->success = success;
        response->message = message;
        if (success) {
            auto settings_msg = state_manager_->get_circuit_settings(id);
            circuit_settings_pub_->publish(settings_msg);
        }
        promise->set_value();
    };

    control_logic_->handle_set_circuit_settings_request(request, final_callback);
    if (future.wait_for(3s) == std::future_status::timeout)
    {
        RCLCPP_ERROR(this->get_logger(), "Timeout waiting for service response for circuit ID %u.", request->settings.circuit_id);
        response->success = false;
        response->message = "Timeout: A downstream service (e.g., persistence) is not responding.";
    }
}

// --- 广播回调实现 ---
void ControlNode::broadcast_status_callback()
{
    // 从StateManager中读取处理后的常规数据并发布
    for(uint8_t id = 1; id <= StateManager::NUM_CIRCUITS; ++id)
    {
        auto msg = state_manager_->get_circuit_status(id);
        circuit_status_pub_->publish(msg);
        //RCLCPP_INFO(this->get_logger(), "circuit status id = %d", id);
    }

    for(uint8_t id = 1; id <= StateManager::NUM_REGULATORS; ++id)
    {
        auto msg = state_manager_->get_regulator_status(id);
        regulator_status_pub_->publish(msg);
        //RCLCPP_INFO(this->get_logger(), "regulator status id = %d", id);
    }
}

void ControlNode::broadcast_settings_callback()
{
    // [NEW] 守卫逻辑：如果还没有从数据库完全加载配置，则不进行广播，
    // 防止 RecordNode 收到默认值后覆盖数据库。
    if (!control_logic_->is_settings_synced()) {
        RCLCPP_DEBUG(this->get_logger(), "Settings not synced from DB yet. Skipping broadcast to protect database.");
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Broadcasting all current settings via timer.");

    // 1. 广播系统设置
    // 从状态管理器获取最新的系统设置消息
    auto system_settings_msg = state_manager_->get_system_settings();
    // 通过对应的发布者发布出去
    system_settings_pub_->publish(system_settings_msg);

    // 2. 循环广播所有调压器的设置
    for (uint8_t id = 1; id <= StateManager::NUM_REGULATORS; ++id)
    {
        // 从状态管理器获取指定ID的调压器设置消息
        auto msg = state_manager_->get_regulator_settings(id);
        // 发布
        regulator_settings_pub_->publish(msg);
    }

    // 3. 循环广播所有回路的设置
    for (uint8_t id = 1; id <= StateManager::NUM_CIRCUITS; ++id)
    {
        // 从状态管理器获取指定ID的回路设置消息
        auto msg = state_manager_->get_circuit_settings(id);
        // 发布
        circuit_settings_pub_->publish(msg);

        // RCLCPP_INFO(
        //     this->get_logger(),
        //     "CircuitSettings msg:\n%s",
        //     ros2_interfaces::msg::to_yaml(msg).c_str()
        //     //rosidl_generator_traits::to_yaml(msg).c_str()
        //     );
    }
}
