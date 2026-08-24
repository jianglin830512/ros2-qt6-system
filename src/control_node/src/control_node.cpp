#include "control_node/control_node.hpp"
#include "control_node/control_node_constants.hpp"
#include "control_node/state_manager.hpp"
#include "control_node/hardware_coordinator.hpp"
#include "control_node/persistence_coordinator.hpp"
#include "control_node/control_logic.hpp"
#include <future>
#include <set>

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

    this->declare_parameter(control_node_constants::PLC_COMMAND_TIMEOUT_PARAM, control_node_constants::DEFAULT_PLC_COMMAND_TIMEOUT);
    double plc_cmd_timeout = this->get_parameter(control_node_constants::PLC_COMMAND_TIMEOUT_PARAM).as_double();
    control_logic_->set_command_timeout(plc_cmd_timeout);

    // ============================================================
    // [Publishers]
    // ============================================================
    auto circuit_status_topic = this->declare_parameter<std::string>(
        control_node_constants::CIRCUIT_STATUS_TOPIC_PARAM, control_node_constants::DEFAULT_CIRCUIT_STATUS_TOPIC);
    circuit_status_pub_ = this->create_publisher<ros2_interfaces::msg::CircuitStatus>(circuit_status_topic, 10);

    auto regulator_status_topic = this->declare_parameter<std::string>(
        control_node_constants::REGULATOR_STATUS_TOPIC_PARAM, control_node_constants::DEFAULT_REGULATOR_STATUS_TOPIC);
    regulator_status_pub_ = this->create_publisher<ros2_interfaces::msg::RegulatorStatus>(regulator_status_topic, 10);

    auto system_status_topic = this->declare_parameter<std::string>(
        control_node_constants::SYSTEM_STATUS_TOPIC_PARAM, control_node_constants::DEFAULT_SYSTEM_STATUS_TOPIC);
    system_status_pub_ = this->create_publisher<ros2_interfaces::msg::SystemStatus>(system_status_topic, 10);

    auto system_settings_topic = this->declare_parameter<std::string>(
        control_node_constants::SYSTEM_SETTINGS_TOPIC_PARAM, control_node_constants::DEFAULT_SYSTEM_SETTINGS_TOPIC);
    system_settings_pub_ = this->create_publisher<ros2_interfaces::msg::SystemSettings>(system_settings_topic, rclcpp::QoS(10).transient_local());

    auto regulator_settings_topic = this->declare_parameter<std::string>(
        control_node_constants::REGULATOR_SETTINGS_TOPIC_PARAM, control_node_constants::DEFAULT_REGULATOR_SETTINGS_TOPIC);
    regulator_settings_pub_ = this->create_publisher<ros2_interfaces::msg::RegulatorSettings>(regulator_settings_topic, rclcpp::QoS(10).transient_local());

    auto circuit_settings_topic = this->declare_parameter<std::string>(
        control_node_constants::CIRCUIT_SETTINGS_TOPIC_PARAM, control_node_constants::DEFAULT_CIRCUIT_SETTINGS_TOPIC);
    circuit_settings_pub_ = this->create_publisher<ros2_interfaces::msg::CircuitSettings>(circuit_settings_topic, rclcpp::QoS(10).transient_local());

    // ============================================================
    // [Subscribers]
    // ============================================================
    auto reg_op_command_topic = this->declare_parameter<std::string>(
        control_node_constants::REGULATOR_OPERATION_COMMAND_TOPIC_PARAM, control_node_constants::DEFAULT_REGULATOR_OPERATION_COMMAND_TOPIC);
    regulator_operation_command_sub_ = this->create_subscription<ros2_interfaces::msg::RegulatorOperationCommand>(
        reg_op_command_topic, 10, std::bind(&ControlNode::regulator_operation_command_callback, this, std::placeholders::_1));

    auto clear_alarm_topic = this->declare_parameter<std::string>(
        control_node_constants::CLEAR_ALARM_TOPIC_PARAM, control_node_constants::DEFAULT_CLEAR_ALARM_TOPIC);
    clear_alarm_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        clear_alarm_topic, 10, std::bind(&ControlNode::clear_alarm_callback, this, std::placeholders::_1));

    // ============================================================
    // [Services]
    // ============================================================
    auto set_system_settings_srv = this->declare_parameter<std::string>(
        control_node_constants::SET_SYSTEM_SETTINGS_SERVICE_PARAM, control_node_constants::DEFAULT_SET_SYSTEM_SETTINGS_SERVICE);
    set_system_settings_service_ = this->create_service<ros2_interfaces::srv::SetSystemSettings>(
        set_system_settings_srv, std::bind(&ControlNode::set_system_settings_callback, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), server_cb_group_);

    auto set_regulator_settings_srv = this->declare_parameter<std::string>(
        control_node_constants::SET_REGULATOR_SETTINGS_SERVICE_PARAM, control_node_constants::DEFAULT_SET_REGULATOR_SETTINGS_SERVICE);
    set_regulator_settings_service_ = this->create_service<ros2_interfaces::srv::SetRegulatorSettings>(
        set_regulator_settings_srv, std::bind(&ControlNode::set_regulator_settings_callback, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), server_cb_group_);

    auto set_circuit_settings_srv = this->declare_parameter<std::string>(
        control_node_constants::SET_CIRCUIT_SETTINGS_SERVICE_PARAM, control_node_constants::DEFAULT_SET_CIRCUIT_SETTINGS_SERVICE);
    set_circuit_settings_service_ = this->create_service<ros2_interfaces::srv::SetCircuitSettings>(
        set_circuit_settings_srv, std::bind(&ControlNode::set_circuit_settings_callback, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), server_cb_group_);

    auto reg_breaker_command_srv = this->declare_parameter<std::string>(
        control_node_constants::REGULATOR_BREAKER_COMMAND_SERVICE_PARAM, control_node_constants::DEFAULT_REGULATOR_BREAKER_COMMAND_SERVICE);
    regulator_breaker_command_service_ = this->create_service<ros2_interfaces::srv::RegulatorBreakerCommand>(
        reg_breaker_command_srv, std::bind(&ControlNode::regulator_breaker_command_callback, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), server_cb_group_);

    auto circuit_breaker_command_srv = this->declare_parameter<std::string>(
        control_node_constants::CIRCUIT_BREAKER_COMMAND_SERVICE_PARAM, control_node_constants::DEFAULT_CIRCUIT_BREAKER_COMMAND_SERVICE);
    circuit_breaker_command_service_ = this->create_service<ros2_interfaces::srv::CircuitBreakerCommand>(
        circuit_breaker_command_srv, std::bind(&ControlNode::circuit_breaker_command_callback, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), server_cb_group_);

    // [NEW] Global Node 电缆同步 Service Client
    auto get_cable_batch_srv = this->declare_parameter<std::string>(
        control_node_constants::GET_CABLE_INFO_BATCH_SERVICE_PARAM, control_node_constants::DEFAULT_GET_CABLE_INFO_BATCH_SERVICE);
    get_cable_info_batch_client_ = this->create_client<ros2_interfaces::srv::GetCableInfoBatch>(
        get_cable_batch_srv, rclcpp::ServicesQoS(), client_cb_group_);

    // ============================================================
    // [Timers]
    // ============================================================
    control_logic_timer_ = this->create_wall_timer(20ms, std::bind(&ControlLogic::update, control_logic_.get()));
    status_broadcast_timer_ = this->create_wall_timer(200ms, std::bind(&ControlNode::broadcast_status_callback, this));
    settings_broadcast_timer_ = this->create_wall_timer(1s, std::bind(&ControlNode::broadcast_settings_callback, this));
    lifecycle_check_timer_ = this->create_wall_timer(1s, std::bind(&ControlLogic::maintain_lifecycle, control_logic_.get()));

    // [NEW] 启动低频电缆信息同步定时器
    int sync_interval = this->declare_parameter<int>(
        control_node_constants::CABLE_SYNC_INTERVAL_SEC_PARAM, control_node_constants::DEFAULT_CABLE_SYNC_INTERVAL_SEC);
    cable_sync_timer_ = this->create_wall_timer(std::chrono::seconds(sync_interval), std::bind(&ControlNode::cable_sync_timer_callback, this));

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

// [NEW] 电缆信息定时同步实现
void ControlNode::cable_sync_timer_callback() {
    if (!control_logic_->is_settings_synced()) return;

    if (!get_cable_info_batch_client_->service_is_ready()) {
        return; // Global Node 不在线时跳过同步
    }

    auto c1 = state_manager_->get_circuit_settings(1);
    auto c2 = state_manager_->get_circuit_settings(2);

    std::set<int32_t> ids_to_fetch;
    if (c1.sample_cable.id > 0) ids_to_fetch.insert(c1.sample_cable.id);
    if (c2.sample_cable.id > 0) ids_to_fetch.insert(c2.sample_cable.id);

    if (ids_to_fetch.empty()) return;

    auto req = std::make_shared<ros2_interfaces::srv::GetCableInfoBatch::Request>();
    req->ids.assign(ids_to_fetch.begin(), ids_to_fetch.end());

    get_cable_info_batch_client_->async_send_request(req,
                                                     [this, c1_id = c1.sample_cable.id, c2_id = c2.sample_cable.id]
                                                     (rclcpp::Client<ros2_interfaces::srv::GetCableInfoBatch>::SharedFuture future) {
                                                         try {
                                                             auto resp = future.get();
                                                             if (resp->success) {
                                                                 for (const auto& cable : resp->cables) {
                                                                     if (cable.id == c1_id) {
                                                                         auto current_c1 = state_manager_->get_circuit_settings(1);
                                                                         if (current_c1.sample_cable.id == cable.id) {
                                                                             current_c1.sample_cable = cable;
                                                                             state_manager_->update_circuit_settings(1, current_c1);
                                                                         }
                                                                     }
                                                                     if (cable.id == c2_id) {
                                                                         auto current_c2 = state_manager_->get_circuit_settings(2);
                                                                         if (current_c2.sample_cable.id == cable.id) {
                                                                             current_c2.sample_cable = cable;
                                                                             state_manager_->update_circuit_settings(2, current_c2);
                                                                         }
                                                                     }
                                                                 }
                                                             }
                                                         } catch (const std::exception& e) {
                                                             RCLCPP_DEBUG(this->get_logger(), "Cable sync failed: %s", e.what());
                                                         }
                                                     }
                                                     );
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
