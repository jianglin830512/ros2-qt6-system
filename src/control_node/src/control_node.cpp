#include "control_node/control_node.hpp"
#include "control_node/control_node_constants.hpp"
#include "control_node/control_node_context.hpp"
#include "control_node/control_logic.hpp"

#include <future>

// 使用 std::chrono_literals 来方便地使用时间单位，例如 20ms
using namespace std::chrono_literals;

ControlNode::ControlNode() : Node("control_node")
{
    RCLCPP_INFO(this->get_logger(), "Initializing ControlNode with layered architecture...");

    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    server_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // === 1. 创建核心组件 (严格按照依赖顺序) ===
    context_ = std::make_shared<ControlNodeContext>();
    context_->logging_interface = this->get_node_logging_interface();
    control_logic_ = std::make_unique<ControlLogic>(context_);


    RCLCPP_INFO(this->get_logger(), "Persistence Coordinator created.");
    RCLCPP_INFO(this->get_logger(), "Core components (StateManager, HardwareCoordinator, ControlLogic) created.");

    // === 2. 声明并获取所有ROS参数 ===
    // --- 发布器参数 ---
    this->declare_parameter<std::string>(
        control_node_constants::CIRCUIT_STATUS_TOPIC_PARAM,
        control_node_constants::DEFAULT_CIRCUIT_STATUS_TOPIC);
    this->declare_parameter<std::string>(
        control_node_constants::REGULATOR_STATUS_TOPIC_PARAM,
        control_node_constants::DEFAULT_REGULATOR_STATUS_TOPIC);

    // --- 订阅器参数 ---
    this->declare_parameter<std::string>(
        control_node_constants::REGULATOR_COMMAND_TOPIC_PARAM,
        control_node_constants::DEFAULT_REGULATOR_COMMAND_TOPIC);
    this->declare_parameter<std::string>(
        control_node_constants::CIRCUIT_COMMAND_TOPIC_PARAM,
        control_node_constants::DEFAULT_CIRCUIT_COMMAND_TOPIC);
    this->declare_parameter<std::string>(
        control_node_constants::CLEAR_ALARM_TOPIC_PARAM,
        control_node_constants::DEFAULT_CLEAR_ALARM_TOPIC);

    // --- 服务服务器参数 ---
    // Server
    this->declare_parameter<std::string>(
        control_node_constants::SET_SYSTEM_SETTINGS_SERVICE_PARAM,
        control_node_constants::DEFAULT_SET_SYSTEM_SETTINGS_SERVICE);
    this->declare_parameter<std::string>(
        control_node_constants::SET_REGULATOR_SETTINGS_SERVICE_PARAM,
        control_node_constants::DEFAULT_SET_REGULATOR_SETTINGS_SERVICE);
    this->declare_parameter<std::string>(
        control_node_constants::SET_CIRCUIT_SETTINGS_SERVICE_PARAM,
        control_node_constants::DEFAULT_SET_CIRCUIT_SETTINGS_SERVICE);
    // Client
    this->declare_parameter<std::string>(
        control_node_constants::SAVE_SYSTEM_SETTINGS_SERVICE_PARAM,
        control_node_constants::DEFAULT_SAVE_SYSTEM_SETTINGS_SERVICE);
    this->declare_parameter<std::string>(
        control_node_constants::SAVE_REGULATOR_SETTINGS_SERVICE_PARAM,
        control_node_constants::DEFAULT_SAVE_REGULATOR_SETTINGS_SERVICE);
    this->declare_parameter<std::string>(
        control_node_constants::SAVE_CIRCUIT_SETTINGS_SERVICE_PARAM,
        control_node_constants::DEFAULT_SAVE_CIRCUIT_SETTINGS_SERVICE);

    RCLCPP_INFO(this->get_logger(), "All parameters declared.");


    // === 3. 创建ROS句柄并填充Context ===
    // --- 发布器 ---

    this->get_parameter(control_node_constants::CIRCUIT_STATUS_TOPIC_PARAM, circuit_status_topic_);
    this->get_parameter(control_node_constants::REGULATOR_STATUS_TOPIC_PARAM, regulator_status_topic_);
    context_->circuit_status_publisher = this->create_publisher<ros2_interfaces::msg::CircuitStatus>(circuit_status_topic_, 10);
    context_->regulator_status_publisher = this->create_publisher<ros2_interfaces::msg::VoltageRegulatorStatus>(regulator_status_topic_, 10);
    RCLCPP_INFO(this->get_logger(), "Publishers created and added to context.");

    // ... 如果有需要，在这里创建客户端并添加到context ...


    // === 4. 创建订阅器 ===
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    this->get_parameter(control_node_constants::REGULATOR_COMMAND_TOPIC_PARAM, regulator_command_topic_);
    this->get_parameter(control_node_constants::CIRCUIT_COMMAND_TOPIC_PARAM, circuit_command_topic_);
    this->get_parameter(control_node_constants::CLEAR_ALARM_TOPIC_PARAM, clear_alarm_topic_);

    regulator_command_sub_ = this->create_subscription<ros2_interfaces::msg::VoltageRegulatorCommand>(
        regulator_command_topic_, qos, std::bind(&ControlNode::regulator_command_callback, this, std::placeholders::_1));
    circuit_command_sub_ = this->create_subscription<ros2_interfaces::msg::CircuitCommand>(
        circuit_command_topic_, qos, std::bind(&ControlNode::circuit_command_callback, this, std::placeholders::_1));
    clear_alarm_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        clear_alarm_topic_, qos, std::bind(&ControlNode::clear_alarm_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribers created.");


    // === 5. 创建服务服务器 ===

    // Server
    this->get_parameter(control_node_constants::SET_SYSTEM_SETTINGS_SERVICE_PARAM, set_system_settings_service_name_);
    this->get_parameter(control_node_constants::SET_REGULATOR_SETTINGS_SERVICE_PARAM, set_regulator_settings_service_name_);
    this->get_parameter(control_node_constants::SET_CIRCUIT_SETTINGS_SERVICE_PARAM, set_circuit_settings_service_name_);


    set_system_settings_service_ = this->create_service<ros2_interfaces::srv::SetSystemSettings>(
        set_system_settings_service_name_,
        std::bind(&ControlNode::set_system_settings_callback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(),
        server_cb_group_
        );
    set_regulator_settings_service_ = this->create_service<ros2_interfaces::srv::SetRegulatorSettings>(
        set_regulator_settings_service_name_,
        std::bind(&ControlNode::set_regulator_settings_callback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(),
        server_cb_group_
        );
    set_circuit_settings_service_ = this->create_service<ros2_interfaces::srv::SetCircuitSettings>(
        set_circuit_settings_service_name_,
        std::bind(&ControlNode::set_circuit_settings_callback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(),
        server_cb_group_
        );
    RCLCPP_INFO(this->get_logger(), "Service servers created.");

    // Client
    this->get_parameter(control_node_constants::SAVE_SYSTEM_SETTINGS_SERVICE_PARAM, save_system_settings_service_name_);
    this->get_parameter(control_node_constants::SAVE_REGULATOR_SETTINGS_SERVICE_PARAM, save_regulator_settings_service_name_);
    this->get_parameter(control_node_constants::SAVE_CIRCUIT_SETTINGS_SERVICE_PARAM, save_circuit_settings_service_name_);

    context_->save_system_settings_client = this->create_client<ros2_interfaces::srv::SetSystemSettings>(
        save_system_settings_service_name_,
        rclcpp::ServicesQoS(),
        client_cb_group_
        );
    context_->save_regulator_settings_client = this->create_client<ros2_interfaces::srv::SetRegulatorSettings>(
        save_regulator_settings_service_name_,
        rclcpp::ServicesQoS(),
        client_cb_group_
        );

    context_->save_circuit_settings_client = this->create_client<ros2_interfaces::srv::SetCircuitSettings>(
        save_circuit_settings_service_name_,
        rclcpp::ServicesQoS(),
        client_cb_group_
        );
    RCLCPP_INFO(this->get_logger(), "Service clients created.");

    // === 6. 创建驱动ControlLogic的核心定时器 ===
    control_logic_timer_ = this->create_wall_timer(
        20ms, // 50Hz 控制频率
        std::bind(&ControlLogic::update, control_logic_.get())
        );
    RCLCPP_INFO(this->get_logger(), "ControlLogic core timer started at 50Hz.");

    RCLCPP_INFO(this->get_logger(), "ControlNode initialization complete. System is running.");
}

ControlNode::~ControlNode()
{
    RCLCPP_INFO(this->get_logger(), "ControlNode destroyed.");
}


// =======================================================================
//               回调函数的实现 (现在非常简洁)
// =======================================================================

// --- 命令回调 ---

void ControlNode::regulator_command_callback(const ros2_interfaces::msg::VoltageRegulatorCommand::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received regulator command for ID %u, command: %u", msg->regulator_id, msg->command);
    // 委托给ControlLogic来处理这个命令
    // control_logic_->process_regulator_command(msg);
}

void ControlNode::circuit_command_callback(const ros2_interfaces::msg::CircuitCommand::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received circuit command for ID %u, command: %u", msg->circuit_id, msg->command);
    // 委托给ControlLogic来处理这个命令
    // control_logic_->process_circuit_command(msg);
}

void ControlNode::clear_alarm_callback(const std_msgs::msg::Empty::SharedPtr)
{
    RCLCPP_INFO(this->get_logger(), "Received clear alarm command.");
    // 委托给StateManager或ControlLogic来清除报警状态
    // state_manager_->clear_all_alarms();
}


// --- 服务回调 ---

void ControlNode::set_system_settings_callback(
    const std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Request> request,
    std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "SetSystemSettings service called. Processing asynchronously...");

    auto promise = std::make_shared<std::promise<void>>();
    auto future = promise->get_future();

    // 定义回调 lambda。它捕获 promise
    auto final_callback = [response, promise](bool success, const std::string& message) {
        response->success = success;
        response->message = message;
        // 唤醒正在等待 future 的线程。
        promise->set_value();
    };

    control_logic_->handle_set_system_settings_request(request, final_callback);

    // 服务回调的主线程将等待 future 的结果。
    // 在多线程执行器中，其他线程可以继续工作来最终调用 set_value()
    future.get();

    // 一旦 future.get() 返回，说明 promise 已被兑现，response 已被填充。
    RCLCPP_DEBUG(this->get_logger(), "Function set_system_settings_callback() is down.");
}

void ControlNode::set_regulator_settings_callback(
    const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request> request,
    std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "SetRegulatorSettings service called for ID %u. Processing asynchronously...", request->regulator_id);

    auto promise = std::make_shared<std::promise<void>>();
    auto future = promise->get_future();

    // 定义回调 lambda。它捕获 promise
    auto final_callback = [response, promise](bool success, const std::string& message) {
        response->success = success;
        response->message = message;
        // 唤醒正在等待 future 的线程。
        promise->set_value();
    };

    control_logic_->handle_set_regulator_settings_request(request,final_callback);

    // 服务回调的主线程将等待 future 的结果。
    // 在多线程执行器中，其他线程可以继续工作来最终调用 set_value()
    future.get();

    // 一旦 future.get() 返回，说明 promise 已被兑现，response 已被填充。
    RCLCPP_DEBUG(this->get_logger(), "Function set_regulator_settings_callback() is down.");
}

void ControlNode::set_circuit_settings_callback(
    const std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Request> request,
    std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "SetCircuitSettings service called for ID %u. Blocking until async chain is complete...", request->circuit_id);

    auto promise = std::make_shared<std::promise<void>>();
    auto future = promise->get_future();

    // 定义回调 lambda。它捕获 promise
    auto final_callback = [response, promise](bool success, const std::string& message) {
        response->success = success;
        response->message = message;
        // 唤醒正在等待 future 的线程。
        promise->set_value();
    };

    control_logic_->handle_set_circuit_settings_request(request, final_callback);

    // future.get();
    if (future.wait_for(3s) == std::future_status::timeout)
    {
        // 如果超时了，意味着 promise 没有被兑现，异步链条在某处卡住了（很可能是因为服务端不存在）
        RCLCPP_ERROR(this->get_logger(), "Timeout waiting for service response for circuit ID %u. The service may be down.", request->circuit_id);
        // 我们必须手动填充一个失败的 response，否则客户端会一直等待
        response->success = false;
        response->message = "Timeout: The persistence service is not responding.";
    }

    // 一旦 future.get() 返回，说明 promise 已被兑现，response 已被填充。
    RCLCPP_DEBUG(this->get_logger(), "Function set_circuit_settings_callback() is down.");
}
