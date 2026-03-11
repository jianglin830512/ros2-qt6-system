#include "qt_node/qt_ros_node.hpp"
#include "qt_node/qt_node_constants.hpp"
#include <QDebug>
#include <QPointF>

// Helper function to avoid code duplication (optional but recommended)
namespace {

void convertLoopStatusRosToQt(const ros2_interfaces::msg::LoopStatus& ros_loop, LoopStatusData& qt_loop)
{
    // common
    qt_loop.is_heat = ros_loop.is_heat;
    qt_loop.completed_cycle_count = ros_loop.completed_cycle_count;
    qt_loop.remaining_cycle_count = ros_loop.remaining_cycle_count;
    qt_loop.elapsed_heating_time_sec = ros_loop.elapsed_heating_time.sec;
    qt_loop.remaining_heating_time_sec = ros_loop.remaining_heating_time.sec;
    // hardware
    qt_loop.current = ros_loop.hardware_loop_status.current;
    qt_loop.breaker_closed_switch_ack = ros_loop.hardware_loop_status.breaker_closed_switch_ack;
    qt_loop.breaker_opened_switch_ack = ros_loop.hardware_loop_status.breaker_opened_switch_ack;
    // 高效地转换数组
    qt_loop.temperature_array.resize(ros_loop.hardware_loop_status.temperature_array.size());
    std::copy(ros_loop.hardware_loop_status.temperature_array.begin(), ros_loop.hardware_loop_status.temperature_array.end(), qt_loop.temperature_array.begin());
}

// =======================================================================
//   Helper function to convert Qt LoopSettingsData to ROS LoopSettings
// =======================================================================
void convertLoopSettingsQtToRos(const LoopSettingsData* qt_loop_settings, ros2_interfaces::msg::LoopSettings& ros_loop_settings)
{
    // The main function has already checked for null pointers, but it's good practice.
    if (!qt_loop_settings) {
        return; // Or throw an exception, depending on error handling strategy
    }

    // --- Convert simple numeric types ---
    ros_loop_settings.hardware_loop_settings.start_current_a = qt_loop_settings->start_current_a();
    ros_loop_settings.hardware_loop_settings.max_current_a = qt_loop_settings->max_current_a();
    ros_loop_settings.hardware_loop_settings.current_change_range_percent = qt_loop_settings->current_change_range_percent();
    ros_loop_settings.hardware_loop_settings.ct_ratio = qt_loop_settings->ct_ratio();

    ros_loop_settings.cycle_count = qt_loop_settings->cycle_count();

    // --- Convert Date (Time msg) ---
    // 要求：时间部分为0
    QDateTime qt_date = qt_loop_settings->start_date();
    QDate date_part = qt_date.date();
    // 构造一个当日 00:00:00 的 QDateTime
    QDateTime pure_date(date_part, QTime(0, 0, 0));

    ros_loop_settings.start_date.sec = static_cast<int32_t>(pure_date.toSecsSinceEpoch());
    ros_loop_settings.start_date.nanosec = 0;

    // --- Convert Heating Time (Duration msg) ---
    // 要求：从00:00:00开始计算
    ros_loop_settings.heating_time.sec = qt_loop_settings->heating_start_time_sec();
    ros_loop_settings.heating_time.nanosec = 0;

    // --- Convert int (seconds) to builtin_interfaces::msg::Duration ---
    ros_loop_settings.heating_duration.sec = qt_loop_settings->heating_duration_sec();
    ros_loop_settings.heating_duration.nanosec = 0;

    ros_loop_settings.enabled = qt_loop_settings->enabled();
}


// =======================================================================
//   Main function to convert Qt CircuitSettingsData to ROS CircuitSettings
// =======================================================================
void convertCircuitSettingsQtToRos(const CircuitSettingsData* qt_circuit_settings, ros2_interfaces::msg::CircuitSettings& ros_circuit_settings, const rclcpp::Logger& logger)
{
    // --- 1. Safety Checks ---
    if (!qt_circuit_settings) {
        RCLCPP_ERROR(logger, "Received a null pointer for CircuitSettingsData in convertCircuitSettingsQtToRos. Conversion aborted.");
        return;
    }
    // Check nested QObject pointers
    if (!qt_circuit_settings->test_loop() || !qt_circuit_settings->ref_loop() || !qt_circuit_settings->sample_params()) {
        RCLCPP_ERROR(logger, "Nested settings data (test_loop, ref_loop, or sample_params) is null. Conversion aborted.");
        return;
    }

    // --- 2. Convert Test and Reference Loops using the helper function ---
    RCLCPP_INFO(logger, "Converting Test Loop settings...");
    convertLoopSettingsQtToRos(qt_circuit_settings->test_loop(), ros_circuit_settings.test_loop);

    RCLCPP_INFO(logger, "Converting Reference Loop settings...");
    convertLoopSettingsQtToRos(qt_circuit_settings->ref_loop(), ros_circuit_settings.ref_loop);

    // --- 3. Convert Sample Parameters ---
    RCLCPP_INFO(logger, "Converting Sample Parameters...");
    ros_circuit_settings.sample_params.cable_type = qt_circuit_settings->sample_params()->cable_type().toStdString();
    ros_circuit_settings.sample_params.cable_spec = qt_circuit_settings->sample_params()->cable_spec().toStdString();
    ros_circuit_settings.sample_params.insulation_material = qt_circuit_settings->sample_params()->insulation_material().toStdString();
    ros_circuit_settings.sample_params.insulation_thickness = static_cast<float>(qt_circuit_settings->sample_params()->insulation_thickness());

    // --- 4. Convert Other Parameters ---
    ros_circuit_settings.curr_mode_use_ref = qt_circuit_settings->curr_mode_use_ref();

    RCLCPP_INFO(logger, "CircuitSettings conversion complete.");
}

} // End of anonymous namespace

QtROSNode::QtROSNode(const std::string &node_name, QObject *parent)
    : QObject(parent), rclcpp::Node(node_name)
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    // --- 创建订阅者 (同时声明、获取并使用参数) ---
    // Circuit Status Subscriber
    circuit_status_topic_ = this->declare_parameter<std::string>(
        qt_node_constants::CIRCUIT_STATUS_TOPIC_PARAM,
        qt_node_constants::DEFAULT_CIRCUIT_STATUS_TOPIC);
    RCLCPP_INFO(this->get_logger(), "Subscribing to Circuit Status Topic: '%s'", circuit_status_topic_.c_str());
    circuit_status_sub_ = this->create_subscription<ros2_interfaces::msg::CircuitStatus>(
        circuit_status_topic_, qos, std::bind(&QtROSNode::circuit_status_callback, this, std::placeholders::_1));

    // Regulator Status Subscriber
    regulator_status_topic_ = this->declare_parameter<std::string>(
        qt_node_constants::REGULATOR_STATUS_TOPIC_PARAM,
        qt_node_constants::DEFAULT_REGULATOR_STATUS_TOPIC);
    RCLCPP_INFO(this->get_logger(), "Subscribing to Regulator Status Topic: '%s'", regulator_status_topic_.c_str());
    regulator_status_sub_ = this->create_subscription<ros2_interfaces::msg::RegulatorStatus>(
        regulator_status_topic_, qos, std::bind(&QtROSNode::regulator_status_callback, this, std::placeholders::_1));

    // System Settings Subscriber
    system_settings_topic_ = this->declare_parameter<std::string>(
        qt_node_constants::SYSTEM_SETTINGS_TOPIC_PARAM,
        qt_node_constants::DEFAULT_SYSTEM_SETTINGS_TOPIC);
    RCLCPP_INFO(this->get_logger(), "Subscribing to System Settings Topic: '%s'", system_settings_topic_.c_str());
    system_settings_sub_ = this->create_subscription<ros2_interfaces::msg::SystemSettings>(
        system_settings_topic_, qos, std::bind(&QtROSNode::system_settings_callback, this, std::placeholders::_1));

    // Regulator Settings Subscriber
    regulator_settings_topic_ = this->declare_parameter<std::string>(
        qt_node_constants::REGULATOR_SETTINGS_TOPIC_PARAM,
        qt_node_constants::DEFAULT_REGULATOR_SETTINGS_TOPIC);
    RCLCPP_INFO(this->get_logger(), "Subscribing to Regulator Settings Topic: '%s'", regulator_settings_topic_.c_str());
    regulator_settings_sub_ = this->create_subscription<ros2_interfaces::msg::RegulatorSettings>(
        regulator_settings_topic_, qos, std::bind(&QtROSNode::regulator_settings_callback, this, std::placeholders::_1));

    // Circuit Settings Subscriber
    circuit_settings_topic_ = this->declare_parameter<std::string>(
        qt_node_constants::CIRCUIT_SETTINGS_TOPIC_PARAM,
        qt_node_constants::DEFAULT_CIRCUIT_SETTINGS_TOPIC);
    RCLCPP_INFO(this->get_logger(), "Subscribing to Circuit Settings Topic: '%s'", circuit_settings_topic_.c_str());
    circuit_settings_sub_ = this->create_subscription<ros2_interfaces::msg::CircuitSettings>(
        circuit_settings_topic_, qos, std::bind(&QtROSNode::circuit_settings_callback, this, std::placeholders::_1));


    // --- 创建发布者 (同时声明、获取并使用参数) ---
    // Regulator Operation Command Publisher
    regulator_operation_command_topic_ = this->declare_parameter<std::string>(
        qt_node_constants::REGULATOR_OPERATION_COMMAND_TOPIC_PARAM,
        qt_node_constants::DEFAULT_REGULATOR_OPERATION_COMMAND_TOPIC);
    RCLCPP_INFO(this->get_logger(), "Publishing to Regulator Operation Command Topic: '%s'", regulator_operation_command_topic_.c_str());
    regulator_operation_command_pub_ =
        this->create_publisher<ros2_interfaces::msg::RegulatorOperationCommand>(regulator_operation_command_topic_, qos);

    // Clear Alarm Publisher
    clear_alarm_topic_ = this->declare_parameter<std::string>(
        qt_node_constants::CLEAR_ALARM_TOPIC_PARAM,
        qt_node_constants::DEFAULT_CLEAR_ALARM_TOPIC);
    RCLCPP_INFO(this->get_logger(), "Publishing to Clear Alarm Topic: '%s'", clear_alarm_topic_.c_str());
    clear_alarm_pub_ = this->create_publisher<std_msgs::msg::Empty>(clear_alarm_topic_, qos);


    // --- 创建服务客户端 (同时声明、获取并使用参数) ---
    // Command Services
    regulator_breaker_command_service_name_ = this->declare_parameter<std::string>(
        qt_node_constants::REGULATOR_BREAKER_COMMAND_SERVICE_PARAM,
        qt_node_constants::DEFAULT_REGULATOR_BREAKER_COMMAND_SERVICE);
    RCLCPP_INFO(this->get_logger(), "Using RegulatorBreakerCommand service: '%s'", regulator_breaker_command_service_name_.c_str());
    regulator_breaker_command_client_ =
        this->create_client<ros2_interfaces::srv::RegulatorBreakerCommand>(regulator_breaker_command_service_name_);

    circuit_mode_command_service_name_ = this->declare_parameter<std::string>(
        qt_node_constants::CIRCUIT_MODE_COMMAND_SERVICE_PARAM,
        qt_node_constants::DEFAULT_CIRCUIT_MODE_COMMAND_SERVICE);
    RCLCPP_INFO(this->get_logger(), "Using CircuitModeCommand service: '%s'", circuit_mode_command_service_name_.c_str());
    circuit_mode_command_client_ =
        this->create_client<ros2_interfaces::srv::CircuitModeCommand>(circuit_mode_command_service_name_);

    circuit_breaker_command_service_name_ = this->declare_parameter<std::string>(
        qt_node_constants::CIRCUIT_BREAKER_COMMAND_SERVICE_PARAM,
        qt_node_constants::CIRCUIT_REGULATOR_BREAKER_COMMAND_SERVICE);
    RCLCPP_INFO(this->get_logger(), "Using CircuitBreakerCommand service: '%s'", circuit_breaker_command_service_name_.c_str());
    circuit_breaker_command_client_ =
        this->create_client<ros2_interfaces::srv::CircuitBreakerCommand>(circuit_breaker_command_service_name_);

    // Settings Services
    set_system_settings_service_name_ = this->declare_parameter<std::string>(
        qt_node_constants::SET_SYSTEM_SETTINGS_SERVICE_PARAM,
        qt_node_constants::DEFAULT_SET_SYSTEM_SETTINGS_SERVICE);
    RCLCPP_INFO(this->get_logger(), "Using SetSystemSettings service: '%s'", set_system_settings_service_name_.c_str());
    set_system_settings_client_ =
        this->create_client<ros2_interfaces::srv::SetSystemSettings>(set_system_settings_service_name_);

    set_regulator_settings_service_name_ = this->declare_parameter<std::string>(
        qt_node_constants::SET_REGULATOR_SETTINGS_SERVICE_PARAM,
        qt_node_constants::DEFAULT_SET_REGULATOR_SETTINGS_SERVICE);
    RCLCPP_INFO(this->get_logger(), "Using SetRegulatorSettings service: '%s'", set_regulator_settings_service_name_.c_str());
    set_regulator_settings_client_ =
        this->create_client<ros2_interfaces::srv::SetRegulatorSettings>(set_regulator_settings_service_name_);

    set_circuit_settings_service_name_ = this->declare_parameter<std::string>(
        qt_node_constants::SET_CIRCUIT_SETTINGS_SERVICE_PARAM,
        qt_node_constants::DEFAULT_SET_CIRCUIT_SETTINGS_SERVICE);
    RCLCPP_INFO(this->get_logger(), "Using SetCircuitSettings service: '%s'", set_circuit_settings_service_name_.c_str());
    set_circuit_settings_client_ =
        this->create_client<ros2_interfaces::srv::SetCircuitSettings>(set_circuit_settings_service_name_);

    // 初始化 Query Data Records Service Client
    query_data_records_service_name_ = this->declare_parameter<std::string>(
        qt_node_constants::QUERY_DATA_RECORDS_SERVICE_PARAM,
        qt_node_constants::DEFAULT_QUERY_DATA_RECORDS_SERVICE);
    RCLCPP_INFO(this->get_logger(), "Using QueryDataRecords service: '%s'", query_data_records_service_name_.c_str());

    query_data_records_client_ =
        this->create_client<ros2_interfaces::srv::QueryDataRecords>(query_data_records_service_name_);

    // --- 启动ROS事件循环的Qt定时器 ---
    m_ros_timer = new QTimer(this);
    connect(m_ros_timer, &QTimer::timeout, this, &QtROSNode::spin_some);
    RCLCPP_INFO(this->get_logger(), "Qt Node started and spinning.");
}

QtROSNode::~QtROSNode()
{
    RCLCPP_INFO(this->get_logger(), "Qt Node destroyed.");
}

void QtROSNode::onShutdownRequested()
{
    RCLCPP_INFO(this->get_logger(), "Shutdown requested. Cleaning up ROS node.");

    // 1. 停止所有活动！这是至关重要的一步，防止在关闭时还有ROS操作在执行。
    m_ros_timer->stop();

    // 2. 关闭 rclcpp。这会让 spin_some/spin_one 等调用立即返回。
    rclcpp::shutdown();

    RCLCPP_INFO(this->get_logger(), "ROS node cleanup complete.");

    // 3. 通知主线程，我们这边已经搞定了。
    emit shutdownFinished();
}

void QtROSNode::startTimer()
{
    m_ros_timer->start(50); // 每 100 毫秒 spin 一次

    // [新增] 启动 2 秒后执行一次查询测试，给 record_node 一点准备时间
    QTimer::singleShot(2000, this, &QtROSNode::testQueryData);
}

void QtROSNode::spin_some()
{
    rclcpp::spin_some(this->get_node_base_interface());
}

// --- 回调函数实现: Status ---

void QtROSNode::circuit_status_callback(const ros2_interfaces::msg::CircuitStatus::SharedPtr msg)
{
    // 1. 创建 C++ 数据实例
    CircuitStatusData data;

    // 2. 转换全局字段
    data.circuit_id = msg->circuit_id;
    data.control_mode = msg->control_mode;
    //RCLCPP_INFO(this->get_logger(), "received circuit_id = %d",  msg->circuit_id);

    // 3. 使用辅助函数转换嵌套的回路状态，代码清晰且可重用
    convertLoopStatusRosToQt(msg->test_loop, data.test_loop);
    convertLoopStatusRosToQt(msg->ref_loop, data.ref_loop);

    // 4. 发射信号，将整合后的数据传递给GUI线程
    emit circuitStatusReceived(data);
}

void QtROSNode::regulator_status_callback(const ros2_interfaces::msg::RegulatorStatus::SharedPtr msg)
{
    RegulatorStatusData data;
    data.regulator_id = msg->regulator_id; // 字段名变更
    data.voltage_reading = msg->voltage_reading;
    data.current_reading = msg->current_reading;
    data.voltage_direction = msg->voltage_direction;
    data.breaker_closed_switch_ack = msg->breaker_closed_switch_ack;
    data.breaker_opened_switch_ack = msg->breaker_opened_switch_ack;
    data.upper_limit_switch_on = msg->upper_limit_switch_on;
    data.lower_limit_switch_on = msg->lower_limit_switch_on;
    data.over_current_on = msg->over_current_on;
    data.over_voltage_on = msg->over_voltage_on;
    //RCLCPP_INFO(this->get_logger(), "received regulator_id = %d",  msg->regulator_id);
    emit regulatorStatusReceived(data); // 信号名变更
}

// --- 回调函数实现: Settings ---

void QtROSNode::system_settings_callback(SystemSettingsMsgPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "Forwarding system settings update.");
    emit systemSettingsReceived(msg); // Directly emit the received message
}

void QtROSNode::regulator_settings_callback(RegulatorSettingsMsgPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "Forwarding regulator settings update for ID %d.", msg->regulator_id);
    emit regulatorSettingsReceived(msg);
}

void QtROSNode::circuit_settings_callback(CircuitSettingsMsgPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "Forwarding circuit settings update for ID %d.", msg->circuit_id);
    //RCLCPP_INFO(this->get_logger(), "CircuitSettings msg:\n%s", ros2_interfaces::msg::to_yaml(*msg).c_str());
    emit circuitSettingsReceived(msg);

}

// --- 命令发送槽函数的实现 ---

void QtROSNode::onSendRegulatorOperationCommand(quint8 regulator_id, quint8 command)
{
    auto msg = std::make_unique<ros2_interfaces::msg::RegulatorOperationCommand>();
    msg->regulator_id = regulator_id;
    msg->command = command;
    RCLCPP_INFO(this->get_logger(), "Publishing Regulator Operation Command: [ID: %d, CMD: %d]", msg->regulator_id, msg->command);
    regulator_operation_command_pub_->publish(std::move(msg));
}

void QtROSNode::onSendRegulatorBreakerCommand(quint8 regulator_id, quint8 command)
{
    if (!regulator_breaker_command_client_->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "Service '%s' not available.", regulator_breaker_command_service_name_.c_str());
        emit commandResult(QString::fromStdString(regulator_breaker_command_service_name_), false, "Service not available");
        return;
    }
    auto request = std::make_shared<ros2_interfaces::srv::RegulatorBreakerCommand::Request>();
    request->regulator_id = regulator_id;
    request->command = command;

    auto response_callback = [this, srv_name = regulator_breaker_command_service_name_](rclcpp::Client<ros2_interfaces::srv::RegulatorBreakerCommand>::SharedFuture future) {
        auto response = future.get();
        emit commandResult(QString::fromStdString(srv_name), response->success, QString::fromStdString(response->message));
    };
    regulator_breaker_command_client_->async_send_request(request, response_callback);
}

void QtROSNode::onSendCircuitModeCommand(quint8 circuit_id, quint8 command)
{
    if (!circuit_mode_command_client_->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "Service '%s' not available.", circuit_mode_command_service_name_.c_str());
        emit commandResult(QString::fromStdString(circuit_mode_command_service_name_), false, "Service not available");
        return;
    }
    auto request = std::make_shared<ros2_interfaces::srv::CircuitModeCommand::Request>();
    request->circuit_id = circuit_id;
    request->command = command;

    auto response_callback = [this, srv_name = circuit_mode_command_service_name_](rclcpp::Client<ros2_interfaces::srv::CircuitModeCommand>::SharedFuture future) {
        auto response = future.get();
        emit commandResult(QString::fromStdString(srv_name), response->success, QString::fromStdString(response->message));
    };
    circuit_mode_command_client_->async_send_request(request, response_callback);
}

void QtROSNode::onSendCircuitBreakerCommand(quint8 circuit_id, quint8 command)
{
    if (!circuit_breaker_command_client_->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "Service '%s' not available.", circuit_breaker_command_service_name_.c_str());
        emit commandResult(QString::fromStdString(circuit_breaker_command_service_name_), false, "Service not available");
        return;
    }
    auto request = std::make_shared<ros2_interfaces::srv::CircuitBreakerCommand::Request>();
    request->circuit_id = circuit_id;
    request->command = command;

    auto response_callback = [this, srv_name = circuit_breaker_command_service_name_](rclcpp::Client<ros2_interfaces::srv::CircuitBreakerCommand>::SharedFuture future) {
        auto response = future.get();
        emit commandResult(QString::fromStdString(srv_name), response->success, QString::fromStdString(response->message));
    };
    circuit_breaker_command_client_->async_send_request(request, response_callback);
}

void QtROSNode::onSendClearAlarm()
{
    auto msg = std::make_unique<std_msgs::msg::Empty>();
    RCLCPP_INFO(this->get_logger(), "Publishing Clear Alarm Command.");
    clear_alarm_pub_->publish(std::move(msg));
}

// --- 设置参数的实现 ---

void QtROSNode::onSetSystemSettings(SystemSettingsData* data)
{
    // --- 1. 安全性检查：确保传入的指针有效 ---
    if (!data) {
        RCLCPP_ERROR(this->get_logger(), "onSetSystemSettings received a null pointer. Aborting request.");
        emit settingsUpdateResult(QString::fromStdString(set_system_settings_service_name_), false, "Internal error: Null data");
        return;
    }

    // --- 2. 检查服务是否可用 (异步方式) ---
    if (!set_system_settings_client_->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "Service '%s' not available.", set_system_settings_service_name_.c_str());
        emit settingsUpdateResult(QString::fromStdString(set_system_settings_service_name_), false, "Service not available");
        return;
    }

    // --- 3. 创建并填充请求 ---
    auto request = std::make_shared<ros2_interfaces::srv::SetSystemSettings::Request>();
    // --- 改动：使用 getter 方法访问数据 ---
    request->settings.sample_interval_sec = data->sample_interval_sec();
    request->settings.record_interval_min = data->record_interval_min();
    request->settings.keep_record_on_shutdown = data->keep_record_on_shutdown();

    // --- 4. 定义回调并异步发送 (逻辑不变) ---
    auto response_callback = [this, service_name = set_system_settings_service_name_](rclcpp::Client<ros2_interfaces::srv::SetSystemSettings>::SharedFuture future) {
        auto response = future.get();
        emit settingsUpdateResult(QString::fromStdString(service_name), response->success, QString::fromStdString(response->message));
    };

    set_system_settings_client_->async_send_request(request, response_callback);
}

void QtROSNode::onSetRegulatorSettings(quint8 regulator_id, RegulatorSettingsData  *data)
{
    // --- 1. 安全性检查：确保传入的指针有效 ---
    if (!data) {
        RCLCPP_ERROR(this->get_logger(), "onSetRegulatorSettings received a null pointer. Aborting request.");
        emit settingsUpdateResult(QString::fromStdString(set_regulator_settings_service_name_), false, "Internal error: Null data");
        return;
    }

    // --- 2. 检查服务是否可用 (异步方式) ---
    if (!set_regulator_settings_client_->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "Service '%s' not available.", set_regulator_settings_service_name_.c_str());
        emit settingsUpdateResult(QString::fromStdString(set_regulator_settings_service_name_), false, "Service not available");
        return;
    }

    // --- 3. 创建并填充请求 ---
    auto request = std::make_shared<ros2_interfaces::srv::SetRegulatorSettings::Request>();
    request->settings.regulator_id = regulator_id;
    // --- 改动：使用 getter 方法访问数据 ---
    request->settings.over_current_a = data->over_current_a();
    request->settings.over_voltage_v = data->over_voltage_v();
    request->settings.voltage_up_speed_percent = data->voltage_up_speed_percent();
    request->settings.voltage_down_speed_percent = data->voltage_down_speed_percent();
    request->settings.over_voltage_protection_mode = data->over_voltage_protection_mode();

    // --- 4. 定义回调并异步发送 (逻辑不变) ---
    auto response_callback = [this, service_name = set_regulator_settings_service_name_](rclcpp::Client<ros2_interfaces::srv::SetRegulatorSettings>::SharedFuture future) {
        auto response = future.get();
        if (response) {
            emit settingsUpdateResult(QString::fromStdString(service_name), response->success, QString::fromStdString(response->message));
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service '%s'", service_name.c_str());
            emit settingsUpdateResult(QString::fromStdString(service_name), false, "Failed to call service");
        }
    };

    set_regulator_settings_client_->async_send_request(request, response_callback);
}

void QtROSNode::onSetCircuitSettings(quint8 circuit_id, CircuitSettingsData* data)
{
    // --- 1. 安全性检查：确保传入的指针有效 ---
    if (!data) {
        RCLCPP_ERROR(this->get_logger(), "onSetCircuitSettings received a null pointer. Aborting request.");
        emit settingsUpdateResult(QString::fromStdString(set_circuit_settings_service_name_), false, "Internal error: Null data");
        return;
    }

    // --- 2. 检查服务是否可用 (异步方式) ---
    if (!set_circuit_settings_client_->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "Service '%s' not available.", set_circuit_settings_service_name_.c_str());
        emit settingsUpdateResult(QString::fromStdString(set_circuit_settings_service_name_), false, "Service not available");
        return;
    }

    // --- 3. 创建请求对象 ---
    auto request = std::make_shared<ros2_interfaces::srv::SetCircuitSettings::Request>();
    request->settings.circuit_id = circuit_id;

    // --- 4. 改动：调用新的转换函数，并传入 logger ---
    convertCircuitSettingsQtToRos(data, request->settings, this->get_logger());

    // --- 5. 定义异步回调并发送 (逻辑不变) ---
    auto response_callback = [this, service_name = set_circuit_settings_service_name_](rclcpp::Client<ros2_interfaces::srv::SetCircuitSettings>::SharedFuture future) {
        RCLCPP_INFO(this->get_logger(), "SetCircuitSettings -- 5");
        auto response = future.get();
        if (response) {
            emit settingsUpdateResult(QString::fromStdString(service_name), response->success, QString::fromStdString(response->message));
            RCLCPP_INFO(this->get_logger(), "SetCircuitSettings %s : %s", response->success ? "success":"failed", response->message.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service '%s'", service_name.c_str());
            emit settingsUpdateResult(QString::fromStdString(service_name), false, "Failed to call service");
        }
    };

    set_circuit_settings_client_->async_send_request(request, response_callback);
}

void QtROSNode::testQueryData()
{
    RCLCPP_INFO(this->get_logger(), ">>> STARTING QUERY DATA TEST <<<");

    // 1. 检查服务是否在线
    if (!query_data_records_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "Query service '%s' is NOT ready via wait_for_service.", query_data_records_service_name_.c_str());
        return;
    }

    // 2. 构建请求
    auto request = std::make_shared<ros2_interfaces::srv::QueryDataRecords::Request>();

    // 2.1 设置时间范围 (这里为了演示，获取当前时间的前后24小时，或者您可以写死一个具体的测试时间)
    // 假设数据库里有最近的数据
    QDateTime now = QDateTime::currentDateTime();
    QDateTime startTime = now.addDays(-2); // 前天
    QDateTime endTime = now;    // 今天

    // 格式化为 SQL 兼容格式: "yyyy-MM-dd HH:mm:ss"
    request->start_time = startTime.toString("yyyy-MM-dd HH:mm:ss").toStdString();
    request->end_time = endTime.toString("yyyy-MM-dd HH:mm:ss").toStdString();

    // 2.2 设置要查询的列 (根据接口文档)
    // 比如：查询 时间、回路ID、试验支路电流、调压器电压
    request->column_names = {
        "record_time",
        "circuit_id",
        "test_loop_current",
        "regulator_voltage"
    };

    // 2.3 设置过滤器 (0表示不过滤，查所有回路)
    request->circuit_id_filter = 0;

    RCLCPP_INFO(this->get_logger(), "Sending Request -> Start: %s, End: %s, Cols: %lu",
                request->start_time.c_str(), request->end_time.c_str(), request->column_names.size());

    // 3. 发送异步请求
    auto response_callback = [this](rclcpp::Client<ros2_interfaces::srv::QueryDataRecords>::SharedFuture future) {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), ">>> QUERY SUCCESS <<<");
            RCLCPP_INFO(this->get_logger(), "Message: %s", response->message.c_str());

            // --- [修复的部分] ---
            QStringList headerList;
            for (const std::string& h : response->header) {
                headerList.append(QString::fromStdString(h));
            }
            QString headerStr = headerList.join(" | ");
            // -------------------

            RCLCPP_INFO(this->get_logger(), "Header: [ %s ]", headerStr.toStdString().c_str());
            RCLCPP_INFO(this->get_logger(), "Rows Count: %lu", response->data_rows.size());

            // 打印前 5 行数据示例
            int count = 0;
            for (const std::string& row_std_str : response->data_rows) {
                if (count >= 5) break;

                QString row_str = QString::fromStdString(row_std_str);
                // 这里不需要复杂的分割逻辑，只是打印演示
                RCLCPP_INFO(this->get_logger(), "Row %d: %s", count + 1, row_str.toStdString().c_str());
                count++;
            }
            if (response->data_rows.size() > 5) {
                RCLCPP_INFO(this->get_logger(), "... (Remaining %lu rows omitted)", response->data_rows.size() - 5);
            }

        } else {
            RCLCPP_ERROR(this->get_logger(), ">>> QUERY FAILED <<<");
            RCLCPP_ERROR(this->get_logger(), "Reason: %s", response->message.c_str());
        }
    };

    query_data_records_client_->async_send_request(request, response_callback);
}

void QtROSNode::queryHistoryData(const QString& start_time_str, int duration_hours, const QStringList& requested_keys)
{
    if (!query_data_records_client_->service_is_ready()) {
        emit historyQueryFailed("Query service is not ready.");
        return;
    }

    QDateTime startDt = QDateTime::fromString(start_time_str, "yyyy-MM-dd HH:mm:ss");
    if (!startDt.isValid()) {
        emit historyQueryFailed("Invalid start time format.");
        return;
    }
    QDateTime endDt = startDt.addSecs(duration_hours * 3600);

    auto request = std::make_shared<ros2_interfaces::srv::QueryDataRecords::Request>();
    request->start_time = startDt.toString("yyyy-MM-dd HH:mm:ss").toStdString();
    request->end_time = endDt.toString("yyyy-MM-dd HH:mm:ss").toStdString();

    // 1. 解析 QML 传来的带有 ID 的 key (例如 "1|regulator_voltage")，提取出实际的数据库列名
    QSet<QString> db_columns;
    db_columns.insert("record_time");
    db_columns.insert("circuit_id"); // 必须请求 ID，以便区分数据属于哪条曲线

    for(const QString& key : requested_keys) {
        QStringList parts = key.split('|');
        if(parts.size() == 2) {
            db_columns.insert(parts[1]); // 提取实际列名
        }
    }

    // 填充请求
    for(const QString& col : db_columns) {
        request->column_names.push_back(col.toStdString());
    }

    request->circuit_id_filter = 0; // 0 表示查询全部回路，我们在收到数据后再按 ID 拆分

    RCLCPP_INFO(this->get_logger(), "Querying History: %s -> %s, Cols: %lu",
                request->start_time.c_str(), request->end_time.c_str(), request->column_names.size());

    // 2. 异步回调与数据解析
    auto callback = [this, requested_keys](rclcpp::Client<ros2_interfaces::srv::QueryDataRecords>::SharedFuture future) {
        auto response = future.get();
        if (!response->success) {
            emit historyQueryFailed(QString::fromStdString(response->message));
            return;
        }

        // 获取列索引映射
        QMap<QString, int> col_indices;
        for (int i = 0; i < (int)response->header.size(); ++i) {
            col_indices[QString::fromStdString(response->header[i])] = i;
        }

        int time_idx = col_indices.value("record_time", -1);
        int cid_idx = col_indices.value("circuit_id", -1);

        if (time_idx == -1 || cid_idx == -1) {
            emit historyQueryFailed("Response missing record_time or circuit_id.");
            return;
        }

        // 使用 QVariantList 存放 QVariant::fromValue(QPointF)，这是 QML 最容易接受的格式
        // 【修改点 1】: 将 QPointF 替换为纯粹的 QVariantMap，确保 QML 能够毫无障碍地读取
        QMap<QString, QVariantList> temp_points;

        // 遍历所有数据行
        for (const std::string& row_std : response->data_rows) {
            QString row_str = QString::fromStdString(row_std);
            QStringList values = row_str.split(',');

            // 获取时间和回路ID
            QString time_str = values[time_idx].trimmed();
            QString cid_str = values[cid_idx].trimmed();

            QDateTime dt = QDateTime::fromString(time_str, "yyyy-MM-dd HH:mm:ss");
            if (!dt.isValid()) continue;
            qint64 x_ms = dt.toMSecsSinceEpoch(); // X轴要求毫秒时间戳

            // 遍历 QML 请求的 keys (例如 "1|regulator_voltage")
            for (const QString& req_key : requested_keys) {
                QStringList parts = req_key.split('|');
                if (parts.size() != 2) continue;

                QString target_cid = parts[0];
                QString target_col = parts[1];

                // 只有当当前行的 circuit_id 与请求的 circuit_id 匹配时，才提取数据
                if (cid_str == target_cid) {
                    int val_idx = col_indices.value(target_col, -1);
                    if (val_idx != -1 && val_idx < values.size()) {
                        double y_val = values[val_idx].toDouble();

                        // 【修改点 2】: 使用 QVariantMap ("x", "y") 取代 QPointF
                        QVariantMap point_map;
                        point_map["x"] = x_ms;
                        point_map["y"] = y_val;
                        temp_points[req_key].append(point_map);
                    }
                }
            }
        }

        // 打印调试信息，确认 C++ 提取到了数据
        for(auto it = temp_points.begin(); it != temp_points.end(); ++it) {
            RCLCPP_INFO(this->get_logger(), "Parsed Column: %s, Point Count: %d", it.key().toStdString().c_str(), it.value().size());
        }

        // 转换为 QVariantMap 发送给 QML
        QVariantMap result_map;
        QMapIterator<QString, QVariantList> i(temp_points);
        while (i.hasNext()) {
            i.next();
            result_map[i.key()] = i.value();
        }

        emit historyDataFetched(result_map);
    };

    query_data_records_client_->async_send_request(request, callback);
}
