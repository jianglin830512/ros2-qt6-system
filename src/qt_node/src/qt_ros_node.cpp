#include "qt_node/qt_ros_node.hpp"
#include "qt_node/qt_node_constants.hpp"
#include <QDebug>

// Helper function to avoid code duplication (optional but recommended)
namespace {

void convertQtToRos(const CircuitSettingsData* qt_data, ros2_interfaces::msg::CircuitSettings& ros_msg, const rclcpp::Logger& logger)
{
    // --- 增加安全性检查并记录日志 ---
    if (!qt_data) {
        RCLCPP_ERROR(logger, "Received a null pointer for CircuitSettingsData in convertQtToRos. Conversion aborted.");
        return;
    }
    // 同样检查嵌套的指针
    if (!qt_data->test_loop() || !qt_data->ref_loop() || !qt_data->sample_params()) {
        RCLCPP_ERROR(logger, "Nested settings data (test_loop, ref_loop, or sample_params) is null. Conversion aborted.");
        return;
    }

    // --- 1. 转换试验回路 (Test Loop) ---
    // 改动：通过 -> 调用 getter 方法，这些方法返回指针，再通过 -> 调用值的 getter
    ros_msg.test_loop.start_current_a = qt_data->test_loop()->start_current_a();
    ros_msg.test_loop.max_current_a = qt_data->test_loop()->max_current_a();
    ros_msg.test_loop.current_change_range_percent = qt_data->test_loop()->current_change_range_percent();
    ros_msg.test_loop.ct_ratio = qt_data->test_loop()->ct_ratio();
    ros_msg.test_loop.start_date = qt_data->test_loop()->start_date().toString("yyyy-MM-dd").toStdString();
    ros_msg.test_loop.cycle_count = qt_data->test_loop()->cycle_count();
    ros_msg.test_loop.heating_time = qt_data->test_loop()->heating_time().toString("HH:mm").toStdString();
    ros_msg.test_loop.heating_duration_min = qt_data->test_loop()->heating_duration_min();

    // --- 2. 转换参考回路 (Reference Loop) ---
    // 改动：应用相同的模式
    ros_msg.ref_loop.start_current_a = qt_data->ref_loop()->start_current_a();
    ros_msg.ref_loop.max_current_a = qt_data->ref_loop()->max_current_a();
    ros_msg.ref_loop.current_change_range_percent = qt_data->ref_loop()->current_change_range_percent();
    ros_msg.ref_loop.ct_ratio = qt_data->ref_loop()->ct_ratio();
    ros_msg.ref_loop.start_date = qt_data->ref_loop()->start_date().toString("yyyy-MM-dd").toStdString();
    ros_msg.ref_loop.cycle_count = qt_data->ref_loop()->cycle_count();
    ros_msg.ref_loop.heating_time = qt_data->ref_loop()->heating_time().toString("HH:mm").toStdString();
    ros_msg.ref_loop.heating_duration_min = qt_data->ref_loop()->heating_duration_min();

    // --- 3. 转换试品参数 (Sample Parameters) ---
    // 改动：应用相同的模式
    ros_msg.sample_params.cable_type = qt_data->sample_params()->cable_type().toStdString();
    ros_msg.sample_params.cable_spec = qt_data->sample_params()->cable_spec().toStdString();
    ros_msg.sample_params.insulation_material = qt_data->sample_params()->insulation_material().toStdString();
    ros_msg.sample_params.insulation_thickness = static_cast<float>(qt_data->sample_params()->insulation_thickness());
}

} // End of anonymous namespace

QtROSNode::QtROSNode(const std::string &node_name, QObject *parent)
    : QObject(parent), rclcpp::Node(node_name)
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    // --- 声明和获取订阅话题参数 ---
    this->declare_parameter<std::string>(
        qt_node_constants::CIRCUIT_STATUS_TOPIC_PARAM,
        qt_node_constants::DEFAULT_CIRCUIT_STATUS_TOPIC);
    this->declare_parameter<std::string>(
        qt_node_constants::REGULATOR_STATUS_TOPIC_PARAM,
        qt_node_constants::DEFAULT_REGULATOR_STATUS_TOPIC);
    this->get_parameter(qt_node_constants::CIRCUIT_STATUS_TOPIC_PARAM, circuit_status_topic_);
    this->get_parameter(qt_node_constants::REGULATOR_STATUS_TOPIC_PARAM, regulator_status_topic_);
    RCLCPP_INFO(this->get_logger(), "Using Circuit Status Topic: '%s'", circuit_status_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Using Regulator Status Topic: '%s'", regulator_status_topic_.c_str());

    // --- 创建订阅者 ---
    circuit_status_sub_ = this->create_subscription<ros2_interfaces::msg::CircuitStatus>(
        circuit_status_topic_, qos, std::bind(&QtROSNode::circuit_status_callback, this, std::placeholders::_1));
    regulator_status_sub_ = this->create_subscription<ros2_interfaces::msg::VoltageRegulatorStatus>(
        regulator_status_topic_, qos, std::bind(&QtROSNode::voltage_regulator_status_callback, this, std::placeholders::_1));

    // --- 声明和获取发布话题参数 ---
    this->declare_parameter<std::string>(
        qt_node_constants::REGULATOR_COMMAND_TOPIC_PARAM,
        qt_node_constants::DEFAULT_REGULATOR_COMMAND_TOPIC);
    this->declare_parameter<std::string>(
        qt_node_constants::CIRCUIT_COMMAND_TOPIC_PARAM,
        qt_node_constants::DEFAULT_CIRCUIT_COMMAND_TOPIC);
    this->declare_parameter<std::string>(
        qt_node_constants::CLEAR_ALARM_TOPIC_PARAM,
        qt_node_constants::DEFAULT_CLEAR_ALARM_TOPIC);

    this->get_parameter(qt_node_constants::REGULATOR_COMMAND_TOPIC_PARAM, regulator_command_topic_);
    this->get_parameter(qt_node_constants::CIRCUIT_COMMAND_TOPIC_PARAM, circuit_command_topic_);
    this->get_parameter(qt_node_constants::CLEAR_ALARM_TOPIC_PARAM, clear_alarm_topic_);
    RCLCPP_INFO(this->get_logger(), "Using Regulator Command Topic: '%s'", regulator_command_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Using Circuit Command Topic: '%s'", circuit_command_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Using Clear Alarm Topic: '%s'", clear_alarm_topic_.c_str());

    // --- 创建发布者 ---
    regulator_command_pub_ = this->create_publisher<ros2_interfaces::msg::VoltageRegulatorCommand>(regulator_command_topic_, qos);
    circuit_command_pub_ = this->create_publisher<ros2_interfaces::msg::CircuitCommand>(circuit_command_topic_, qos);
    clear_alarm_pub_ = this->create_publisher<std_msgs::msg::Empty>(clear_alarm_topic_, qos);

    // --- Declare service name parameters ---
    this->declare_parameter<std::string>(qt_node_constants::SET_SYSTEM_SETTINGS_SERVICE_PARAM, qt_node_constants::DEFAULT_SET_SYSTEM_SETTINGS_SERVICE);
    this->declare_parameter<std::string>(qt_node_constants::SET_REGULATOR_SETTINGS_SERVICE_PARAM, qt_node_constants::DEFAULT_SET_REGULATOR_SETTINGS_SERVICE);
    this->declare_parameter<std::string>(qt_node_constants::SET_CIRCUIT_SETTINGS_SERVICE_PARAM, qt_node_constants::DEFAULT_SET_CIRCUIT_SETTINGS_SERVICE);

    this->get_parameter(qt_node_constants::SET_SYSTEM_SETTINGS_SERVICE_PARAM, set_system_settings_service_name_);
    this->get_parameter(qt_node_constants::SET_REGULATOR_SETTINGS_SERVICE_PARAM, set_regulator_settings_service_name_);
    this->get_parameter(qt_node_constants::SET_CIRCUIT_SETTINGS_SERVICE_PARAM, set_circuit_settings_service_name_);

    RCLCPP_INFO(this->get_logger(), "Using SetSystemSettings service: '%s'", set_system_settings_service_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Using SetRegulatorSettings service: '%s'", set_regulator_settings_service_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Using SetCircuitSettings service: '%s'", set_circuit_settings_service_name_.c_str());

    // --- Create Service Clients ---
    set_system_settings_client_ = this->create_client<ros2_interfaces::srv::SetSystemSettings>(set_system_settings_service_name_);
    set_regulator_settings_client_ = this->create_client<ros2_interfaces::srv::SetRegulatorSettings>(set_regulator_settings_service_name_);
    set_circuit_settings_client_ = this->create_client<ros2_interfaces::srv::SetCircuitSettings>(set_circuit_settings_service_name_);

    // QT_ROS_NODE start up
    m_ros_timer = new QTimer(this);
    connect(m_ros_timer, &QTimer::timeout, this, &QtROSNode::spin_some);
    RCLCPP_INFO(this->get_logger(), "Qt Node 启动.");
}

QtROSNode::~QtROSNode()
{
    RCLCPP_INFO(this->get_logger(), "Qt Node 销毁.");
}

void QtROSNode::startTimer()
{
    m_ros_timer->start(100); // 每 100 毫秒 spin 一次
}

void QtROSNode::spin_some()
{
    rclcpp::spin_some(this->get_node_base_interface());
}

// --- 回调函数实现 ---

void QtROSNode::circuit_status_callback(const ros2_interfaces::msg::CircuitStatus::SharedPtr msg)
{
    // 1. 创建POD实例
    CircuitStatusData data;

    // 2. 将ROS消息数据复制到POD中
    data.circuit_id = msg->circuit_id;
    data.test_current = msg->test_current;
    data.ref_current = msg->ref_current;

    // 复制数组
    data.test_temperature_array.reserve(16);
    for(const auto& temp : msg->test_temperature_array) {
        data.test_temperature_array.append(temp);
    }
    data.ref_temperature_array.reserve(8);
    for(const auto& temp : msg->ref_temperature_array) {
        data.ref_temperature_array.append(temp);
    }

    data.elapsed_heating_time_sec = msg->elapsed_heating_time.sec;
    data.remaining_heating_time_sec = msg->remaining_heating_time.sec;
    data.completed_cycle_count = msg->completed_cycle_count;
    data.remaining_cycle_count = msg->remaining_cycle_count;
    data.control_mode = msg->control_mode;
    data.circuit_enabled = msg->circuit_enabled;
    data.breaker_closed_switch_ack = msg->breaker_closed_switch_ack;
    data.breaker_opened_switch_ack = msg->breaker_opened_switch_ack;

    // 3. 发射信号，将数据传递给GUI线程
    emit circuitStatusReceived(data);
}

void QtROSNode::voltage_regulator_status_callback(const ros2_interfaces::msg::VoltageRegulatorStatus::SharedPtr msg)
{
    // 1. 创建POD实例
    VoltageRegulatorStatusData data;

    // 2. 将ROS消息数据复制到POD中
    data.voltage_regulator_id = msg->voltage_regulator_id;
    data.voltage_reading = msg->voltage_reading;
    data.current_reading = msg->current_reading;
    data.voltage_direction = msg->voltage_direction;
    data.breaker_closed_switch_ack = msg->breaker_closed_switch_ack;
    data.breaker_opened_switch_ack = msg->breaker_opened_switch_ack;
    data.upper_limit_switch_on = msg->upper_limit_switch_on;
    data.lower_limit_switch_on = msg->lower_limit_switch_on;
    data.over_current_on = msg->over_current_on;
    data.over_voltage_on = msg->over_voltage_on;

    // 3. 发射信号，将数据传递给GUI线程
    emit voltageRegulatorStatusReceived(data);
}

// --- 命令发送槽函数的实现 ---

void QtROSNode::onSendRegulatorCommand(quint8 regulator_id, quint8 command)
{
    auto msg = std::make_unique<ros2_interfaces::msg::VoltageRegulatorCommand>();
    msg->regulator_id = regulator_id;
    msg->command = command;
    RCLCPP_INFO(this->get_logger(), "Publishing Regulator Command: [ID: %d, CMD: %d]", msg->regulator_id, msg->command);
    regulator_command_pub_->publish(std::move(msg));
}

void QtROSNode::onSendCircuitCommand(quint8 circuit_id, quint8 command)
{
    auto msg = std::make_unique<ros2_interfaces::msg::CircuitCommand>();
    msg->circuit_id = circuit_id;
    msg->command = command;
    RCLCPP_INFO(this->get_logger(), "Publishing Circuit Command: [ID: %d, CMD: %d]", msg->circuit_id, msg->command);
    circuit_command_pub_->publish(std::move(msg));
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

void QtROSNode::onSetRegulatorSettings(quint8 regulator_id, VoltageRegulatorSettingsData *data)
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
    request->regulator_id = regulator_id;
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
    request->circuit_id = circuit_id;

    // --- 4. 改动：调用新的转换函数，并传入 logger ---
    convertQtToRos(data, request->settings, this->get_logger());

    // --- 5. 定义异步回调并发送 (逻辑不变) ---
    auto response_callback = [this, service_name = set_circuit_settings_service_name_](rclcpp::Client<ros2_interfaces::srv::SetCircuitSettings>::SharedFuture future) {
        auto response = future.get();
        if (response) {
            emit settingsUpdateResult(QString::fromStdString(service_name), response->success, QString::fromStdString(response->message));
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service '%s'", service_name.c_str());
            emit settingsUpdateResult(QString::fromStdString(service_name), false, "Failed to call service");
        }
    };

    set_circuit_settings_client_->async_send_request(request, response_callback);
}
