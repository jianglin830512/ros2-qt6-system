#include "qt_node/qt_ros_node.hpp"
#include "qt_node/qt_node_constants.hpp"
#include <QDebug>
#include <QPointF>
#include <QStringConverter>
#include <QUrl>
#include <QtGlobal>

namespace {

void convertLoopStatusRosToQt(const ros2_interfaces::msg::LoopStatus& ros_loop, LoopStatusData& qt_loop)
{
    qt_loop.is_heat = ros_loop.is_heat;
    qt_loop.completed_cycle_count = ros_loop.completed_cycle_count;
    qt_loop.remaining_cycle_count = ros_loop.remaining_cycle_count;
    qt_loop.elapsed_heating_time_sec = ros_loop.elapsed_heating_time.sec;
    qt_loop.remaining_heating_time_sec = ros_loop.remaining_heating_time.sec;
    qt_loop.current = ros_loop.hardware_loop_status.current;
    qt_loop.breaker_closed_switch_ack = ros_loop.hardware_loop_status.breaker_closed_switch_ack;
    qt_loop.breaker_opened_switch_ack = ros_loop.hardware_loop_status.breaker_opened_switch_ack;
    qt_loop.plc_control_mode = ros_loop.hardware_loop_status.plc_control_mode;
    // 【修改点】温度不再属于支路，此处不处理 temperature_array
}

void convertLoopSettingsQtToRos(const LoopSettingsData* qt_loop_settings, ros2_interfaces::msg::LoopSettings& ros_loop_settings)
{
    if (!qt_loop_settings) return;
    ros_loop_settings.hardware_loop_settings.start_current_a = qt_loop_settings->start_current_a();
    ros_loop_settings.hardware_loop_settings.max_current_a = qt_loop_settings->max_current_a();
    ros_loop_settings.hardware_loop_settings.current_change_range_percent = qt_loop_settings->current_change_range_percent();
    ros_loop_settings.hardware_loop_settings.ct_ratio = qt_loop_settings->ct_ratio();

    ros_loop_settings.cycle_count = qt_loop_settings->cycle_count();

    QDateTime qt_date = qt_loop_settings->start_date();
    QDate date_part = qt_date.date();
    QDateTime pure_date(date_part, QTime(0, 0, 0));

    ros_loop_settings.start_date.sec = static_cast<int32_t>(pure_date.toSecsSinceEpoch());
    ros_loop_settings.start_date.nanosec = 0;

    ros_loop_settings.heating_time.sec = qt_loop_settings->heating_start_time_sec();
    ros_loop_settings.heating_time.nanosec = 0;

    ros_loop_settings.heating_duration.sec = qt_loop_settings->heating_duration_sec();
    ros_loop_settings.heating_duration.nanosec = 0;

    ros_loop_settings.enabled = qt_loop_settings->enabled();
    ros_loop_settings.auto_strategy = qt_loop_settings->auto_strategy();
}

void convertCircuitSettingsQtToRos(const CircuitSettingsData* qt_circuit_settings, ros2_interfaces::msg::CircuitSettings& ros_circuit_settings, const rclcpp::Logger& logger)
{
    if (!qt_circuit_settings) return;
    if (!qt_circuit_settings->test_loop() || !qt_circuit_settings->ref_loop() || !qt_circuit_settings->sample_cable()) return;

    convertLoopSettingsQtToRos(qt_circuit_settings->test_loop(), ros_circuit_settings.test_loop);
    convertLoopSettingsQtToRos(qt_circuit_settings->ref_loop(), ros_circuit_settings.ref_loop);

    ros_circuit_settings.sample_cable.id = qt_circuit_settings->sample_cable()->id();
    ros_circuit_settings.sample_cable.name = qt_circuit_settings->sample_cable()->name().toStdString();
    ros_circuit_settings.sample_cable.core_diameter = qt_circuit_settings->sample_cable()->core_diameter();
    ros_circuit_settings.sample_cable.core_material = qt_circuit_settings->sample_cable()->core_material().toStdString();
    ros_circuit_settings.sample_cable.insulation_thickness = qt_circuit_settings->sample_cable()->insulation_thickness();
    ros_circuit_settings.sample_cable.insulation_material = qt_circuit_settings->sample_cable()->insulation_material().toStdString();
    ros_circuit_settings.sample_cable.voltage_grade = qt_circuit_settings->sample_cable()->voltage_grade();
    ros_circuit_settings.sample_cable.system_format = qt_circuit_settings->sample_cable()->system_format();
    ros_circuit_settings.sample_cable.notes = qt_circuit_settings->sample_cable()->notes().toStdString();
    ros_circuit_settings.sample_cable.last_modified = qt_circuit_settings->sample_cable()->last_modified().toStdString();
    ros_circuit_settings.test_id = qt_circuit_settings->test_id();
}

} // End of anonymous namespace

QtROSNode::QtROSNode(const std::string &node_name, QObject *parent)
    : QObject(parent), rclcpp::Node(node_name)
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    circuit_status_topic_ = this->declare_parameter<std::string>(
        qt_node_constants::CIRCUIT_STATUS_TOPIC_PARAM,
        qt_node_constants::DEFAULT_CIRCUIT_STATUS_TOPIC);
    circuit_status_sub_ = this->create_subscription<ros2_interfaces::msg::CircuitStatus>(
        circuit_status_topic_, qos, std::bind(&QtROSNode::circuit_status_callback, this, std::placeholders::_1));

    regulator_status_topic_ = this->declare_parameter<std::string>(
        qt_node_constants::REGULATOR_STATUS_TOPIC_PARAM,
        qt_node_constants::DEFAULT_REGULATOR_STATUS_TOPIC);
    regulator_status_sub_ = this->create_subscription<ros2_interfaces::msg::RegulatorStatus>(
        regulator_status_topic_, qos, std::bind(&QtROSNode::regulator_status_callback, this, std::placeholders::_1));

    system_status_topic_ = this->declare_parameter<std::string>(
        qt_node_constants::SYSTEM_STATUS_TOPIC_PARAM,
        qt_node_constants::DEFAULT_SYSTEM_STATUS_TOPIC);
    system_status_sub_ = this->create_subscription<ros2_interfaces::msg::SystemStatus>(
        system_status_topic_, qos, std::bind(&QtROSNode::system_status_callback, this, std::placeholders::_1));

    system_settings_topic_ = this->declare_parameter<std::string>(
        qt_node_constants::SYSTEM_SETTINGS_TOPIC_PARAM,
        qt_node_constants::DEFAULT_SYSTEM_SETTINGS_TOPIC);
    system_settings_sub_ = this->create_subscription<ros2_interfaces::msg::SystemSettings>(
        system_settings_topic_, qos, std::bind(&QtROSNode::system_settings_callback, this, std::placeholders::_1));

    regulator_settings_topic_ = this->declare_parameter<std::string>(
        qt_node_constants::REGULATOR_SETTINGS_TOPIC_PARAM,
        qt_node_constants::DEFAULT_REGULATOR_SETTINGS_TOPIC);
    regulator_settings_sub_ = this->create_subscription<ros2_interfaces::msg::RegulatorSettings>(
        regulator_settings_topic_, qos, std::bind(&QtROSNode::regulator_settings_callback, this, std::placeholders::_1));

    circuit_settings_topic_ = this->declare_parameter<std::string>(
        qt_node_constants::CIRCUIT_SETTINGS_TOPIC_PARAM,
        qt_node_constants::DEFAULT_CIRCUIT_SETTINGS_TOPIC);
    circuit_settings_sub_ = this->create_subscription<ros2_interfaces::msg::CircuitSettings>(
        circuit_settings_topic_, qos, std::bind(&QtROSNode::circuit_settings_callback, this, std::placeholders::_1));

    regulator_operation_command_topic_ = this->declare_parameter<std::string>(
        qt_node_constants::REGULATOR_OPERATION_COMMAND_TOPIC_PARAM,
        qt_node_constants::DEFAULT_REGULATOR_OPERATION_COMMAND_TOPIC);
    regulator_operation_command_pub_ =
        this->create_publisher<ros2_interfaces::msg::RegulatorOperationCommand>(regulator_operation_command_topic_, qos);

    clear_alarm_topic_ = this->declare_parameter<std::string>(
        qt_node_constants::CLEAR_ALARM_TOPIC_PARAM,
        qt_node_constants::DEFAULT_CLEAR_ALARM_TOPIC);
    clear_alarm_pub_ = this->create_publisher<std_msgs::msg::Empty>(clear_alarm_topic_, qos);

    regulator_breaker_command_service_name_ = this->declare_parameter<std::string>(
        qt_node_constants::REGULATOR_BREAKER_COMMAND_SERVICE_PARAM,
        qt_node_constants::DEFAULT_REGULATOR_BREAKER_COMMAND_SERVICE);
    regulator_breaker_command_client_ =
        this->create_client<ros2_interfaces::srv::RegulatorBreakerCommand>(regulator_breaker_command_service_name_);

    circuit_breaker_command_service_name_ = this->declare_parameter<std::string>(
        qt_node_constants::CIRCUIT_BREAKER_COMMAND_SERVICE_PARAM,
        qt_node_constants::DEFAULT_CIRCUIT_BREAKER_COMMAND_SERVICE);
    circuit_breaker_command_client_ =
        this->create_client<ros2_interfaces::srv::CircuitBreakerCommand>(circuit_breaker_command_service_name_);

    set_system_settings_service_name_ = this->declare_parameter<std::string>(
        qt_node_constants::SET_SYSTEM_SETTINGS_SERVICE_PARAM,
        qt_node_constants::DEFAULT_SET_SYSTEM_SETTINGS_SERVICE);
    set_system_settings_client_ =
        this->create_client<ros2_interfaces::srv::SetSystemSettings>(set_system_settings_service_name_);

    set_regulator_settings_service_name_ = this->declare_parameter<std::string>(
        qt_node_constants::SET_REGULATOR_SETTINGS_SERVICE_PARAM,
        qt_node_constants::DEFAULT_SET_REGULATOR_SETTINGS_SERVICE);
    set_regulator_settings_client_ =
        this->create_client<ros2_interfaces::srv::SetRegulatorSettings>(set_regulator_settings_service_name_);

    set_circuit_settings_service_name_ = this->declare_parameter<std::string>(
        qt_node_constants::SET_CIRCUIT_SETTINGS_SERVICE_PARAM,
        qt_node_constants::DEFAULT_SET_CIRCUIT_SETTINGS_SERVICE);
    set_circuit_settings_client_ =
        this->create_client<ros2_interfaces::srv::SetCircuitSettings>(set_circuit_settings_service_name_);

    query_data_records_service_name_ = this->declare_parameter<std::string>(
        qt_node_constants::QUERY_DATA_RECORDS_SERVICE_PARAM,
        qt_node_constants::DEFAULT_QUERY_DATA_RECORDS_SERVICE);
    query_data_records_client_ =
        this->create_client<ros2_interfaces::srv::QueryDataRecords>(query_data_records_service_name_);

    std::string list_test_records_service_name = this->declare_parameter<std::string>(
        qt_node_constants::LIST_TEST_RECORDS_SERVICE_PARAM,
        qt_node_constants::DEFAULT_LIST_TEST_RECORDS_SERVICE);
    list_test_records_client_ = this->create_client<ros2_interfaces::srv::ListTestRecords>(list_test_records_service_name);

    std::string save_test_record_service_name = this->declare_parameter<std::string>(
        qt_node_constants::SAVE_TEST_RECORD_SERVICE_PARAM,
        qt_node_constants::DEFAULT_SAVE_TEST_RECORD_SERVICE);
    save_test_record_client_ = this->create_client<ros2_interfaces::srv::SaveTestRecord>(save_test_record_service_name);

    std::string delete_test_record_service_name = this->declare_parameter<std::string>(
        qt_node_constants::DELETE_TEST_RECORD_SERVICE_PARAM,
        qt_node_constants::DEFAULT_DELETE_TEST_RECORD_SERVICE);
    delete_test_record_client_ = this->create_client<ros2_interfaces::srv::DeleteTestRecord>(delete_test_record_service_name);

    save_cable_service_name_ = this->declare_parameter<std::string>(
        qt_node_constants::SAVE_CABLE_SERVICE_PARAM,
        qt_node_constants::DEFAULT_SAVE_CABLE_SERVICE);
    save_cable_client_ = this->create_client<ros2_interfaces::srv::SaveCable>(save_cable_service_name_);

    delete_cable_service_name_ = this->declare_parameter<std::string>(
        qt_node_constants::DELETE_CABLE_SERVICE_PARAM,
        qt_node_constants::DEFAULT_DELETE_CABLE_SERVICE);
    delete_cable_client_ = this->create_client<ros2_interfaces::srv::DeleteCable>(delete_cable_service_name_);

    list_cables_service_name_ = this->declare_parameter<std::string>(
        qt_node_constants::LIST_CABLES_SERVICE_PARAM,
        qt_node_constants::DEFAULT_LIST_CABLES_SERVICE);
    list_cables_client_ = this->create_client<ros2_interfaces::srv::ListCables>(list_cables_service_name_);

    m_ros_timer = new QTimer(this);
    connect(m_ros_timer, &QTimer::timeout, this, &QtROSNode::spin_some);
}

QtROSNode::~QtROSNode() {}

void QtROSNode::onShutdownRequested() {
    if (m_ros_timer) { m_ros_timer->stop(); delete m_ros_timer; m_ros_timer = nullptr; }
    rclcpp::shutdown(); emit shutdownFinished();
}
void QtROSNode::startTimer() { m_ros_timer->start(50); }
void QtROSNode::spin_some() { rclcpp::spin_some(this->get_node_base_interface()); }

void QtROSNode::circuit_status_callback(const ros2_interfaces::msg::CircuitStatus::SharedPtr msg)
{
    CircuitStatusData data;
    data.circuit_id = msg->circuit_id;
    convertLoopStatusRosToQt(msg->test_loop, data.test_loop);
    convertLoopStatusRosToQt(msg->ref_loop, data.ref_loop);

    // 【修改点】映射回路级的 40路 温度
    data.temperature_array.resize(msg->temperature_array.size());
    std::copy(msg->temperature_array.begin(), msg->temperature_array.end(), data.temperature_array.begin());

    emit circuitStatusReceived(data);
}

void QtROSNode::regulator_status_callback(const ros2_interfaces::msg::RegulatorStatus::SharedPtr msg) {
    RegulatorStatusData data;
    data.regulator_id = msg->regulator_id; data.voltage_reading = msg->voltage_reading;
    data.current_reading = msg->current_reading; data.voltage_direction = msg->voltage_direction;
    data.breaker_closed_switch_ack = msg->breaker_closed_switch_ack; data.breaker_opened_switch_ack = msg->breaker_opened_switch_ack;
    data.upper_limit_switch_on = msg->upper_limit_switch_on; data.lower_limit_switch_on = msg->lower_limit_switch_on;
    data.over_current_on = msg->over_current_on; data.over_voltage_on = msg->over_voltage_on;
    emit regulatorStatusReceived(data);
}

void QtROSNode::system_status_callback(const ros2_interfaces::msg::SystemStatus::SharedPtr msg) {
    SystemStatusData data;
    data.is_remote = msg->hardware_system_status.is_remote;
    data.emergency_stop_on = msg->hardware_system_status.emergency_stop_on;
    data.system_state = msg->system_state;
    data.circuit_work_status = msg->circuit_work_status;
    data.hardware_connected = msg->hardware_connected;
    data.plc_connected = msg->hardware_system_status.plc_connected;
    data.temp_monitor_connected = msg->hardware_system_status.temp_monitor_connected;
    data.temp_monitor_type = msg->hardware_system_status.temp_monitor_type;
    current_temp_monitor_type_ = msg->hardware_system_status.temp_monitor_type;
    emit systemStatusReceived(data);
}

void QtROSNode::system_settings_callback(SystemSettingsMsgPtr msg) { emit systemSettingsReceived(msg); }
void QtROSNode::regulator_settings_callback(RegulatorSettingsMsgPtr msg) { emit regulatorSettingsReceived(msg); }
void QtROSNode::circuit_settings_callback(CircuitSettingsMsgPtr msg) { emit circuitSettingsReceived(msg); }

void QtROSNode::onSendRegulatorOperationCommand(quint8 regulator_id, quint8 command) {
    auto msg = std::make_unique<ros2_interfaces::msg::RegulatorOperationCommand>();
    msg->regulator_id = regulator_id; msg->command = command;
    regulator_operation_command_pub_->publish(std::move(msg));
}

void QtROSNode::onSendRegulatorBreakerCommand(quint8 regulator_id, quint8 command) {
    if (!regulator_breaker_command_client_->service_is_ready()) { emit commandResult(QString::fromStdString(regulator_breaker_command_service_name_), false, "Service not available"); return; }
    auto request = std::make_shared<ros2_interfaces::srv::RegulatorBreakerCommand::Request>();
    request->regulator_id = regulator_id; request->command = command;
    auto response_callback = [this, srv_name = regulator_breaker_command_service_name_](rclcpp::Client<ros2_interfaces::srv::RegulatorBreakerCommand>::SharedFuture future) {
        auto response = future.get(); emit commandResult(QString::fromStdString(srv_name), response->success, QString::fromStdString(response->message));
    };
    regulator_breaker_command_client_->async_send_request(request, response_callback);
}

void QtROSNode::onSendCircuitBreakerCommand(quint8 circuit_id, quint8 command) {
    if (!circuit_breaker_command_client_->service_is_ready()) { emit commandResult(QString::fromStdString(circuit_breaker_command_service_name_), false, "Service not available"); return; }
    auto request = std::make_shared<ros2_interfaces::srv::CircuitBreakerCommand::Request>();
    request->circuit_id = circuit_id; request->command = command;
    auto response_callback = [this, srv_name = circuit_breaker_command_service_name_](rclcpp::Client<ros2_interfaces::srv::CircuitBreakerCommand>::SharedFuture future) {
        auto response = future.get(); emit commandResult(QString::fromStdString(srv_name), response->success, QString::fromStdString(response->message));
    };
    circuit_breaker_command_client_->async_send_request(request, response_callback);
}

void QtROSNode::onSendClearAlarm() {
    auto msg = std::make_unique<std_msgs::msg::Empty>();
    clear_alarm_pub_->publish(std::move(msg));
}

void QtROSNode::onSetSystemSettings(SystemSettingsData* data) {
    if (!data) return;
    if (!set_system_settings_client_->service_is_ready()) { emit settingsUpdateResult(QString::fromStdString(set_system_settings_service_name_), false, "Service not available"); return; }
    auto request = std::make_shared<ros2_interfaces::srv::SetSystemSettings::Request>();
    request->settings.sample_interval_sec = data->sample_interval_sec();
    request->settings.record_interval_min = data->record_interval_min();
    request->settings.keep_record_on_shutdown = data->keep_record_on_shutdown();
    request->settings.auto_on = data->auto_on();
    auto response_callback = [this, service_name = set_system_settings_service_name_](rclcpp::Client<ros2_interfaces::srv::SetSystemSettings>::SharedFuture future) {
        auto response = future.get(); emit settingsUpdateResult(QString::fromStdString(service_name), response->success, QString::fromStdString(response->message));
    };
    set_system_settings_client_->async_send_request(request, response_callback);
}

void QtROSNode::onSetRegulatorSettings(quint8 regulator_id, RegulatorSettingsData  *data) {
    if (!data) return;
    if (!set_regulator_settings_client_->service_is_ready()) { emit settingsUpdateResult(QString::fromStdString(set_regulator_settings_service_name_), false, "Service not available"); return; }
    auto request = std::make_shared<ros2_interfaces::srv::SetRegulatorSettings::Request>();
    request->settings.regulator_id = regulator_id;
    request->settings.over_current_a = data->over_current_a(); request->settings.over_voltage_v = data->over_voltage_v();
    request->settings.voltage_up_speed_percent = data->voltage_up_speed_percent(); request->settings.voltage_down_speed_percent = data->voltage_down_speed_percent();
    request->settings.over_voltage_protection_mode = data->over_voltage_protection_mode();
    auto response_callback = [this, service_name = set_regulator_settings_service_name_](rclcpp::Client<ros2_interfaces::srv::SetRegulatorSettings>::SharedFuture future) {
        auto response = future.get(); emit settingsUpdateResult(QString::fromStdString(service_name), response->success, QString::fromStdString(response->message));
    };
    set_regulator_settings_client_->async_send_request(request, response_callback);
}

void QtROSNode::onSetCircuitSettings(quint8 circuit_id, CircuitSettingsData* data) {
    if (!data) return;
    if (!set_circuit_settings_client_->service_is_ready()) { emit settingsUpdateResult(QString::fromStdString(set_circuit_settings_service_name_), false, "Service not available"); return; }
    auto request = std::make_shared<ros2_interfaces::srv::SetCircuitSettings::Request>();
    request->settings.circuit_id = circuit_id;
    convertCircuitSettingsQtToRos(data, request->settings, this->get_logger());
    auto response_callback = [this, service_name = set_circuit_settings_service_name_](rclcpp::Client<ros2_interfaces::srv::SetCircuitSettings>::SharedFuture future) {
        auto response = future.get(); emit settingsUpdateResult(QString::fromStdString(service_name), response->success, QString::fromStdString(response->message));
    };
    set_circuit_settings_client_->async_send_request(request, response_callback);
}

void QtROSNode::queryHistoryData(const QString& start_time_str, int duration_hours, const QStringList& requested_keys) {
    if (!query_data_records_client_->service_is_ready()) { emit historyQueryFailed("Query service is not ready."); return; }
    QDateTime startDt = QDateTime::fromString(start_time_str, "yyyy-MM-dd HH:mm:ss");
    if (!startDt.isValid()) { emit historyQueryFailed("Invalid start time format."); return; }
    QDateTime endDt = startDt.addSecs(duration_hours * 3600);

    auto request = std::make_shared<ros2_interfaces::srv::QueryDataRecords::Request>();
    request->start_time = startDt.toString("yyyy-MM-dd HH:mm:ss").toStdString();
    request->end_time = endDt.toString("yyyy-MM-dd HH:mm:ss").toStdString();

    QSet<QString> db_columns;
    db_columns.insert("record_time"); db_columns.insert("circuit_id");
    for(const QString& key : requested_keys) {
        QStringList parts = key.split('|');
        if(parts.size() == 2) db_columns.insert(parts[1]);
    }
    for(const QString& col : db_columns) request->column_names.push_back(col.toStdString());
    request->circuit_id_filter = 0;

    auto callback = [this, requested_keys](rclcpp::Client<ros2_interfaces::srv::QueryDataRecords>::SharedFuture future) {
        auto response = future.get();
        if (!response->success) { emit historyQueryFailed(QString::fromStdString(response->message)); return; }
        QMap<QString, int> col_indices;
        for (int i = 0; i < (int)response->header.size(); ++i) col_indices[QString::fromStdString(response->header[i])] = i;

        int time_idx = col_indices.value("record_time", -1); int cid_idx = col_indices.value("circuit_id", -1);
        if (time_idx == -1 || cid_idx == -1) { emit historyQueryFailed("Response missing record_time or circuit_id."); return; }

        QMap<QString, QVariantList> temp_points;
        for (const std::string& row_std : response->data_rows) {
            QString row_str = QString::fromStdString(row_std);
            QStringList values = row_str.split(',');
            QString time_str = values[time_idx].trimmed(); QString cid_str = values[cid_idx].trimmed();
            QDateTime dt = QDateTime::fromString(time_str, "yyyy-MM-dd HH:mm:ss");
            if (!dt.isValid()) continue;
            qint64 x_ms = dt.toMSecsSinceEpoch();

            for (const QString& req_key : requested_keys) {
                QStringList parts = req_key.split('|');
                if (parts.size() != 2) continue;
                if (cid_str == parts[0]) {
                    int val_idx = col_indices.value(parts[1], -1);
                    if (val_idx != -1 && val_idx < values.size()) {
                        QVariantMap point_map; point_map["x"] = x_ms; point_map["y"] = values[val_idx].toDouble();
                        temp_points[req_key].append(point_map);
                    }
                }
            }
        }
        QVariantMap result_map;
        QMapIterator<QString, QVariantList> i(temp_points);
        while (i.hasNext()) { i.next(); result_map[i.key()] = i.value(); }
        emit historyDataFetched(result_map);
    };
    query_data_records_client_->async_send_request(request, callback);
}

void QtROSNode::queryTableData(const QString& start_time_str, int duration_hours, int circuit_id) {
    if (!query_data_records_client_->service_is_ready()) {
        emit tableQueryFailed("Query service is not ready.");
        return;
    }

    QDateTime startDt = QDateTime::fromString(start_time_str, "yyyy-MM-dd HH:mm:ss");
    if (!startDt.isValid()) {
        emit tableQueryFailed("Invalid start time format.");
        return;
    }
    QDateTime endDt = startDt.addSecs(duration_hours * 3600);

    auto request = std::make_shared<ros2_interfaces::srv::QueryDataRecords::Request>();
    request->start_time = startDt.toString("yyyy-MM-dd HH:mm:ss").toStdString();
    request->end_time = endDt.toString("yyyy-MM-dd HH:mm:ss").toStdString();

    QStringList target_columns = {
        "record_time",
        "regulator_1_voltage", "regulator_1_current",
        "regulator_2_voltage", "regulator_2_current",
        "test_loop_current", "ref_loop_current"
    };

    // 【核心修正点】动态判定通道数、卡数和起始卡号
    // current_temp_monitor_type_ == 1 表示 TEMP_DEV_TP1100_64CH (即每个回路32路)
    int num_channels = (current_temp_monitor_type_ == 1) ? 32 : 40;
    int num_cards = (current_temp_monitor_type_ == 1) ? 4 : 5;

    // 如果是回路1，永远从卡1开始。
    // 如果是回路2，如果是32路设备则从卡5开始，如果是40路设备则从卡6开始。
    int start_card = (circuit_id == 1) ? 1 : ((current_temp_monitor_type_ == 1) ? 5 : 6);

    // 追加温度通道数据库字段 circuit_temp01, circuit_temp02...
    for (int i = 1; i <= num_channels; ++i) {
        target_columns.append(QString("circuit_temp%1").arg(i, 2, 10, QChar('0')));
    }

    for (const QString& col : target_columns) {
        request->column_names.push_back(col.toStdString());
    }
    request->circuit_id_filter = circuit_id;

    // 回调函数，捕获必要的局部变量用于生成表头
    auto callback = [this, target_columns, num_cards, start_card](rclcpp::Client<ros2_interfaces::srv::QueryDataRecords>::SharedFuture future) {
        auto response = future.get();
        if (!response->success) {
            emit tableQueryFailed(QString::fromStdString(response->message));
            return;
        }

        QStringList headers = {
            "记录时间",
            "主调压器\n电压(V)", "主调压器\n电流(A)",
            "辅调压器\n电压(V)", "辅调压器\n电流(A)",
            "试验回路\n电流(A)", "模拟回路\n电流(A)"
        };

        // 动态映射表格头为 采集卡x-y
        for (int r = 0; r < num_cards; ++r) {
            for (int c = 1; c <= 8; ++c) {
                headers.append(QString("采集卡%1\n通道%2(℃)").arg(start_card + r).arg(c));
            }
        }

        QMap<QString, int> col_indices;
        for (int i = 0; i < (int)response->header.size(); ++i) {
            col_indices[QString::fromStdString(response->header[i])] = i;
        }

        QVariantList rows;
        for (const std::string& row_std : response->data_rows) {
            QString row_str = QString::fromStdString(row_std);
            QStringList values = row_str.split(',');
            QVariantList row_data;

            for (const QString& col_name : target_columns) {
                int idx = col_indices.value(col_name, -1);
                if (idx != -1 && idx < values.size()) {
                    if (col_name == "record_time") {
                        row_data.append(values[idx].trimmed());
                    } else {
                        // 数值保留1位小数
                        row_data.append(QString::number(values[idx].toDouble(), 'f', 1));
                    }
                } else {
                    row_data.append("-");
                }
            }

            // 兼容可能有额外未预期的列 (保护性代码)
            if (response->header.size() > target_columns.size()) {
                for (int i = 0; i < (int)response->header.size(); ++i) {
                    QString extra_col = QString::fromStdString(response->header[i]);
                    if (!target_columns.contains(extra_col)) {
                        if (rows.isEmpty()) headers.append(extra_col);
                        if (i < values.size()) row_data.append(values[i].trimmed());
                        else row_data.append("-");
                    }
                }
            }
            rows.append(QVariant::fromValue(row_data));
        }

        QVariantMap result_map;
        result_map["headers"] = headers;
        result_map["rows"] = rows;
        emit tableDataFetched(result_map);
    };

    query_data_records_client_->async_send_request(request, callback);
}

void QtROSNode::exportDataRecords(const QString& start_date, const QString& end_date, int circuit_id, const QString& file_path) {
    if (current_export_task_.active) { emit exportFinished(false, "已有导出任务正在进行中！"); return; }

    QUrl url(file_path);
    current_export_task_.file = new QFile(url.isLocalFile() ? url.toLocalFile() : file_path);
    if (!current_export_task_.file->open(QIODevice::WriteOnly | QIODevice::Text)) {
        emit exportFinished(false, "无法打开文件以写入数据！"); delete current_export_task_.file; current_export_task_.file = nullptr; return;
    }

    current_export_task_.final_start = QDateTime::fromString(start_date + " 00:00:00", "yyyy-MM-dd HH:mm:ss");
    current_export_task_.final_end = QDateTime::fromString(end_date + " 23:59:59", "yyyy-MM-dd HH:mm:ss");
    if (current_export_task_.final_start > current_export_task_.final_end) {
        current_export_task_.file->close(); delete current_export_task_.file; current_export_task_.file = nullptr;
        emit exportFinished(false, "起始日期不能晚于结束日期！"); return;
    }

    current_export_task_.current_start = current_export_task_.final_start;
    current_export_task_.circuit_id = circuit_id;
    current_export_task_.active = true;

    int num_channels = (current_temp_monitor_type_ == 1) ? 32 : 40;
    int num_cards = (current_temp_monitor_type_ == 1) ? 4 : 5;
    int start_card = (circuit_id == 1) ? 1 : ((current_temp_monitor_type_ == 1) ? 5 : 6);

    QStringList headers;
    headers << "时间" << "回路编号" << "自动/手动" << "主调压器开关" << "主调压器电压" << "主调压器电流"
            << "试验回路加热状态" << "试验回路开关状态" << "试验回路电流"
            << "辅调压器开关" << "辅调压器电压" << "辅调压器电流"
            << "模拟回路加热状态" << "模拟回路开关状态" << "模拟回路电流";

    // 【修改点】映射 CSV 表头为 采集卡x-y
    for (int r = 0; r < num_cards; ++r) {
        for (int c = 1; c <= 8; ++c) {
            headers << QString("采集卡%1-通%2").arg(start_card + r).arg(c);
        }
    }

    QString header_str = headers.join(",") + "\n";
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    QStringEncoder encoder("GBK"); current_export_task_.file->write(encoder(header_str));
#else
    QTextCodec *codec = QTextCodec::codecForName("GBK"); current_export_task_.file->write(codec->fromUnicode(header_str));
#endif

    current_export_task_.db_columns = {
        "record_time", "circuit_id", "auto_on",
        "regulator_1_breaker_closed", "regulator_1_voltage", "regulator_1_current",
        "test_loop_is_heat", "test_loop_breaker_closed", "test_loop_current",
        "regulator_2_breaker_closed", "regulator_2_voltage", "regulator_2_current",
        "ref_loop_is_heat", "ref_loop_breaker_closed", "ref_loop_current"
    };

    // 【修改点】生成 32/40 路的 DB 查询字段
    for(int i = 1; i <= num_channels; ++i) {
        current_export_task_.db_columns.push_back(QString("circuit_temp%1").arg(i, 2, 10, QChar('0')).toStdString());
    }

    executeNextExportChunk();
}

void QtROSNode::executeNextExportChunk() {
    if (!current_export_task_.active) return;
    QDateTime chunk_end = current_export_task_.current_start.addDays(10);
    if (chunk_end > current_export_task_.final_end) chunk_end = current_export_task_.final_end;

    auto request = std::make_shared<ros2_interfaces::srv::QueryDataRecords::Request>();
    request->start_time = current_export_task_.current_start.toString("yyyy-MM-dd HH:mm:ss").toStdString();
    request->end_time = chunk_end.toString("yyyy-MM-dd HH:mm:ss").toStdString();
    request->column_names = current_export_task_.db_columns;
    request->circuit_id_filter = current_export_task_.circuit_id;

    auto callback = [this, chunk_end](rclcpp::Client<ros2_interfaces::srv::QueryDataRecords>::SharedFuture future) {
        auto response = future.get();
        if (!response->success) {
            current_export_task_.active = false; current_export_task_.file->close(); delete current_export_task_.file; current_export_task_.file = nullptr;
            emit exportFinished(false, QString::fromStdString(response->message)); return;
        }
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        QStringEncoder encoder("GBK");
#else
        QTextCodec *codec = QTextCodec::codecForName("GBK");
#endif
        for (const auto& row : response->data_rows) {
            QString row_str = QString::fromStdString(row) + "\n";
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
            current_export_task_.file->write(encoder(row_str));
#else
            current_export_task_.file->write(codec->fromUnicode(row_str));
#endif
        }
        qint64 total = current_export_task_.final_start.secsTo(current_export_task_.final_end);
        qint64 current = current_export_task_.final_start.secsTo(chunk_end);
        int percent = (total > 0) ? (int)((current * 100) / total) : 100;
        if (percent > 100) percent = 100;
        emit exportProgress(percent);

        if (chunk_end >= current_export_task_.final_end) {
            current_export_task_.active = false; current_export_task_.file->flush(); current_export_task_.file->close();
            delete current_export_task_.file; current_export_task_.file = nullptr;
            emit exportFinished(true, "数据导出成功！");
        } else {
            current_export_task_.current_start = chunk_end.addSecs(1); executeNextExportChunk();
        }
    };
    query_data_records_client_->async_send_request(request, callback);
}

// 试验管理
void QtROSNode::onListTestRecordsRequested(const QString& keyword, int page, int page_size, int circuit_id) {
    if (!list_test_records_client_->service_is_ready()) {
        QVariantMap errMap; errMap["success"] = false; emit testRecordsListed(circuit_id, errMap); return;
    }
    auto request = std::make_shared<ros2_interfaces::srv::ListTestRecords::Request>();
    request->search_keyword = keyword.toStdString();
    request->page = page;
    request->page_size = page_size;
    request->circuit_id = circuit_id; // 设置请求条件

    auto callback = [this, circuit_id](rclcpp::Client<ros2_interfaces::srv::ListTestRecords>::SharedFuture future) {
        auto response = future.get();
        QVariantMap map; map["success"] = response->success; map["total_pages"] = response->total_pages; map["current_page"] = response->current_page;
        QVariantList list;
        for (const auto& r : response->records) {
            QVariantMap rmap;
            rmap["id"] = r.id;
            rmap["circuit_id"] = r.circuit_id;
            rmap["start_date"] = QString::fromStdString(r.start_date);
            rmap["end_date"] = QString::fromStdString(r.end_date);
            rmap["cable_id"] = r.cable_id;
            rmap["cable_name"] = QString::fromStdString(r.cable_name);
            rmap["remarks"] = QString::fromStdString(r.remarks);
            QVariantList tps;
            for(int i=0; i<40; ++i) tps.append(QString::fromStdString(r.temp_points[i]));
            rmap["temp_points"] = tps;
            list.append(rmap);
        }
        map["records"] = list;
        emit testRecordsListed(circuit_id, map);
    };
    list_test_records_client_->async_send_request(request, callback);
}

void QtROSNode::onSaveTestRecordRequested(const QVariantMap& map) {
    if (!save_test_record_client_->service_is_ready()) { emit testRecordSaveResult(false, "Service not available."); return; }
    auto request = std::make_shared<ros2_interfaces::srv::SaveTestRecord::Request>();
    request->record.id = map["id"].toInt();
    request->record.circuit_id = map["circuit_id"].toInt();
    request->record.start_date = map["start_date"].toString().toStdString();
    request->record.end_date = map["end_date"].toString().toStdString();
    request->record.cable_id = map["cable_id"].toInt();
    request->record.cable_name = map["cable_name"].toString().toStdString();
    request->record.remarks = map["remarks"].toString().toStdString();
    QVariantList tps = map["temp_points"].toList();
    for(int i=0; i<40 && i<tps.size(); ++i) request->record.temp_points[i] = tps[i].toString().toStdString();

    auto callback = [this](rclcpp::Client<ros2_interfaces::srv::SaveTestRecord>::SharedFuture future) { emit testRecordSaveResult(future.get()->success, QString::fromStdString(future.get()->message)); };
    save_test_record_client_->async_send_request(request, callback);
}

void QtROSNode::onDeleteTestRecordRequested(int id) {
    if (!delete_test_record_client_->service_is_ready()) { emit testRecordDeleteResult(false, "Service not available."); return; }
    auto request = std::make_shared<ros2_interfaces::srv::DeleteTestRecord::Request>(); request->id = id;
    auto callback = [this](rclcpp::Client<ros2_interfaces::srv::DeleteTestRecord>::SharedFuture future) { emit testRecordDeleteResult(future.get()->success, QString::fromStdString(future.get()->message)); };
    delete_test_record_client_->async_send_request(request, callback);
}

// 导出 CSV (由C++处理，保证全量导出和GBK编码)
void QtROSNode::onExportTestRecordsRequested(const QString& file_path) {
    if (!list_test_records_client_->service_is_ready()) { emit exportTestRecordResult(false, "服务未就绪"); return; }

    auto request = std::make_shared<ros2_interfaces::srv::ListTestRecords::Request>();
    request->page = 1; request->page_size = 999999; // 一次性拉取全部

    auto callback = [this, file_path](rclcpp::Client<ros2_interfaces::srv::ListTestRecords>::SharedFuture future) {
        auto response = future.get();
        if(!response->success) { emit exportTestRecordResult(false, "拉取数据失败"); return; }

        QUrl url(file_path);
        QFile file(url.isLocalFile() ? url.toLocalFile() : file_path);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) { emit exportTestRecordResult(false, "无法写入文件"); return; }

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        QStringEncoder encoder("GBK");
#else
        QTextCodec *codec = QTextCodec::codecForName("GBK");
#endif
        // 表头
        QString header = "回路ID,起始日期,结束日期,电缆ID,电缆名称,备注";
        for(int i=1; i<=40; ++i) header += QString(",测温点%1").arg(i);
        header += "\n";
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        file.write(encoder(header));
#else
        file.write(codec->fromUnicode(header));
#endif
        // 数据行
        for(const auto& r : response->records) {
            QString row = QString("%1,%2,%3,%4,%5,%6")
            .arg(r.circuit_id).arg(QString::fromStdString(r.start_date)).arg(QString::fromStdString(r.end_date))
                .arg(r.cable_id).arg(QString::fromStdString(r.cable_name)).arg(QString::fromStdString(r.remarks));
            for(int i=0; i<40; ++i) row += "," + QString::fromStdString(r.temp_points[i]);
            row += "\n";
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
            file.write(encoder(row));
#else
            file.write(codec->fromUnicode(row));
#endif
        }
        file.close();
        emit exportTestRecordResult(true, "导出成功");
    };
    list_test_records_client_->async_send_request(request, callback);
}

// 电缆管理
void QtROSNode::onListCablesRequested(const QString& keyword, int page, int page_size, int sort_column, bool is_ascending) {
    if (!list_cables_client_->service_is_ready()) { QVariantMap errMap; errMap["success"] = false; errMap["message"] = "ListCables service not available."; emit cablesListed(errMap); return; }
    auto request = std::make_shared<ros2_interfaces::srv::ListCables::Request>();
    request->search_keyword = keyword.toStdString(); request->page = page; request->page_size = page_size; request->sort_column = sort_column; request->is_ascending = is_ascending;
    auto callback = [this](rclcpp::Client<ros2_interfaces::srv::ListCables>::SharedFuture future) {
        auto response = future.get(); QVariantMap map; map["success"] = response->success; map["message"] = QString::fromStdString(response->message); map["total_pages"] = response->total_pages; map["current_page"] = response->current_page;
        QVariantList list;
        for (const auto& c : response->cables) {
            QVariantMap cmap; cmap["id"] = c.id; cmap["name"] = QString::fromStdString(c.name); cmap["core_diameter"] = c.core_diameter; cmap["core_material"] = QString::fromStdString(c.core_material); cmap["insulation_thickness"] = c.insulation_thickness; cmap["insulation_material"] = QString::fromStdString(c.insulation_material); cmap["voltage_grade"] = c.voltage_grade; cmap["system_format"] = c.system_format; cmap["notes"] = QString::fromStdString(c.notes); cmap["last_modified"] = QString::fromStdString(c.last_modified);
            list.append(cmap);
        }
        map["cables"] = list; emit cablesListed(map);
    };
    list_cables_client_->async_send_request(request, callback);
}

void QtROSNode::onSaveCableRequested(const QVariantMap& cableMap) {
    if (!save_cable_client_->service_is_ready()) { emit cableSaveResult(false, "SaveCable service not available."); return; }
    auto request = std::make_shared<ros2_interfaces::srv::SaveCable::Request>();
    request->cable_data.id = cableMap["id"].toInt(); request->cable_data.name = cableMap["name"].toString().toStdString(); request->cable_data.core_diameter = cableMap["core_diameter"].toDouble(); request->cable_data.core_material = cableMap["core_material"].toString().toStdString(); request->cable_data.insulation_thickness = cableMap["insulation_thickness"].toDouble(); request->cable_data.insulation_material = cableMap["insulation_material"].toString().toStdString(); request->cable_data.voltage_grade = cableMap["voltage_grade"].toInt(); request->cable_data.system_format = cableMap["system_format"].toInt(); request->cable_data.notes = cableMap["notes"].toString().toStdString();
    auto callback = [this](rclcpp::Client<ros2_interfaces::srv::SaveCable>::SharedFuture future) { auto response = future.get(); emit cableSaveResult(response->success, QString::fromStdString(response->message)); };
    save_cable_client_->async_send_request(request, callback);
}

void QtROSNode::onDeleteCableRequested(int id) {
    if (!delete_cable_client_->service_is_ready()) { emit cableDeleteResult(false, "DeleteCable service not available."); return; }
    auto request = std::make_shared<ros2_interfaces::srv::DeleteCable::Request>(); request->id = id;
    auto callback = [this](rclcpp::Client<ros2_interfaces::srv::DeleteCable>::SharedFuture future) { auto response = future.get(); emit cableDeleteResult(response->success, QString::fromStdString(response->message)); };
    delete_cable_client_->async_send_request(request, callback);
}
