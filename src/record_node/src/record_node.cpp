#include "record_node/record_node.hpp"
#include "record_node/record_node_constants.hpp"
#include <chrono>
#include <ctime>
#include <iomanip>
#include <vector>

using std::placeholders::_1;
using std::placeholders::_2;

RecordNode::RecordNode() : Node("record_node")
{
    RCLCPP_INFO(this->get_logger(), "Initializing RecordNode...");

    auto db_path = this->declare_parameter<std::string>(
        record_node_constants::DB_PATH_PARAM,
        record_node_constants::DEFAULT_DB_PATH);
    db_manager_ = std::make_unique<DatabaseManager>(db_path, this->get_logger());

    record_interval_min_ = this->declare_parameter<int64_t>(
        record_node_constants::RECORD_INTERVAL_MIN_PARAM,
        record_node_constants::DEFAULT_RECORD_INTERVAL_MIN);
    if (record_interval_min_ < 1) record_interval_min_ = 1;

    load_initial_settings();

    auto sys_set_topic = this->declare_parameter<std::string>(
        record_node_constants::SYSTEM_SETTINGS_TOPIC_PARAM,
        record_node_constants::DEFAULT_SYSTEM_SETTINGS_TOPIC);
    system_settings_sub_ = this->create_subscription<ros2_interfaces::msg::SystemSettings>(
        sys_set_topic, 10, std::bind(&RecordNode::system_settings_topic_callback, this, _1));

    auto reg_set_topic = this->declare_parameter<std::string>(
        record_node_constants::REGULATOR_SETTINGS_TOPIC_PARAM,
        record_node_constants::DEFAULT_REGULATOR_SETTINGS_TOPIC);
    regulator_settings_sub_ = this->create_subscription<ros2_interfaces::msg::RegulatorSettings>(
        reg_set_topic, 10, std::bind(&RecordNode::regulator_settings_topic_callback, this, _1));

    auto cir_set_topic = this->declare_parameter<std::string>(
        record_node_constants::CIRCUIT_SETTINGS_TOPIC_PARAM,
        record_node_constants::DEFAULT_CIRCUIT_SETTINGS_TOPIC);
    circuit_settings_sub_ = this->create_subscription<ros2_interfaces::msg::CircuitSettings>(
        cir_set_topic, 10, std::bind(&RecordNode::circuit_settings_topic_callback, this, _1));

    auto circuit_topic = this->declare_parameter<std::string>(
        record_node_constants::CIRCUIT_STATUS_TOPIC_PARAM,
        record_node_constants::DEFAULT_CIRCUIT_STATUS_TOPIC);
    circuit_status_sub_ = this->create_subscription<ros2_interfaces::msg::CircuitStatus>(
        circuit_topic, 10, std::bind(&RecordNode::circuit_status_callback, this, _1));

    auto regulator_topic = this->declare_parameter<std::string>(
        record_node_constants::REGULATOR_STATUS_TOPIC_PARAM,
        record_node_constants::DEFAULT_REGULATOR_STATUS_TOPIC);
    regulator_status_sub_ = this->create_subscription<ros2_interfaces::msg::RegulatorStatus>(
        regulator_topic, 10, std::bind(&RecordNode::regulator_status_callback, this, _1));

    auto get_sys_name = this->declare_parameter<std::string>(
        record_node_constants::GET_SYSTEM_SETTINGS_SERVICE_PARAM,
        record_node_constants::DEFAULT_GET_SYSTEM_SETTINGS_SERVICE);
    get_system_settings_service_ = this->create_service<ros2_interfaces::srv::GetSystemSettings>(
        get_sys_name, std::bind(&RecordNode::get_system_settings_callback, this, _1, _2));

    auto get_reg_name = this->declare_parameter<std::string>(
        record_node_constants::GET_REGULATOR_SETTINGS_SERVICE_PARAM,
        record_node_constants::DEFAULT_GET_REGULATOR_SETTINGS_SERVICE);
    get_regulator_settings_service_ = this->create_service<ros2_interfaces::srv::GetRegulatorSettings>(
        get_reg_name, std::bind(&RecordNode::get_regulator_settings_callback, this, _1, _2));

    auto get_cir_name = this->declare_parameter<std::string>(
        record_node_constants::GET_CIRCUIT_SETTINGS_SERVICE_PARAM,
        record_node_constants::DEFAULT_GET_CIRCUIT_SETTINGS_SERVICE);
    get_circuit_settings_service_ = this->create_service<ros2_interfaces::srv::GetCircuitSettings>(
        get_cir_name, std::bind(&RecordNode::get_circuit_settings_callback, this, _1, _2));

    auto get_data_name = this->declare_parameter<std::string>(
        record_node_constants::GET_DATA_RECORDS_SERVICE_PARAM,
        record_node_constants::DEFAULT_GET_DATA_RECORDS_SERVICE);
    get_data_records_service_ = this->create_service<ros2_interfaces::srv::GetDataRecords>(
        get_data_name, std::bind(&RecordNode::get_data_records_callback, this, _1, _2));

    auto query_service_name = this->declare_parameter<std::string>(
        record_node_constants::QUERY_DATA_RECORDS_SERVICE_PARAM,
        record_node_constants::DEFAULT_QUERY_DATA_RECORDS_SERVICE);
    query_data_records_service_ = this->create_service<ros2_interfaces::srv::QueryDataRecords>(
        query_service_name, std::bind(&RecordNode::query_data_records_callback, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Service %s created.", query_service_name.c_str());

    auto list_test_service_name = this->declare_parameter<std::string>(
        record_node_constants::LIST_TEST_RECORDS_SERVICE_PARAM,
        record_node_constants::DEFAULT_LIST_TEST_RECORDS_SERVICE);
    list_test_records_service_ = this->create_service<ros2_interfaces::srv::ListTestRecords>(
        list_test_service_name, std::bind(&RecordNode::list_test_records_callback, this, _1, _2));

    auto save_test_service_name = this->declare_parameter<std::string>(
        record_node_constants::SAVE_TEST_RECORD_SERVICE_PARAM,
        record_node_constants::DEFAULT_SAVE_TEST_RECORD_SERVICE);
    save_test_record_service_ = this->create_service<ros2_interfaces::srv::SaveTestRecord>(
        save_test_service_name, std::bind(&RecordNode::save_test_record_callback, this, _1, _2));

    auto delete_test_service_name = this->declare_parameter<std::string>(
        record_node_constants::DELETE_TEST_RECORD_SERVICE_PARAM,
        record_node_constants::DEFAULT_DELETE_TEST_RECORD_SERVICE);
    delete_test_record_service_ = this->create_service<ros2_interfaces::srv::DeleteTestRecord>(
        delete_test_service_name, std::bind(&RecordNode::delete_test_record_callback, this, _1, _2));

    auto db_status_topic = this->declare_parameter<std::string>(
        record_node_constants::DATABASE_STATUS_TOPIC_PARAM,
        record_node_constants::DEFAULT_DATABASE_STATUS_TOPIC);
    database_status_pub_ = this->create_publisher<ros2_interfaces::msg::DatabaseStatus>(
        db_status_topic, 10);
    database_status_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&RecordNode::database_status_timer_callback, this));

    reschedule_timers();
    RCLCPP_INFO(this->get_logger(), "RecordNode initialization complete.");
}

void RecordNode::load_initial_settings()
{
    if (db_manager_->get_system_settings(current_system_settings_)) {
        keep_record_on_shutdown_ = current_system_settings_.keep_record_on_shutdown;
        RCLCPP_INFO(this->get_logger(), "Loaded initial system settings. Interval: %d min", current_system_settings_.record_interval_min);
    } else {
        current_system_settings_.record_interval_min = 1;
        current_system_settings_.keep_record_on_shutdown = true;
        RCLCPP_WARN(this->get_logger(), "Failed to load initial system settings (using defaults).");
    }

    for (uint8_t id = 1; id <= 2; ++id) {
        ros2_interfaces::msg::RegulatorSettings settings;
        if (db_manager_->get_regulator_settings(id, settings)) {
            current_regulator_settings_[id] = settings;
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to load regulator settings for ID %d", id);
        }
    }

    for (uint8_t id = 1; id <= 2; ++id) {
        ros2_interfaces::msg::CircuitSettings settings;
        if (db_manager_->get_circuit_settings(id, settings)) {
            current_circuit_settings_[id] = settings;
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to load circuit settings for ID %d", id);
        }
    }
}

void RecordNode::system_settings_topic_callback(const ros2_interfaces::msg::SystemSettings::SharedPtr msg)
{
    if (*msg != current_system_settings_) {
        RCLCPP_INFO(this->get_logger(), "Detected System Settings change. Updating DB.");

        bool interval_changed = (msg->record_interval_min != current_system_settings_.record_interval_min);

        bool success = db_manager_->save_system_settings(*msg);
        if (success) {
            current_system_settings_ = *msg;
            keep_record_on_shutdown_ = msg->keep_record_on_shutdown;

            if (interval_changed) {
                RCLCPP_INFO(this->get_logger(),
                            "Record interval changed to %d min. Rescheduling timers...",
                            current_system_settings_.record_interval_min);
                reschedule_timers();
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to save updated system settings to DB!");
        }
    }
}

void RecordNode::regulator_settings_topic_callback(const ros2_interfaces::msg::RegulatorSettings::SharedPtr msg)
{
    uint8_t id = msg->regulator_id;
    if (current_regulator_settings_.find(id) == current_regulator_settings_.end() ||
        current_regulator_settings_[id] != *msg)
    {
        RCLCPP_INFO(this->get_logger(), "Detected Regulator Settings change for ID %d. Updating DB.", id);

        bool success = db_manager_->save_regulator_settings(id, *msg);
        if (success) {
            current_regulator_settings_[id] = *msg;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to save regulator settings ID %d to DB!", id);
        }
    }
}

void RecordNode::circuit_settings_topic_callback(const ros2_interfaces::msg::CircuitSettings::SharedPtr msg)
{
    uint8_t id = msg->circuit_id;
    if (current_circuit_settings_.find(id) == current_circuit_settings_.end() ||
        current_circuit_settings_[id] != *msg)
    {
        RCLCPP_INFO(this->get_logger(), "Detected Circuit Settings change for ID %d. Updating DB.", id);

        bool success = db_manager_->save_circuit_settings(id, *msg);
        if (success) {
            current_circuit_settings_[id] = *msg;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to save circuit settings ID %d to DB!", id);
        }
    }
}

void RecordNode::circuit_status_callback(const ros2_interfaces::msg::CircuitStatus::SharedPtr msg)
{
    latest_circuit_status_[msg->circuit_id] = *msg;
}

void RecordNode::regulator_status_callback(const ros2_interfaces::msg::RegulatorStatus::SharedPtr msg)
{
    latest_regulator_status_[msg->regulator_id] = *msg;
}

void RecordNode::get_system_settings_callback(
    const std::shared_ptr<ros2_interfaces::srv::GetSystemSettings::Request> /*request*/,
    std::shared_ptr<ros2_interfaces::srv::GetSystemSettings::Response> response)
{
    response->settings = current_system_settings_;
    response->success = true;
}

void RecordNode::get_regulator_settings_callback(
    const std::shared_ptr<ros2_interfaces::srv::GetRegulatorSettings::Request> request,
    std::shared_ptr<ros2_interfaces::srv::GetRegulatorSettings::Response> response)
{
    uint8_t id = request->regulator_id;
    if (current_regulator_settings_.count(id)) {
        response->settings = current_regulator_settings_[id];
        response->success = true;
    } else {
        response->success = db_manager_->get_regulator_settings(id, response->settings);
    }
}

void RecordNode::get_circuit_settings_callback(
    const std::shared_ptr<ros2_interfaces::srv::GetCircuitSettings::Request> request,
    std::shared_ptr<ros2_interfaces::srv::GetCircuitSettings::Response> response)
{
    uint8_t id = request->circuit_id;
    if (current_circuit_settings_.count(id)) {
        response->settings = current_circuit_settings_[id];
        response->success = true;
    } else {
        response->success = db_manager_->get_circuit_settings(id, response->settings);
    }
}

void RecordNode::get_data_records_callback(
    const std::shared_ptr<ros2_interfaces::srv::GetDataRecords::Request> request,
    std::shared_ptr<ros2_interfaces::srv::GetDataRecords::Response> response)
{
    auto results = db_manager_->get_data_records(request->start_time, request->end_time);
    response->records = results;
    response->success = true;
    response->message = "Retrieved " + std::to_string(results.size()) + " records.";
}

void RecordNode::query_data_records_callback(
    const std::shared_ptr<ros2_interfaces::srv::QueryDataRecords::Request> request,
    std::shared_ptr<ros2_interfaces::srv::QueryDataRecords::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received query request: %lu columns, Filter ID: %d, Time: %s to %s",
                request->column_names.size(), request->circuit_id_filter,
                request->start_time.c_str(), request->end_time.c_str());

    std::vector<std::string> headers;
    std::vector<std::string> rows;

    bool result = db_manager_->query_data_records(
        request->column_names,
        request->start_time,
        request->end_time,
        request->circuit_id_filter,
        headers,
        rows
        );

    if (result) {
        response->success = true;
        response->header = headers;
        response->data_rows = rows;
        response->message = "Query successful. Retrieved " + std::to_string(rows.size()) + " rows.";

        std::string header_str;
        for (const auto& h : headers) { header_str += h + ", "; }

        RCLCPP_INFO(this->get_logger(), "[Service Response] Success: True. Returned Rows: %zu", rows.size());
        RCLCPP_INFO(this->get_logger(), "[Service Response] Headers parsed: [%s]", header_str.c_str());

        if (!rows.empty()) {
            RCLCPP_INFO(this->get_logger(), "[Service Response] Sample First Row: %s", rows.front().c_str());
            RCLCPP_INFO(this->get_logger(), "[Service Response] Sample Last Row: %s", rows.back().c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "[Service Response] 0 rows returned! Please check if the 'record_time' in the database exactly matches the format and falls between the requested time range.");
        }

    } else {
        response->success = false;
        response->message = "Database query failed or no valid columns provided.";
        RCLCPP_ERROR(this->get_logger(), "[Service Response] Query Failed!");
    }
}

// [修改] 传递 circuit_id 到数据层
void RecordNode::list_test_records_callback(
    const std::shared_ptr<ros2_interfaces::srv::ListTestRecords::Request> request,
    std::shared_ptr<ros2_interfaces::srv::ListTestRecords::Response> response)
{
    std::vector<ros2_interfaces::msg::TestRecord> records;
    int total_pages = 0;

    bool result = db_manager_->list_test_records(
        request->circuit_id,    // 新增传递
        request->search_keyword,
        request->page,
        request->page_size,
        records,
        total_pages);

    if (result) {
        response->success = true;
        response->records = records;
        response->current_page = request->page;
        response->total_pages = total_pages;
        response->message = "Query test records successful.";
        RCLCPP_INFO(this->get_logger(), "Listed %zu test records (filter circuit: %d).", records.size(), request->circuit_id);
    } else {
        response->success = false;
        response->message = "Failed to query test records from database.";
        RCLCPP_ERROR(this->get_logger(), "Failed to query test records.");
    }
}

void RecordNode::save_test_record_callback(
    const std::shared_ptr<ros2_interfaces::srv::SaveTestRecord::Request> request,
    std::shared_ptr<ros2_interfaces::srv::SaveTestRecord::Response> response)
{
    bool result = db_manager_->save_test_record(request->record);
    if (result) {
        response->success = true;
        response->message = "Test record saved successfully.";
        RCLCPP_INFO(this->get_logger(), "Test record saved successfully (Circuit ID: %d, Cable: %s).",
                    request->record.circuit_id, request->record.cable_name.c_str());
    } else {
        response->success = false;
        response->message = "Failed to save test record to database.";
        RCLCPP_ERROR(this->get_logger(), "Failed to save test record.");
    }
}

void RecordNode::delete_test_record_callback(
    const std::shared_ptr<ros2_interfaces::srv::DeleteTestRecord::Request> request,
    std::shared_ptr<ros2_interfaces::srv::DeleteTestRecord::Response> response)
{
    bool result = db_manager_->delete_test_record(request->id);
    if (result) {
        response->success = true;
        response->message = "Test record deleted successfully.";
        RCLCPP_INFO(this->get_logger(), "Test record (ID: %d) deleted successfully.", request->id);
    } else {
        response->success = false;
        response->message = "Failed to delete test record from database.";
        RCLCPP_ERROR(this->get_logger(), "Failed to delete test record (ID: %d).", request->id);
    }
}

void RecordNode::reschedule_timers()
{
    if (alignment_timer_ && !alignment_timer_->is_canceled()) {
        alignment_timer_->cancel();
    }
    if (record_timer_ && !record_timer_->is_canceled()) {
        record_timer_->cancel();
    }

    int interval_min = current_system_settings_.record_interval_min;
    if (interval_min < 1) interval_min = 1;

    this->record_interval_min_ = interval_min;

    auto now = std::chrono::system_clock::now();
    time_t t = std::chrono::system_clock::to_time_t(now);
    struct tm tm_struct;
#ifdef _MSC_VER
    localtime_s(&tm_struct, &t);
#else
    localtime_r(&t, &tm_struct);
#endif

    long current_seconds_in_hour = tm_struct.tm_min * 60 + tm_struct.tm_sec;
    long interval_seconds = interval_min * 60;
    long seconds_to_wait = interval_seconds - (current_seconds_in_hour % interval_seconds);
    auto delay = std::chrono::seconds(seconds_to_wait) + std::chrono::milliseconds(100);

    RCLCPP_INFO(this->get_logger(),
                "Scheduling next record in %ld seconds (Aligning to %d min interval)",
                seconds_to_wait, interval_min);

    alignment_timer_ = this->create_wall_timer(
        delay,
        [this]() {
            this->alignment_timer_->cancel();
            this->record_timer_callback();
            this->record_timer_ = this->create_wall_timer(
                std::chrono::minutes(this->record_interval_min_),
                std::bind(&RecordNode::record_timer_callback, this));
        });
}

void RecordNode::record_timer_callback()
{
    auto now_sys = std::chrono::system_clock::now();
    time_t t = std::chrono::system_clock::to_time_t(now_sys);
    struct tm tm_struct;
#ifdef _MSC_VER
    localtime_s(&tm_struct, &t);
#else
    localtime_r(&t, &tm_struct);
#endif

    tm_struct.tm_sec = 0;

    std::stringstream ss;
    ss << std::put_time(&tm_struct, "%Y-%m-%d %H:%M:%S");
    std::string time_str = ss.str();

    bool auto_on = current_system_settings_.auto_on;

    ros2_interfaces::msg::RegulatorStatus reg1_status;
    ros2_interfaces::msg::RegulatorStatus reg2_status;
    if (latest_regulator_status_.count(1)) {
        reg1_status = latest_regulator_status_[1];
    }
    if (latest_regulator_status_.count(2)) {
        reg2_status = latest_regulator_status_[2];
    }

    for (uint8_t id = 1; id <= 2; ++id) {
        if (latest_circuit_status_.count(id) && current_circuit_settings_.count(id)) {
            const auto& circuit_status = latest_circuit_status_[id];
            const auto& circuit_settings = current_circuit_settings_[id];

            bool should_record = keep_record_on_shutdown_
                                 || circuit_settings.test_loop.enabled
                                 || circuit_settings.ref_loop.enabled;

            if (should_record) {
                db_manager_->insert_data_record(
                    time_str,
                    id,
                    auto_on,
                    circuit_status,
                    circuit_settings,
                    reg1_status,
                    reg2_status
                    );
            }
        }
    }
}

void RecordNode::database_status_timer_callback()
{
    ros2_interfaces::msg::DatabaseStatus status_msg;
    status_msg.database_connected = db_manager_->is_connected();
    database_status_pub_->publish(status_msg);
}
