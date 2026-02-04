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

    // 1. 初始化 DatabaseManager
    auto db_path = this->declare_parameter<std::string>(
        record_node_constants::DB_PATH_PARAM,
        record_node_constants::DEFAULT_DB_PATH);
    db_manager_ = std::make_unique<DatabaseManager>(db_path, this->get_logger());

    // 2. 初始化记录参数
    record_interval_min_ = this->declare_parameter<int64_t>(
        record_node_constants::RECORD_INTERVAL_MIN_PARAM,
        record_node_constants::DEFAULT_RECORD_INTERVAL_MIN);
    if (record_interval_min_ < 1) record_interval_min_ = 1;

    // 3. 创建订阅者
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

    // 4. 创建设置类服务 (Save/Set)
    auto save_sys_name = this->declare_parameter<std::string>(
        record_node_constants::SAVE_SYSTEM_SETTINGS_SERVICE_PARAM,
        record_node_constants::DEFAULT_SAVE_SYSTEM_SETTINGS_SERVICE);
    save_system_settings_service_ = this->create_service<ros2_interfaces::srv::SetSystemSettings>(
        save_sys_name, std::bind(&RecordNode::save_system_settings_callback, this, _1, _2));

    auto save_reg_name = this->declare_parameter<std::string>(
        record_node_constants::SAVE_REGULATOR_SETTINGS_SERVICE_PARAM,
        record_node_constants::DEFAULT_SAVE_REGULATOR_SETTINGS_SERVICE);
    save_regulator_settings_service_ = this->create_service<ros2_interfaces::srv::SetRegulatorSettings>(
        save_reg_name, std::bind(&RecordNode::save_regulator_settings_callback, this, _1, _2));

    auto save_cir_name = this->declare_parameter<std::string>(
        record_node_constants::SAVE_CIRCUIT_SETTINGS_SERVICE_PARAM,
        record_node_constants::DEFAULT_SAVE_CIRCUIT_SETTINGS_SERVICE);
    save_circuit_settings_service_ = this->create_service<ros2_interfaces::srv::SetCircuitSettings>(
        save_cir_name, std::bind(&RecordNode::save_circuit_settings_callback, this, _1, _2));

    // 5. 创建查询类服务 (Get - 新增)
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

    // 6. 启动时间对齐逻辑
    start_alignment_timer();

    RCLCPP_INFO(this->get_logger(), "RecordNode initialization complete.");
}

// --- Topic Callbacks ---
void RecordNode::circuit_status_callback(const ros2_interfaces::msg::CircuitStatus::SharedPtr msg)
{
    latest_circuit_status_[msg->circuit_id] = *msg;
}

void RecordNode::regulator_status_callback(const ros2_interfaces::msg::RegulatorStatus::SharedPtr msg)
{
    latest_regulator_status_[msg->regulator_id] = *msg;
}

// --- Set/Save Service Callbacks ---
void RecordNode::save_system_settings_callback(
    const std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Request> request,
    std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Response> response)
{
    this->keep_record_on_shutdown_ = request->settings.keep_record_on_shutdown;
    bool success = db_manager_->save_system_settings(request->settings);
    response->success = success;
    response->message = success ? "Success" : "Database error";
}

void RecordNode::save_regulator_settings_callback(
    const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request> request,
    std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Response> response)
{
    bool success = db_manager_->save_regulator_settings(request->settings.regulator_id, request->settings);
    response->success = success;
    response->message = success ? "Success" : "Database error";
}

void RecordNode::save_circuit_settings_callback(
    const std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Request> request,
    std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Response> response)
{
    bool success = db_manager_->save_circuit_settings(request->settings.circuit_id, request->settings);
    response->success = success;
    response->message = success ? "Success" : "Database error";
}

// --- Get/Query Service Callbacks (新增) ---
void RecordNode::get_system_settings_callback(
    const std::shared_ptr<ros2_interfaces::srv::GetSystemSettings::Request> /*request*/,
    std::shared_ptr<ros2_interfaces::srv::GetSystemSettings::Response> response)
{
    response->success = db_manager_->get_system_settings(response->settings);
}

void RecordNode::get_regulator_settings_callback(
    const std::shared_ptr<ros2_interfaces::srv::GetRegulatorSettings::Request> request,
    std::shared_ptr<ros2_interfaces::srv::GetRegulatorSettings::Response> response)
{
    response->success = db_manager_->get_regulator_settings(request->regulator_id, response->settings);
}

void RecordNode::get_circuit_settings_callback(
    const std::shared_ptr<ros2_interfaces::srv::GetCircuitSettings::Request> request,
    std::shared_ptr<ros2_interfaces::srv::GetCircuitSettings::Response> response)
{
    response->success = db_manager_->get_circuit_settings(request->circuit_id, response->settings);
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

// --- Recording Logic ---
void RecordNode::start_alignment_timer()
{
    auto now = std::chrono::system_clock::now();
    time_t t = std::chrono::system_clock::to_time_t(now);
    struct tm tm_struct;
#ifdef _MSC_VER
    gmtime_s(&tm_struct, &t);
#else
    gmtime_r(&t, &tm_struct);
#endif

    int seconds_to_next_minute = 60 - tm_struct.tm_sec;
    auto delay = std::chrono::seconds(seconds_to_next_minute) + std::chrono::milliseconds(100);

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
    gmtime_s(&tm_struct, &t);
#else
    gmtime_r(&t, &tm_struct);
#endif
    tm_struct.tm_sec = 0;

    std::stringstream ss;
    ss << std::put_time(&tm_struct, "%Y-%m-%d %H:%M:%S");
    std::string time_str = ss.str();

    for (uint8_t id = 1; id <= 2; ++id) {
        if (latest_circuit_status_.count(id) && latest_regulator_status_.count(id)) {
            const auto& circuit = latest_circuit_status_[id];
            const auto& regulator = latest_regulator_status_[id];

            bool should_record = keep_record_on_shutdown_ || circuit.test_loop.enabled || circuit.ref_loop.enabled;

            if (should_record) {
                db_manager_->insert_data_record(time_str, circuit, regulator);
            }
        }
    }
}
