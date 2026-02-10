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

    // 3. 从数据库加载初始设置到内存 (必须在创建订阅前完成)
    load_initial_settings();

    // 4. 创建设置类订阅者 (监听变更并保存)
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

    // 5. 创建状态类订阅者
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

    // 6. 创建查询类服务 (Get)
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

    // 7. 启动时间对齐逻辑
    reschedule_timers();

    RCLCPP_INFO(this->get_logger(), "RecordNode initialization complete.");
}

void RecordNode::load_initial_settings()
{
    // 加载系统设置 (ID=1)
    if (db_manager_->get_system_settings(current_system_settings_)) {
        // 同步内存中的控制变量
        keep_record_on_shutdown_ = current_system_settings_.keep_record_on_shutdown;
        // 注意：这里不需要手动设置 record_interval_min_，因为 reschedule_timers 会从 current_system_settings_ 读取
        RCLCPP_INFO(this->get_logger(), "Loaded initial system settings. Interval: %d min", current_system_settings_.record_interval_min);
    } else {
        // 如果没读到，确保 current_system_settings_ 有默认值
        current_system_settings_.record_interval_min = 1;
        current_system_settings_.keep_record_on_shutdown = true;
        RCLCPP_WARN(this->get_logger(), "Failed to load initial system settings (using defaults).");
    }

    // 加载调压器设置 (ID 1 & 2)
    for (uint8_t id = 1; id <= 2; ++id) {
        ros2_interfaces::msg::RegulatorSettings settings;
        if (db_manager_->get_regulator_settings(id, settings)) {
            current_regulator_settings_[id] = settings;
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to load regulator settings for ID %d", id);
        }
    }

    // 加载回路设置 (ID 1 & 2)
    for (uint8_t id = 1; id <= 2; ++id) {
        ros2_interfaces::msg::CircuitSettings settings;
        if (db_manager_->get_circuit_settings(id, settings)) {
            current_circuit_settings_[id] = settings;
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to load circuit settings for ID %d", id);
        }
    }
}

// --- Settings Topic Callbacks ---

void RecordNode::system_settings_topic_callback(const ros2_interfaces::msg::SystemSettings::SharedPtr msg)
{
    // 检查是否有任何变化
    if (*msg != current_system_settings_) {
        RCLCPP_INFO(this->get_logger(), "Detected System Settings change. Updating DB.");

        // 检查记录间隔是否发生了变化
        bool interval_changed = (msg->record_interval_min != current_system_settings_.record_interval_min);

        bool success = db_manager_->save_system_settings(*msg);
        if (success) {
            current_system_settings_ = *msg;
            keep_record_on_shutdown_ = msg->keep_record_on_shutdown; // 同步其他参数

            // 如果时间间隔变了，必须重置定时器
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

    // 如果内存中没有这个ID，或者内容不一致，则更新
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

    // 如果内存中没有这个ID，或者内容不一致，则更新
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

// --- Status Topic Callbacks ---
void RecordNode::circuit_status_callback(const ros2_interfaces::msg::CircuitStatus::SharedPtr msg)
{
    latest_circuit_status_[msg->circuit_id] = *msg;
}

void RecordNode::regulator_status_callback(const ros2_interfaces::msg::RegulatorStatus::SharedPtr msg)
{
    latest_regulator_status_[msg->regulator_id] = *msg;
}

// --- Get/Query Service Callbacks ---
void RecordNode::get_system_settings_callback(
    const std::shared_ptr<ros2_interfaces::srv::GetSystemSettings::Request> /*request*/,
    std::shared_ptr<ros2_interfaces::srv::GetSystemSettings::Response> response)
{
    // 优先从内存返回，效率更高，且保证一致性
    // 如果需要强制读盘，可以改回调用 db_manager_
    response->settings = current_system_settings_;
    response->success = true;
    // 备用：response->success = db_manager_->get_system_settings(response->settings);
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
        // 尝试从DB读取（以防万一内存中没有）
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
        // 尝试从DB读取
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

// --- Recording Logic ---
void RecordNode::reschedule_timers()
{
    // 1. 停止现有的定时器，防止冲突
    if (alignment_timer_ && !alignment_timer_->is_canceled()) {
        alignment_timer_->cancel();
    }
    if (record_timer_ && !record_timer_->is_canceled()) {
        record_timer_->cancel();
    }

    // 2. 获取当前的间隔设置 (确保至少为1分钟)
    int interval_min = current_system_settings_.record_interval_min;
    if (interval_min < 1) interval_min = 1;

    // 更新成员变量以备他用
    this->record_interval_min_ = interval_min;

    // 3. 计算距离下一个“整 interval_min 分钟”的延时
    // 例如：当前 10:03:30，间隔 5分钟。下一个时刻应为 10:05:00。
    // 计算方法：当前总秒数 % (5*60) = 余数。 延时 = (5*60) - 余数。

    auto now = std::chrono::system_clock::now();
    time_t t = std::chrono::system_clock::to_time_t(now);
    struct tm tm_struct;
#ifdef _MSC_VER
    gmtime_s(&tm_struct, &t);
#else
    gmtime_r(&t, &tm_struct);
#endif

    // 当前小时内的分钟数 * 60 + 当前秒数 = 当前小时已过的秒数
    // 注意：我们其实只需要基于分钟对齐，不需要基于小时对齐（比如每90分钟），
    // 但通常“整X分钟”是指相对于小时的 0, 5, 10...
    // 所以我们计算相对于小时起点的秒数。
    long current_seconds_in_hour = tm_struct.tm_min * 60 + tm_struct.tm_sec;
    long interval_seconds = interval_min * 60;

    // 计算还需要多少秒到达下一个整点
    long seconds_to_wait = interval_seconds - (current_seconds_in_hour % interval_seconds);

    // 如果计算结果恰好是0（极小概率刚好卡在整点毫秒级），为了避免立即触发导致逻辑混乱，可以延后一个周期，
    // 或者直接让它立即触发。为了逻辑简单，这里添加 100ms 缓冲确保它在整点之后一点点执行。
    auto delay = std::chrono::seconds(seconds_to_wait) + std::chrono::milliseconds(100);

    RCLCPP_INFO(this->get_logger(),
                "Scheduling next record in %ld seconds (Aligning to %d min interval)",
                seconds_to_wait, interval_min);

    // 4. 创建对齐定时器 (One-shot)
    alignment_timer_ = this->create_wall_timer(
        delay,
        [this]() {
            // 对齐定时器触发：
            // 1. 立即停止自己 (One-shot)
            this->alignment_timer_->cancel();

            // 2. 执行一次记录任务
            this->record_timer_callback();

            // 3. 创建周期性定时器，按照设定的间隔循环执行
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

            // 判断是否记录
            bool should_record = keep_record_on_shutdown_
                                 || current_circuit_settings_.at(id).test_loop.enabled
                                 || current_circuit_settings_.at(id).ref_loop.enabled;

            if (should_record) {
                db_manager_->insert_data_record(time_str, circuit, regulator);
            }
        }
    }
}
