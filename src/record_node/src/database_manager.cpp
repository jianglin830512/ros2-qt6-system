#include "record_node/database_manager.hpp"
#include <iomanip>
#include <sstream>
#include <ctime>
#include <cstdio>

namespace{

time_t custom_timegm(struct tm* tm_ptr) {
#ifdef _MSC_VER
    return _mkgmtime(tm_ptr);
#else
    return timegm(tm_ptr);
#endif
}

std::string time_to_iso_string(const builtin_interfaces::msg::Time& time_msg)
{
    time_t seconds = time_msg.sec;
    struct tm tm_struct;
#ifdef _MSC_VER
    gmtime_s(&tm_struct, &seconds);
#else
    gmtime_r(&seconds, &tm_struct);
#endif

    std::stringstream ss;
    ss << std::put_time(&tm_struct, "%Y-%m-%dT%H:%M:%S");
    ss << "." << std::setfill('0') << std::setw(9) << time_msg.nanosec << "Z";
    return ss.str();
}

// 辅助函数：安全地将 sqlite3 列转换为 string，处理 NULL
std::string safe_column_text(sqlite3_stmt* stmt, int col_idx) {
    const char* text = (const char*)sqlite3_column_text(stmt, col_idx);
    return text ? std::string(text) : std::string("");
}

static builtin_interfaces::msg::Time iso_string_to_time(const std::string& iso_str) {
    builtin_interfaces::msg::Time t;
    struct tm tm_struct = {};
    std::istringstream ss(iso_str);

    // 1. 处理日期和时间部分
    // 检查是否包含 'T' (ISO格式) 还是空格 (普通格式)
    if (iso_str.find('T') != std::string::npos) {
        ss >> std::get_time(&tm_struct, "%Y-%m-%dT%H:%M:%S");
    } else {
        ss >> std::get_time(&tm_struct, "%Y-%m-%d %H:%M:%S");
    }

    if (ss.fail()) {
        // 如果解析失败，返回零值或记录错误
        t.sec = 0;
        t.nanosec = 0;
        return t;
    }

    // 转换为 Unix 时间戳
    t.sec = static_cast<int32_t>(custom_timegm(&tm_struct));

    // 2. 处理纳秒部分 (如果有小数点)
    size_t dot_pos = iso_str.find('.');
    if (dot_pos != std::string::npos) {
        std::string nano_part = iso_str.substr(dot_pos + 1);
        // 去掉末尾的 'Z'
        if (!nano_part.empty() && nano_part.back() == 'Z') {
            nano_part.pop_back();
        }

        try {
            // 确保只取前 9 位，不足则补 0 (处理 .123 -> 123000000)
            if (nano_part.length() > 9) {
                nano_part = nano_part.substr(0, 9);
            } else {
                nano_part.append(9 - nano_part.length(), '0');
            }
            t.nanosec = static_cast<uint32_t>(std::stoul(nano_part));
        } catch (...) {
            t.nanosec = 0;
        }
    } else {
        t.nanosec = 0;
    }

    return t;
}

double duration_to_seconds(const builtin_interfaces::msg::Duration& duration_msg)
{
    return static_cast<double>(duration_msg.sec) + static_cast<double>(duration_msg.nanosec) / 1e9;
}
}

DatabaseManager::DatabaseManager(const std::string& db_path, rclcpp::Logger logger)
    : db_(nullptr), logger_(logger)
{
    if (sqlite3_open(db_path.c_str(), &db_) != SQLITE_OK) {
        RCLCPP_ERROR(logger_, "Could not open database: %s", sqlite3_errmsg(db_));
        db_ = nullptr;
    } else {
        RCLCPP_INFO(logger_, "Database opened successfully: %s", db_path.c_str());
        initialize_database();
    }
}

DatabaseManager::~DatabaseManager()
{
    if (db_) {
        sqlite3_close(db_);
        RCLCPP_INFO(logger_, "Database closed.");
    }
}

bool DatabaseManager::execute_sql(const char* sql, const char* context_msg)
{
    if (!db_) return false;
    int rc = sqlite3_exec(db_, sql, 0, 0, &db_err_msg_);
    if (rc != SQLITE_OK) {
        RCLCPP_ERROR(logger_, "Failed to %s: %s", context_msg, db_err_msg_);
        sqlite3_free(db_err_msg_);
        return false;
    }
    return true;
}

void DatabaseManager::initialize_database()
{
    // 1. 创建 system_settings 表
    const char* create_system_settings_sql =
        "CREATE TABLE IF NOT EXISTS system_settings ("
        "id INTEGER PRIMARY KEY,"
        "sample_interval_sec INTEGER,"
        "record_interval_min INTEGER,"
        "keep_record_on_shutdown BOOLEAN,"
        "update_time DATETIME DEFAULT CURRENT_TIMESTAMP);";
    if (!execute_sql(create_system_settings_sql, "create system_settings table")) return;

    // 2. 创建 regulator_settings 表
    const char* create_regulator_settings_sql =
        "CREATE TABLE IF NOT EXISTS regulator_settings ("
        "regulator_id INTEGER PRIMARY KEY,"
        "over_current_a INTEGER,"
        "over_voltage_v INTEGER,"
        "voltage_up_speed_percent INTEGER,"
        "voltage_down_speed_percent INTEGER,"
        "over_voltage_protection_mode BOOLEAN,"
        "update_time DATETIME DEFAULT CURRENT_TIMESTAMP);";
    if (!execute_sql(create_regulator_settings_sql, "create regulator_settings table")) return;

    // 3. 创建 circuit_settings 表
    const char* create_circuit_settings_sql =
        "CREATE TABLE IF NOT EXISTS circuit_settings ("
        "circuit_id INTEGER PRIMARY KEY,"
        // 试验回路参数
        "test_start_current_a INTEGER, test_max_current_a INTEGER, test_current_change_range_percent INTEGER, "
        "test_ct_ratio INTEGER, "
        "test_start_date TEXT, "
        "test_heating_time REAL, "
        "test_cycle_count INTEGER, test_heating_duration REAL, "
        // 参照回路参数
        "ref_start_current_a INTEGER, ref_max_current_a INTEGER, ref_current_change_range_percent INTEGER, "
        "ref_ct_ratio INTEGER, "
        "ref_start_date TEXT, "
        "ref_heating_time REAL, "
        "ref_cycle_count INTEGER, ref_heating_duration REAL, "
        // 试样参数
        "cable_type TEXT, cable_spec TEXT, insulation_material TEXT, insulation_thickness REAL, "
        "update_time DATETIME DEFAULT CURRENT_TIMESTAMP);";
    if (!execute_sql(create_circuit_settings_sql, "create circuit_settings table")) return;

    // 4. 检查3个 settings 表的内容
    ensure_default_settings();

    // 5. 创建 data_records 表
    // 修改说明: 增加了 test_loop_enabled 和 ref_loop_enabled 列
    std::stringstream ss_create;
    ss_create << "CREATE TABLE IF NOT EXISTS data_records ("
              << "record_id INTEGER PRIMARY KEY AUTOINCREMENT, "
              << "record_time DATETIME, "
              << "circuit_id INTEGER, "
              << "control_mode INTEGER, "
              << "regulator_voltage REAL, "
              << "regulator_current REAL, "
              << "regulator_breaker_closed BOOLEAN, "
              << "test_loop_is_heat BOOLEAN, "
              << "ref_loop_is_heat BOOLEAN, "
              << "test_loop_current REAL, "
              << "ref_loop_current REAL, "
              << "test_loop_breaker_closed BOOLEAN, "
              << "ref_loop_breaker_closed BOOLEAN";

    // 添加 Test Loop 温度列 (01-16)
    for (int i = 1; i <= 16; ++i) {
        ss_create << ", test_loop_temp" << std::setfill('0') << std::setw(2) << i << " REAL";
    }
    // 添加 Ref Loop 温度列 (01-08)
    for (int i = 1; i <= 8; ++i) {
        ss_create << ", ref_loop_temp" << std::setfill('0') << std::setw(2) << i << " REAL";
    }
    ss_create << ");";

    if (!execute_sql(ss_create.str().c_str(), "create data_records table")) return;

    RCLCPP_INFO(logger_, "Database tables initialized successfully.");
}

void DatabaseManager::ensure_default_settings()
{
    // 1. 补全 System Settings (ID=1)
    // 默认: 采样1秒, 记录1分钟, 关机继续记录 True
    const char* sys_sql =
        "INSERT OR IGNORE INTO system_settings "
        "(id, sample_interval_sec, record_interval_min, keep_record_on_shutdown) "
        "VALUES (1, 1, 1, 1);";
    execute_sql(sys_sql, "ensure default system_settings");

    // 2. 补全 Regulator Settings (ID=1, ID=2)
    // 默认: 过流100A, 过压250V, 升降速10%, 保护模式开启
    const char* reg_sql =
        "INSERT OR IGNORE INTO regulator_settings "
        "(regulator_id, over_current_a, over_voltage_v, voltage_up_speed_percent, voltage_down_speed_percent, over_voltage_protection_mode) "
        "VALUES (?, 100, 250, 10, 10, 1);";

    sqlite3_stmt* reg_stmt;
    if (sqlite3_prepare_v2(db_, reg_sql, -1, &reg_stmt, nullptr) == SQLITE_OK) {
        for (int id = 1; id <= 2; ++id) {
            sqlite3_bind_int(reg_stmt, 1, id);
            if (sqlite3_step(reg_stmt) != SQLITE_DONE) {
                RCLCPP_ERROR(logger_, "Failed to insert default regulator %d", id);
            }
            sqlite3_reset(reg_stmt);
        }
        sqlite3_finalize(reg_stmt);
    } else {
        RCLCPP_ERROR(logger_, "Failed to prepare regulator default SQL");
    }

    // 3. 补全 Circuit Settings (ID=1, ID=2)
    // 默认: 电流0, 时间0, 文本为空, CT变比默认1(防止除0), 默认日期1970
    const char* cir_sql =
        "INSERT OR IGNORE INTO circuit_settings ("
        "circuit_id, "
        "test_start_current_a, test_max_current_a, test_current_change_range_percent, test_ct_ratio, "
        "test_start_date, test_heating_time, test_cycle_count, test_heating_duration, "
        "ref_start_current_a, ref_max_current_a, ref_current_change_range_percent, ref_ct_ratio, "
        "ref_start_date, ref_heating_time, ref_cycle_count, ref_heating_duration, "
        "cable_type, cable_spec, insulation_material, insulation_thickness) "
        "VALUES (?, "
        "0, 0, 0, 1, " // Test params (CT=1)
        "'1970-01-01T00:00:00Z', 0.0, 0, 0.0, "
        "0, 0, 0, 1, " // Ref params (CT=1)
        "'1970-01-01T00:00:00Z', 0.0, 0, 0.0, "
        "'', '', '', 0.0);"; // Sample params

    sqlite3_stmt* cir_stmt;
    if (sqlite3_prepare_v2(db_, cir_sql, -1, &cir_stmt, nullptr) == SQLITE_OK) {
        for (int id = 1; id <= 2; ++id) {
            sqlite3_bind_int(cir_stmt, 1, id);
            if (sqlite3_step(cir_stmt) != SQLITE_DONE) {
                RCLCPP_ERROR(logger_, "Failed to insert default circuit %d", id);
            }
            sqlite3_reset(cir_stmt);
        }
        sqlite3_finalize(cir_stmt);
    } else {
        RCLCPP_ERROR(logger_, "Failed to prepare circuit default SQL");
    }
}

bool DatabaseManager::save_system_settings(const ros2_interfaces::msg::SystemSettings& settings)
{
    if (!db_) return false;

    const char* sql = "INSERT OR REPLACE INTO system_settings "
                      "(id, sample_interval_sec, record_interval_min, keep_record_on_shutdown, update_time) "
                      "VALUES (1, ?, ?, ?, CURRENT_TIMESTAMP);";

    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        RCLCPP_ERROR(logger_, "Failed to prepare system_settings SQL statement: %s", sqlite3_errmsg(db_));
        return false;
    }

    sqlite3_bind_int(stmt, 1, settings.sample_interval_sec);
    sqlite3_bind_int(stmt, 2, settings.record_interval_min);
    sqlite3_bind_int(stmt, 3, settings.keep_record_on_shutdown ? 1 : 0);

    bool success = (sqlite3_step(stmt) == SQLITE_DONE);
    if (!success) {
        RCLCPP_ERROR(logger_, "Failed to execute system_settings save: %s", sqlite3_errmsg(db_));
    } else {
        RCLCPP_INFO(logger_, "System settings saved to database successfully.");
    }

    sqlite3_finalize(stmt);
    return success;
}

bool DatabaseManager::save_regulator_settings(uint8_t regulator_id, const ros2_interfaces::msg::RegulatorSettings& settings)
{
    if (!db_) return false;

    const char* sql = "INSERT OR REPLACE INTO regulator_settings "
                      "(regulator_id, over_current_a, over_voltage_v, voltage_up_speed_percent, "
                      "voltage_down_speed_percent, over_voltage_protection_mode, update_time) "
                      "VALUES (?, ?, ?, ?, ?, ?, CURRENT_TIMESTAMP);";

    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        RCLCPP_ERROR(logger_, "Failed to prepare regulator_settings SQL statement: %s", sqlite3_errmsg(db_));
        return false;
    }

    sqlite3_bind_int(stmt, 1, regulator_id);
    sqlite3_bind_int(stmt, 2, settings.over_current_a);
    sqlite3_bind_int(stmt, 3, settings.over_voltage_v);
    sqlite3_bind_int(stmt, 4, settings.voltage_up_speed_percent);
    sqlite3_bind_int(stmt, 5, settings.voltage_down_speed_percent);
    sqlite3_bind_int(stmt, 6, settings.over_voltage_protection_mode ? 1 : 0);

    bool success = (sqlite3_step(stmt) == SQLITE_DONE);
    if (!success) {
        RCLCPP_ERROR(logger_, "Failed to execute regulator_settings save for ID %u: %s", regulator_id, sqlite3_errmsg(db_));
    } else {
        RCLCPP_INFO(logger_, "Settings for regulator %u saved to database successfully.", regulator_id);
    }

    sqlite3_finalize(stmt);
    return success;
}

bool DatabaseManager::save_circuit_settings(uint8_t circuit_id, const ros2_interfaces::msg::CircuitSettings& settings)
{
    if (!db_) return false;

    const char* sql = "INSERT OR REPLACE INTO circuit_settings "
                      "(circuit_id, "
                      "test_start_current_a, test_max_current_a, test_current_change_range_percent, test_ct_ratio, "
                      "test_start_date, test_heating_time, "
                      "test_cycle_count, test_heating_duration, "
                      "ref_start_current_a, ref_max_current_a, ref_current_change_range_percent, ref_ct_ratio, "
                      "ref_start_date, ref_heating_time, "
                      "ref_cycle_count, ref_heating_duration, "
                      "cable_type, cable_spec, insulation_material, insulation_thickness, "
                      "update_time) "
                      "VALUES (?, "
                      "?, ?, ?, ?, ?, ?, ?, ?, "
                      "?, ?, ?, ?, ?, ?, ?, ?, "
                      "?, ?, ?, ?, "
                      "CURRENT_TIMESTAMP);";

    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        RCLCPP_ERROR(logger_, "Failed to prepare circuit_settings SQL statement: %s", sqlite3_errmsg(db_));
        return false;
    }

    // --- 数据转换 ---
    std::string test_start_date_iso = time_to_iso_string(settings.test_loop.start_date);
    double test_heating_time_sec = duration_to_seconds(settings.test_loop.heating_time);
    double test_duration_sec = duration_to_seconds(settings.test_loop.heating_duration);

    std::string ref_start_date_iso = time_to_iso_string(settings.ref_loop.start_date);
    double ref_heating_time_sec = duration_to_seconds(settings.ref_loop.heating_time);
    double ref_duration_sec = duration_to_seconds(settings.ref_loop.heating_duration);

    // --- 绑定参数 ---
    int idx = 1;
    sqlite3_bind_int(stmt, idx++, circuit_id);

    // Test Loop
    sqlite3_bind_int(stmt, idx++, settings.test_loop.hardware_loop_settings.start_current_a);
    sqlite3_bind_int(stmt, idx++, settings.test_loop.hardware_loop_settings.max_current_a);
    sqlite3_bind_int(stmt, idx++, settings.test_loop.hardware_loop_settings.current_change_range_percent);
    sqlite3_bind_int(stmt, idx++, settings.test_loop.hardware_loop_settings.ct_ratio);
    sqlite3_bind_text(stmt, idx++, test_start_date_iso.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_double(stmt, idx++, test_heating_time_sec);
    sqlite3_bind_int(stmt, idx++, settings.test_loop.cycle_count);
    sqlite3_bind_double(stmt, idx++, test_duration_sec);

    // Ref Loop
    sqlite3_bind_int(stmt, idx++, settings.ref_loop.hardware_loop_settings.start_current_a);
    sqlite3_bind_int(stmt, idx++, settings.ref_loop.hardware_loop_settings.max_current_a);
    sqlite3_bind_int(stmt, idx++, settings.ref_loop.hardware_loop_settings.current_change_range_percent);
    sqlite3_bind_int(stmt, idx++, settings.ref_loop.hardware_loop_settings.ct_ratio);
    sqlite3_bind_text(stmt, idx++, ref_start_date_iso.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_double(stmt, idx++, ref_heating_time_sec);
    sqlite3_bind_int(stmt, idx++, settings.ref_loop.cycle_count);
    sqlite3_bind_double(stmt, idx++, ref_duration_sec);

    // Sample Params
    sqlite3_bind_text(stmt, idx++, settings.sample_params.cable_type.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, idx++, settings.sample_params.cable_spec.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, idx++, settings.sample_params.insulation_material.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_double(stmt, idx++, settings.sample_params.insulation_thickness);

    bool success = (sqlite3_step(stmt) == SQLITE_DONE);
    if (!success) {
        RCLCPP_ERROR(logger_, "Failed to execute circuit_settings save for ID %u: %s", circuit_id, sqlite3_errmsg(db_));
    } else {
        RCLCPP_INFO(logger_, "Settings for circuit %u saved to database successfully.", circuit_id);
    }

    sqlite3_finalize(stmt);
    return success;
}

bool DatabaseManager::insert_data_record(
    const std::string& record_time_str,
    const ros2_interfaces::msg::CircuitStatus& circuit_status,
    const ros2_interfaces::msg::RegulatorStatus& regulator_status)
{
    if (!db_) return false;

    // 修改说明: 插入语句增加 test_loop_enabled, ref_loop_enabled
    std::stringstream ss_sql;
    ss_sql << "INSERT INTO data_records ("
           << "record_time, circuit_id, control_mode, "
           << "regulator_voltage, regulator_current, regulator_breaker_closed, "
           << "test_loop_is_heat, ref_loop_is_heat, " // 新增
           << "test_loop_current, ref_loop_current, test_loop_breaker_closed, ref_loop_breaker_closed";

    for (int i = 1; i <= 16; ++i) ss_sql << ", test_loop_temp" << std::setfill('0') << std::setw(2) << i;
    for (int i = 1; i <= 8; ++i) ss_sql << ", ref_loop_temp" << std::setfill('0') << std::setw(2) << i;

    ss_sql << ") VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?"; // 这里现在是 12 个问号 (包含新增的2个)

    // 为温度列添加占位符 (16 + 8 = 24 个)
    for (int i = 0; i < 24; ++i) ss_sql << ", ?";
    ss_sql << ");";

    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, ss_sql.str().c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        RCLCPP_ERROR(logger_, "Failed to prepare insert_data_record SQL: %s", sqlite3_errmsg(db_));
        return false;
    }

    int idx = 1;
    // 1. 基础数据
    sqlite3_bind_text(stmt, idx++, record_time_str.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, idx++, circuit_status.circuit_id);
    sqlite3_bind_int(stmt, idx++, circuit_status.control_mode);

    // 2. 调压器数据
    sqlite3_bind_double(stmt, idx++, regulator_status.voltage_reading);
    sqlite3_bind_double(stmt, idx++, regulator_status.current_reading);
    sqlite3_bind_int(stmt, idx++, regulator_status.breaker_closed_switch_ack ? 1 : 0);

    // 3. 回路使能状态 (新增)
    sqlite3_bind_int(stmt, idx++, circuit_status.test_loop.is_heat  ? 1 : 0);
    sqlite3_bind_int(stmt, idx++, circuit_status.ref_loop.is_heat  ? 1 : 0);

    // 4. 回路电流与开关
    sqlite3_bind_double(stmt, idx++, circuit_status.test_loop.hardware_loop_status.current);
    sqlite3_bind_double(stmt, idx++, circuit_status.ref_loop.hardware_loop_status.current);
    sqlite3_bind_int(stmt, idx++, circuit_status.test_loop.hardware_loop_status.breaker_closed_switch_ack ? 1 : 0);
    sqlite3_bind_int(stmt, idx++, circuit_status.ref_loop.hardware_loop_status.breaker_closed_switch_ack ? 1 : 0);

    // 5. 绑定 Test Loop 温度 (16个)
    for (int i = 0; i < 16; ++i) {
        if (i < 16)
            sqlite3_bind_double(stmt, idx++, circuit_status.test_loop.hardware_loop_status.temperature_array[i]);
        else
            sqlite3_bind_null(stmt, idx++);
    }

    // 6. 绑定 Ref Loop 温度 (8个)
    for (int i = 0; i < 8; ++i) {
        if (i < 16)
            sqlite3_bind_double(stmt, idx++, circuit_status.ref_loop.hardware_loop_status.temperature_array[i]);
        else
            sqlite3_bind_null(stmt, idx++);
    }

    bool success = (sqlite3_step(stmt) == SQLITE_DONE);
    if (!success) {
        RCLCPP_ERROR(logger_, "Failed to execute data record insert: %s", sqlite3_errmsg(db_));
    }

    sqlite3_finalize(stmt);
    return success;
}

bool DatabaseManager::get_system_settings(ros2_interfaces::msg::SystemSettings& settings) {
    const char* sql = "SELECT sample_interval_sec, record_interval_min, keep_record_on_shutdown FROM system_settings WHERE id = 1;";
    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) return false;

    bool found = false;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        settings.sample_interval_sec = sqlite3_column_int(stmt, 0);
        settings.record_interval_min = sqlite3_column_int(stmt, 1);
        settings.keep_record_on_shutdown = sqlite3_column_int(stmt, 2) != 0;
        found = true;
    }
    sqlite3_finalize(stmt);
    return found;
}

bool DatabaseManager::get_regulator_settings(uint8_t regulator_id, ros2_interfaces::msg::RegulatorSettings& settings) {
    const char* sql = "SELECT over_current_a, over_voltage_v, voltage_up_speed_percent, voltage_down_speed_percent, over_voltage_protection_mode "
                      "FROM regulator_settings WHERE regulator_id = ?;";
    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) return false;
    sqlite3_bind_int(stmt, 1, regulator_id);

    bool found = false;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        settings.regulator_id = regulator_id;
        settings.over_current_a = sqlite3_column_int(stmt, 0);
        settings.over_voltage_v = sqlite3_column_int(stmt, 1);
        settings.voltage_up_speed_percent = sqlite3_column_int(stmt, 2);
        settings.voltage_down_speed_percent = sqlite3_column_int(stmt, 3);
        settings.over_voltage_protection_mode = sqlite3_column_int(stmt, 4) != 0;
        found = true;
    }
    sqlite3_finalize(stmt);
    return found;
}

bool DatabaseManager::get_circuit_settings(uint8_t circuit_id, ros2_interfaces::msg::CircuitSettings& settings) {
    const char* sql = "SELECT * FROM circuit_settings WHERE circuit_id = ?;";
    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) return false;
    sqlite3_bind_int(stmt, 1, circuit_id);

    bool found = false;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        settings.circuit_id = circuit_id;

        // --- Test Loop ---
        settings.test_loop.hardware_loop_settings.start_current_a = sqlite3_column_int(stmt, 1);
        settings.test_loop.hardware_loop_settings.max_current_a = sqlite3_column_int(stmt, 2);
        settings.test_loop.hardware_loop_settings.current_change_range_percent = sqlite3_column_int(stmt, 3);
        settings.test_loop.hardware_loop_settings.ct_ratio = sqlite3_column_int(stmt, 4);

        // 修改点：使用 safe_column_text 防止 NULL 崩溃
        settings.test_loop.start_date = iso_string_to_time(safe_column_text(stmt, 5));

        settings.test_loop.heating_time.sec = (int32_t)sqlite3_column_double(stmt, 6);
        settings.test_loop.cycle_count = sqlite3_column_int(stmt, 7);
        settings.test_loop.heating_duration.sec = (int32_t)sqlite3_column_double(stmt, 8);

        // --- Ref Loop ---
        settings.ref_loop.hardware_loop_settings.start_current_a = sqlite3_column_int(stmt, 9);
        settings.ref_loop.hardware_loop_settings.max_current_a = sqlite3_column_int(stmt, 10);
        settings.ref_loop.hardware_loop_settings.current_change_range_percent = sqlite3_column_int(stmt, 11);
        settings.ref_loop.hardware_loop_settings.ct_ratio = sqlite3_column_int(stmt, 12);

        // 修改点：安全转换
        settings.ref_loop.start_date = iso_string_to_time(safe_column_text(stmt, 13));

        settings.ref_loop.heating_time.sec = (int32_t)sqlite3_column_double(stmt, 14);
        settings.ref_loop.cycle_count = sqlite3_column_int(stmt, 15);
        settings.ref_loop.heating_duration.sec = (int32_t)sqlite3_column_double(stmt, 16);

        // --- Sample Params ---
        // 修改点：安全转换
        settings.sample_params.cable_type = safe_column_text(stmt, 17);
        settings.sample_params.cable_spec = safe_column_text(stmt, 18);
        settings.sample_params.insulation_material = safe_column_text(stmt, 19);
        settings.sample_params.insulation_thickness = sqlite3_column_double(stmt, 20);

        found = true;
    }
    sqlite3_finalize(stmt);
    return found;
}

std::vector<ros2_interfaces::msg::DataRecord> DatabaseManager::get_data_records(const std::string& start, const std::string& end) {
    std::vector<ros2_interfaces::msg::DataRecord> results;
    const char* sql = "SELECT * FROM data_records WHERE record_time BETWEEN ? AND ? ORDER BY record_time ASC;";
    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) return results;

    sqlite3_bind_text(stmt, 1, start.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, end.c_str(), -1, SQLITE_TRANSIENT);

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        ros2_interfaces::msg::DataRecord rec;
        int idx = 1; // 跳过 record_id
        rec.record_time = (const char*)sqlite3_column_text(stmt, idx++);
        rec.circuit_id = sqlite3_column_int(stmt, idx++);
        rec.control_mode = sqlite3_column_int(stmt, idx++);
        rec.regulator_voltage = sqlite3_column_double(stmt, idx++);
        rec.regulator_current = sqlite3_column_double(stmt, idx++);
        rec.regulator_breaker_closed = sqlite3_column_int(stmt, idx++) != 0;
        rec.test_loop_is_heat = sqlite3_column_int(stmt, idx++) != 0;
        rec.ref_loop_is_heat = sqlite3_column_int(stmt, idx++) != 0;
        rec.test_loop_current = sqlite3_column_double(stmt, idx++);
        rec.ref_loop_current = sqlite3_column_double(stmt, idx++);
        rec.test_loop_breaker_closed = sqlite3_column_int(stmt, idx++) != 0;
        rec.ref_loop_breaker_closed = sqlite3_column_int(stmt, idx++) != 0;

        for(int i=0; i<16; ++i) rec.test_loop_temp[i] = sqlite3_column_double(stmt, idx++);
        for(int i=0; i<8; ++i) rec.ref_loop_temp[i] = sqlite3_column_double(stmt, idx++);

        results.push_back(rec);
    }
    sqlite3_finalize(stmt);
    return results;
}
