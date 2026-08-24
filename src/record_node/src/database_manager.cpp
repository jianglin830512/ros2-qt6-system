#include "record_node/database_manager.hpp"
#include <iomanip>
#include <sstream>
#include <ctime>
#include <cstdio>
#include <set>

namespace{
time_t custom_timegm(struct tm* tm_ptr) {
#ifdef _MSC_VER
    return _mkgmtime(tm_ptr);
#else
    return timegm(tm_ptr);
#endif
}

std::string time_to_iso_string(const builtin_interfaces::msg::Time& time_msg) {
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

std::string safe_column_text(sqlite3_stmt* stmt, int col_idx) {
    const char* text = (const char*)sqlite3_column_text(stmt, col_idx);
    return text ? std::string(text) : std::string("");
}

static builtin_interfaces::msg::Time iso_string_to_time(const std::string& iso_str) {
    builtin_interfaces::msg::Time t;
    struct tm tm_struct = {};
    std::istringstream ss(iso_str);
    if (iso_str.find('T') != std::string::npos) {
        ss >> std::get_time(&tm_struct, "%Y-%m-%dT%H:%M:%S");
    } else {
        ss >> std::get_time(&tm_struct, "%Y-%m-%d %H:%M:%S");
    }
    if (ss.fail()) {
        t.sec = 0; t.nanosec = 0; return t;
    }
    t.sec = static_cast<int32_t>(custom_timegm(&tm_struct));
    size_t dot_pos = iso_str.find('.');
    if (dot_pos != std::string::npos) {
        std::string nano_part = iso_str.substr(dot_pos + 1);
        if (!nano_part.empty() && nano_part.back() == 'Z') nano_part.pop_back();
        try {
            if (nano_part.length() > 9) nano_part = nano_part.substr(0, 9);
            else nano_part.append(9 - nano_part.length(), '0');
            t.nanosec = static_cast<uint32_t>(std::stoul(nano_part));
        } catch (...) { t.nanosec = 0; }
    } else { t.nanosec = 0; }
    return t;
}

double duration_to_seconds(const builtin_interfaces::msg::Duration& duration_msg) {
    return static_cast<double>(duration_msg.sec) + static_cast<double>(duration_msg.nanosec) / 1e9;
}
} // end namespace

DatabaseManager::DatabaseManager(const std::string& db_path, rclcpp::Logger logger)
    : db_(nullptr), logger_(logger)
{
    if (sqlite3_open(db_path.c_str(), &db_) != SQLITE_OK) {
        RCLCPP_ERROR(logger_, "Could not open database: %s", sqlite3_errmsg(db_));
        db_ = nullptr;
        update_connection_status(false);
    } else {
        RCLCPP_INFO(logger_, "Database opened successfully: %s", db_path.c_str());
        update_connection_status(true);
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
    update_connection_status(rc == SQLITE_OK);
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
        "auto_on BOOLEAN,"
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

    // 3. 创建 circuit_settings 表 (移除 cable 相关的4个废弃字段，替换为只记录 cable_id)
    const char* create_circuit_settings_sql =
        "CREATE TABLE IF NOT EXISTS circuit_settings ("
        "circuit_id INTEGER PRIMARY KEY,"
        "test_start_current_a INTEGER, test_max_current_a INTEGER, test_current_change_range_percent INTEGER, test_ct_ratio INTEGER, "
        "test_start_date TEXT, test_heating_time REAL, test_cycle_count INTEGER, test_heating_duration REAL, test_loop_enabled BOOLEAN, test_auto_strategy INTEGER, "
        "ref_start_current_a INTEGER, ref_max_current_a INTEGER, ref_current_change_range_percent INTEGER, ref_ct_ratio INTEGER, "
        "ref_start_date TEXT, ref_heating_time REAL, ref_cycle_count INTEGER, ref_heating_duration REAL, ref_loop_enabled BOOLEAN, ref_auto_strategy INTEGER, "
        "cable_id INTEGER, "
        "update_time DATETIME DEFAULT CURRENT_TIMESTAMP);";
    if (!execute_sql(create_circuit_settings_sql, "create circuit_settings table")) return;

    // 4. 初始化默认值
    ensure_default_settings();

    // 5. 创建 data_records 表
    std::stringstream ss_create;
    ss_create << "CREATE TABLE IF NOT EXISTS data_records ("
              << "record_id INTEGER PRIMARY KEY AUTOINCREMENT, "
              << "record_time DATETIME, "
              << "circuit_id INTEGER, "
              << "auto_on BOOLEAN, "
              << "regulator_1_breaker_closed BOOLEAN, "
              << "regulator_1_voltage REAL, "
              << "regulator_1_current REAL, "
              << "test_loop_is_heat BOOLEAN, "
              << "test_loop_breaker_closed BOOLEAN, "
              << "test_loop_strategy INTEGER, "
              << "test_loop_current REAL, ";

    for (int i = 1; i <= 16; ++i) {
        ss_create << "test_loop_temp" << std::setfill('0') << std::setw(2) << i << " REAL, ";
    }

    ss_create << "regulator_2_breaker_closed BOOLEAN, "
              << "regulator_2_voltage REAL, "
              << "regulator_2_current REAL, "
              << "ref_loop_is_heat BOOLEAN, "
              << "ref_loop_breaker_closed BOOLEAN, "
              << "ref_loop_strategy INTEGER, "
              << "ref_loop_current REAL";

    for (int i = 1; i <= 16; ++i) {
        ss_create << ", ref_loop_temp" << std::setfill('0') << std::setw(2) << i << " REAL";
    }
    ss_create << ");";

    if (!execute_sql(ss_create.str().c_str(), "create data_records table")) return;

    RCLCPP_INFO(logger_, "Database tables initialized successfully.");
}

void DatabaseManager::ensure_default_settings()
{
    const char* sys_sql =
        "INSERT OR IGNORE INTO system_settings "
        "(id, sample_interval_sec, record_interval_min, keep_record_on_shutdown, auto_on) "
        "VALUES (1, 1, 1, 1, 0);";
    execute_sql(sys_sql, "ensure default system_settings");

    const char* reg_sql =
        "INSERT OR IGNORE INTO regulator_settings "
        "(regulator_id, over_current_a, over_voltage_v, voltage_up_speed_percent, voltage_down_speed_percent, over_voltage_protection_mode) "
        "VALUES (?, 100, 250, 10, 10, 1);";
    sqlite3_stmt* reg_stmt;
    if (sqlite3_prepare_v2(db_, reg_sql, -1, &reg_stmt, nullptr) == SQLITE_OK) {
        for (int id = 1; id <= 2; ++id) {
            sqlite3_bind_int(reg_stmt, 1, id);
            sqlite3_step(reg_stmt);
            sqlite3_reset(reg_stmt);
        }
        sqlite3_finalize(reg_stmt);
    }

    const char* cir_sql =
        "INSERT OR IGNORE INTO circuit_settings ("
        "circuit_id, "
        "test_start_current_a, test_max_current_a, test_current_change_range_percent, test_ct_ratio, "
        "test_start_date, test_heating_time, test_cycle_count, test_heating_duration, test_loop_enabled, test_auto_strategy, "
        "ref_start_current_a, ref_max_current_a, ref_current_change_range_percent, ref_ct_ratio, "
        "ref_start_date, ref_heating_time, ref_cycle_count, ref_heating_duration, ref_loop_enabled, ref_auto_strategy, "
        "cable_id) "
        "VALUES (?, "
        "0, 0, 0, 1, '1970-01-01T00:00:00Z', 0.0, 0, 0.0, 0, 1, "
        "0, 0, 0, 1, '1970-01-01T00:00:00Z', 0.0, 0, 0.0, 0, 1, "
        "0);"; // 默认 cable_id 置空为 0

    sqlite3_stmt* cir_stmt;
    if (sqlite3_prepare_v2(db_, cir_sql, -1, &cir_stmt, nullptr) == SQLITE_OK) {
        for (int id = 1; id <= 2; ++id) {
            sqlite3_bind_int(cir_stmt, 1, id);
            sqlite3_step(cir_stmt);
            sqlite3_reset(cir_stmt);
        }
        sqlite3_finalize(cir_stmt);
    }
}

bool DatabaseManager::save_system_settings(const ros2_interfaces::msg::SystemSettings& settings)
{
    if (!db_) return false;
    const char* sql = "INSERT OR REPLACE INTO system_settings "
                      "(id, sample_interval_sec, record_interval_min, keep_record_on_shutdown, auto_on, update_time) "
                      "VALUES (1, ?, ?, ?, ?, CURRENT_TIMESTAMP);";
    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) return false;

    sqlite3_bind_int(stmt, 1, settings.sample_interval_sec);
    sqlite3_bind_int(stmt, 2, settings.record_interval_min);
    sqlite3_bind_int(stmt, 3, settings.keep_record_on_shutdown ? 1 : 0);
    sqlite3_bind_int(stmt, 4, settings.auto_on ? 1 : 0);

    bool success = (sqlite3_step(stmt) == SQLITE_DONE);
    sqlite3_finalize(stmt);
    return success;
}

bool DatabaseManager::save_circuit_settings(uint8_t circuit_id, const ros2_interfaces::msg::CircuitSettings& settings)
{
    if (!db_) return false;
    const char* sql = "INSERT OR REPLACE INTO circuit_settings "
                      "(circuit_id, "
                      "test_start_current_a, test_max_current_a, test_current_change_range_percent, test_ct_ratio, "
                      "test_start_date, test_heating_time, test_cycle_count, test_heating_duration, test_loop_enabled, test_auto_strategy, "
                      "ref_start_current_a, ref_max_current_a, ref_current_change_range_percent, ref_ct_ratio, "
                      "ref_start_date, ref_heating_time, ref_cycle_count, ref_heating_duration, ref_loop_enabled, ref_auto_strategy, "
                      "cable_id, "
                      "update_time) "
                      "VALUES (?, "
                      "?, ?, ?, ?, ?, ?, ?, ?, ?, ?, "  // Test params (10变量)
                      "?, ?, ?, ?, ?, ?, ?, ?, ?, ?, "  // Ref params (10变量)
                      "?, "                             // Sample params (1个 cable_id 变量)
                      "CURRENT_TIMESTAMP);";

    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) return false;

    std::string test_start_date_iso = time_to_iso_string(settings.test_loop.start_date);
    double test_heating_time_sec = duration_to_seconds(settings.test_loop.heating_time);
    double test_duration_sec = duration_to_seconds(settings.test_loop.heating_duration);

    std::string ref_start_date_iso = time_to_iso_string(settings.ref_loop.start_date);
    double ref_heating_time_sec = duration_to_seconds(settings.ref_loop.heating_time);
    double ref_duration_sec = duration_to_seconds(settings.ref_loop.heating_duration);

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
    sqlite3_bind_int(stmt, idx++, settings.test_loop.enabled ? 1 : 0);
    sqlite3_bind_int(stmt, idx++, settings.test_loop.auto_strategy);

    // Ref Loop
    sqlite3_bind_int(stmt, idx++, settings.ref_loop.hardware_loop_settings.start_current_a);
    sqlite3_bind_int(stmt, idx++, settings.ref_loop.hardware_loop_settings.max_current_a);
    sqlite3_bind_int(stmt, idx++, settings.ref_loop.hardware_loop_settings.current_change_range_percent);
    sqlite3_bind_int(stmt, idx++, settings.ref_loop.hardware_loop_settings.ct_ratio);
    sqlite3_bind_text(stmt, idx++, ref_start_date_iso.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_double(stmt, idx++, ref_heating_time_sec);
    sqlite3_bind_int(stmt, idx++, settings.ref_loop.cycle_count);
    sqlite3_bind_double(stmt, idx++, ref_duration_sec);
    sqlite3_bind_int(stmt, idx++, settings.ref_loop.enabled ? 1 : 0);
    sqlite3_bind_int(stmt, idx++, settings.ref_loop.auto_strategy);

    // Cable ID 占位
    sqlite3_bind_int(stmt, idx++, settings.sample_cable.id);

    bool success = (sqlite3_step(stmt) == SQLITE_DONE);
    sqlite3_finalize(stmt);
    return success;
}

bool DatabaseManager::get_system_settings(ros2_interfaces::msg::SystemSettings& settings) {
    const char* sql = "SELECT sample_interval_sec, record_interval_min, keep_record_on_shutdown, auto_on FROM system_settings WHERE id = 1;";
    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) return false;

    bool found = false;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        settings.sample_interval_sec = sqlite3_column_int(stmt, 0);
        settings.record_interval_min = sqlite3_column_int(stmt, 1);
        settings.keep_record_on_shutdown = sqlite3_column_int(stmt, 2) != 0;
        settings.auto_on = sqlite3_column_int(stmt, 3) != 0;
        found = true;
    }
    sqlite3_finalize(stmt);
    return found;
}

bool DatabaseManager::get_circuit_settings(uint8_t circuit_id, ros2_interfaces::msg::CircuitSettings& settings) {
    // 显式指定查询的字段名，从而保证 sqlite3_column_*(stmt, index) 的索引永远绝对安全
    const char* sql = "SELECT "
                      "test_start_current_a, test_max_current_a, test_current_change_range_percent, test_ct_ratio, "
                      "test_start_date, test_heating_time, test_cycle_count, test_heating_duration, test_loop_enabled, test_auto_strategy, "
                      "ref_start_current_a, ref_max_current_a, ref_current_change_range_percent, ref_ct_ratio, "
                      "ref_start_date, ref_heating_time, ref_cycle_count, ref_heating_duration, ref_loop_enabled, ref_auto_strategy, "
                      "cable_id "
                      "FROM circuit_settings WHERE circuit_id = ?;";

    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        RCLCPP_ERROR(logger_, "Failed to prepare get_circuit_settings SQL: %s", sqlite3_errmsg(db_));
        return false;
    }

    sqlite3_bind_int(stmt, 1, circuit_id);

    bool found = false;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        settings.circuit_id = circuit_id;

        // Test Loop (索引 0 ~ 9)
        settings.test_loop.hardware_loop_settings.start_current_a = sqlite3_column_int(stmt, 0);
        settings.test_loop.hardware_loop_settings.max_current_a = sqlite3_column_int(stmt, 1);
        settings.test_loop.hardware_loop_settings.current_change_range_percent = sqlite3_column_int(stmt, 2);
        settings.test_loop.hardware_loop_settings.ct_ratio = sqlite3_column_int(stmt, 3);
        settings.test_loop.start_date = iso_string_to_time(safe_column_text(stmt, 4));
        settings.test_loop.heating_time.sec = (int32_t)sqlite3_column_double(stmt, 5);
        settings.test_loop.cycle_count = sqlite3_column_int(stmt, 6);
        settings.test_loop.heating_duration.sec = (int32_t)sqlite3_column_double(stmt, 7);
        settings.test_loop.enabled = (sqlite3_column_int(stmt, 8) != 0);
        settings.test_loop.auto_strategy = sqlite3_column_int(stmt, 9);

        // Ref Loop (索引 10 ~ 19)
        settings.ref_loop.hardware_loop_settings.start_current_a = sqlite3_column_int(stmt, 10);
        settings.ref_loop.hardware_loop_settings.max_current_a = sqlite3_column_int(stmt, 11);
        settings.ref_loop.hardware_loop_settings.current_change_range_percent = sqlite3_column_int(stmt, 12);
        settings.ref_loop.hardware_loop_settings.ct_ratio = sqlite3_column_int(stmt, 13);
        settings.ref_loop.start_date = iso_string_to_time(safe_column_text(stmt, 14));
        settings.ref_loop.heating_time.sec = (int32_t)sqlite3_column_double(stmt, 15);
        settings.ref_loop.cycle_count = sqlite3_column_int(stmt, 16);
        settings.ref_loop.heating_duration.sec = (int32_t)sqlite3_column_double(stmt, 17);
        settings.ref_loop.enabled = (sqlite3_column_int(stmt, 18) != 0);
        settings.ref_loop.auto_strategy = sqlite3_column_int(stmt, 19);

        // Cable ID (索引 20)
        settings.sample_cable.id = sqlite3_column_int(stmt, 20);

        found = true;
    }
    sqlite3_finalize(stmt);
    return found;
}

bool DatabaseManager::save_regulator_settings(uint8_t regulator_id, const ros2_interfaces::msg::RegulatorSettings& settings) {
    if (!db_) return false;
    const char* sql = "INSERT OR REPLACE INTO regulator_settings (regulator_id, over_current_a, over_voltage_v, voltage_up_speed_percent, voltage_down_speed_percent, over_voltage_protection_mode, update_time) VALUES (?, ?, ?, ?, ?, ?, CURRENT_TIMESTAMP);";
    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) return false;
    sqlite3_bind_int(stmt, 1, regulator_id);
    sqlite3_bind_int(stmt, 2, settings.over_current_a);
    sqlite3_bind_int(stmt, 3, settings.over_voltage_v);
    sqlite3_bind_int(stmt, 4, settings.voltage_up_speed_percent);
    sqlite3_bind_int(stmt, 5, settings.voltage_down_speed_percent);
    sqlite3_bind_int(stmt, 6, settings.over_voltage_protection_mode ? 1 : 0);
    bool success = (sqlite3_step(stmt) == SQLITE_DONE);
    sqlite3_finalize(stmt);
    return success;
}

bool DatabaseManager::get_regulator_settings(uint8_t regulator_id, ros2_interfaces::msg::RegulatorSettings& settings) {
    const char* sql = "SELECT over_current_a, over_voltage_v, voltage_up_speed_percent, voltage_down_speed_percent, over_voltage_protection_mode FROM regulator_settings WHERE regulator_id = ?;";
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

bool DatabaseManager::insert_data_record(
    const std::string& record_time_str,
    uint8_t circuit_id,
    bool auto_on,
    const ros2_interfaces::msg::CircuitStatus& circuit_status,
    const ros2_interfaces::msg::CircuitSettings& circuit_settings,
    const ros2_interfaces::msg::RegulatorStatus& reg1,
    const ros2_interfaces::msg::RegulatorStatus& reg2)
{
    if (!db_) return false;

    std::stringstream ss_sql;
    ss_sql << "INSERT INTO data_records ("
           << "record_time, circuit_id, auto_on, "
           << "regulator_1_breaker_closed, regulator_1_voltage, regulator_1_current, "
           << "test_loop_is_heat, test_loop_breaker_closed, test_loop_strategy, test_loop_current, ";
    for (int i = 1; i <= 16; ++i) ss_sql << "test_loop_temp" << std::setfill('0') << std::setw(2) << i << ", ";

    ss_sql << "regulator_2_breaker_closed, regulator_2_voltage, regulator_2_current, "
           << "ref_loop_is_heat, ref_loop_breaker_closed, ref_loop_strategy, ref_loop_current";
    for (int i = 1; i <= 16; ++i) ss_sql << ", ref_loop_temp" << std::setfill('0') << std::setw(2) << i;

    ss_sql << ") VALUES (";
    for (int i = 0; i < 49; ++i) {
        ss_sql << (i == 0 ? "?" : ", ?");
    }
    ss_sql << ");";

    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, ss_sql.str().c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        RCLCPP_ERROR(logger_, "Failed to prepare insert_data_record SQL: %s", sqlite3_errmsg(db_));
        return false;
    }

    int idx = 1;
    sqlite3_bind_text(stmt, idx++, record_time_str.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, idx++, circuit_id);
    sqlite3_bind_int(stmt, idx++, auto_on ? 1 : 0);

    sqlite3_bind_int(stmt, idx++, reg1.breaker_closed_switch_ack ? 1 : 0);
    sqlite3_bind_double(stmt, idx++, reg1.voltage_reading);
    sqlite3_bind_double(stmt, idx++, reg1.current_reading);

    sqlite3_bind_int(stmt, idx++, circuit_status.test_loop.is_heat ? 1 : 0);
    sqlite3_bind_int(stmt, idx++, circuit_status.test_loop.hardware_loop_status.breaker_closed_switch_ack ? 1 : 0);
    sqlite3_bind_int(stmt, idx++, circuit_settings.test_loop.auto_strategy);
    sqlite3_bind_double(stmt, idx++, circuit_status.test_loop.hardware_loop_status.current);

    for (int i = 0; i < 16; ++i) {
        sqlite3_bind_double(stmt, idx++, circuit_status.test_loop.hardware_loop_status.temperature_array[i]);
    }

    sqlite3_bind_int(stmt, idx++, reg2.breaker_closed_switch_ack ? 1 : 0);
    sqlite3_bind_double(stmt, idx++, reg2.voltage_reading);
    sqlite3_bind_double(stmt, idx++, reg2.current_reading);

    sqlite3_bind_int(stmt, idx++, circuit_status.ref_loop.is_heat ? 1 : 0);
    sqlite3_bind_int(stmt, idx++, circuit_status.ref_loop.hardware_loop_status.breaker_closed_switch_ack ? 1 : 0);
    sqlite3_bind_int(stmt, idx++, circuit_settings.ref_loop.auto_strategy);
    sqlite3_bind_double(stmt, idx++, circuit_status.ref_loop.hardware_loop_status.current);

    for (int i = 0; i < 16; ++i) {
        sqlite3_bind_double(stmt, idx++, circuit_status.ref_loop.hardware_loop_status.temperature_array[i]);
    }

    bool success = (sqlite3_step(stmt) == SQLITE_DONE);
    if (!success) {
        RCLCPP_ERROR(logger_, "Failed to execute data record insert: %s", sqlite3_errmsg(db_));
    }

    sqlite3_finalize(stmt);
    return success;
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
        int idx = 1;

        rec.record_time = safe_column_text(stmt, idx++);
        rec.circuit_id = sqlite3_column_int(stmt, idx++);
        rec.auto_on = sqlite3_column_int(stmt, idx++) != 0;

        rec.regulator_1_breaker_closed = sqlite3_column_int(stmt, idx++) != 0;
        rec.regulator_1_voltage = sqlite3_column_double(stmt, idx++);
        rec.regulator_1_current = sqlite3_column_double(stmt, idx++);

        rec.test_loop_is_heat = sqlite3_column_int(stmt, idx++) != 0;
        rec.test_loop_breaker_closed = sqlite3_column_int(stmt, idx++) != 0;
        rec.test_loop_strategy = sqlite3_column_int(stmt, idx++);
        rec.test_loop_current = sqlite3_column_double(stmt, idx++);
        for(int i=0; i<16; ++i) rec.test_loop_temp[i] = sqlite3_column_double(stmt, idx++);

        rec.regulator_2_breaker_closed = sqlite3_column_int(stmt, idx++) != 0;
        rec.regulator_2_voltage = sqlite3_column_double(stmt, idx++);
        rec.regulator_2_current = sqlite3_column_double(stmt, idx++);

        rec.ref_loop_is_heat = sqlite3_column_int(stmt, idx++) != 0;
        rec.ref_loop_breaker_closed = sqlite3_column_int(stmt, idx++) != 0;
        rec.ref_loop_strategy = sqlite3_column_int(stmt, idx++);
        rec.ref_loop_current = sqlite3_column_double(stmt, idx++);
        for(int i=0; i<16; ++i) rec.ref_loop_temp[i] = sqlite3_column_double(stmt, idx++);

        results.push_back(rec);
    }
    sqlite3_finalize(stmt);
    return results;
}

bool DatabaseManager::is_valid_column(const std::string& col_name)
{
    static const std::set<std::string> valid_columns = {
        "record_id", "record_time", "circuit_id", "auto_on",
        "regulator_1_breaker_closed", "regulator_1_voltage", "regulator_1_current",
        "test_loop_is_heat", "test_loop_breaker_closed", "test_loop_strategy", "test_loop_current",
        "regulator_2_breaker_closed", "regulator_2_voltage", "regulator_2_current",
        "ref_loop_is_heat", "ref_loop_breaker_closed", "ref_loop_strategy", "ref_loop_current"
    };

    if (valid_columns.count(col_name)) return true;

    if (col_name.find("test_loop_temp") == 0 && col_name.length() == 16) {
        return true;
    }
    if (col_name.find("ref_loop_temp") == 0 && col_name.length() == 15) {
        return true;
    }

    return false;
}

bool DatabaseManager::query_data_records(
    const std::vector<std::string>& column_names,
    const std::string& start_time,
    const std::string& end_time,
    int circuit_id_filter,
    std::vector<std::string>& result_header,
    std::vector<std::string>& result_rows)
{
    if (!db_) return false;

    std::stringstream sql_ss;
    sql_ss << "SELECT ";

    std::vector<std::string> safe_cols;

    if (column_names.empty()) {
        safe_cols.push_back("record_time");
        safe_cols.push_back("circuit_id");
    } else {
        for (const auto& col : column_names) {
            if (is_valid_column(col)) {
                safe_cols.push_back(col);
            } else {
                RCLCPP_WARN(logger_, "Ignored invalid or unknown column request: %s", col.c_str());
            }
        }
    }

    if (safe_cols.empty()) {
        RCLCPP_ERROR(logger_, "No valid columns provided for query.");
        return false;
    }

    for (size_t i = 0; i < safe_cols.size(); ++i) {
        sql_ss << safe_cols[i];
        if (i < safe_cols.size() - 1) sql_ss << ", ";
    }

    result_header = safe_cols;

    sql_ss << " FROM data_records WHERE record_time BETWEEN ? AND ?";

    if (circuit_id_filter > 0) {
        sql_ss << " AND circuit_id = " << circuit_id_filter;
    }

    sql_ss << " ORDER BY record_time ASC;";

    std::string sql_str = sql_ss.str();

    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql_str.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        RCLCPP_ERROR(logger_, "Failed to prepare query SQL: %s", sqlite3_errmsg(db_));
        update_connection_status(false);
        return false;
    }
    update_connection_status(true);

    sqlite3_bind_text(stmt, 1, start_time.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, end_time.c_str(), -1, SQLITE_TRANSIENT);

    result_rows.clear();
    int col_count = static_cast<int>(safe_cols.size());

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        std::stringstream row_ss;
        for (int i = 0; i < col_count; ++i) {
            const char* text = (const char*)sqlite3_column_text(stmt, i);
            if (text) {
                row_ss << text;
            } else {
                row_ss << "NULL";
            }

            if (i < col_count - 1) {
                row_ss << ",";
            }
        }
        result_rows.push_back(row_ss.str());
    }

    sqlite3_finalize(stmt);
    return true;
}
