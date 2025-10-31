#include "record_node/database_manager.hpp"
#include <iomanip>
#include <sstream>
#include <ctime>

namespace{
std::string time_to_iso_string(const builtin_interfaces::msg::Time& time_msg)
{
    time_t seconds = time_msg.sec;
    struct tm tm_struct;
    gmtime_s(&tm_struct, &seconds);

    std::stringstream ss;
    ss << std::put_time(&tm_struct, "%Y-%m-%dT%H:%M:%S");
    ss << "." << std::setfill('0') << std::setw(9) << time_msg.nanosec << "Z";
    return ss.str();
}

// 将 builtin_interfaces::msg::Duration 转换为总秒数 (浮点数)
double duration_to_seconds(const builtin_interfaces::msg::Duration& duration_msg)
{
    return static_cast<double>(duration_msg.sec) + static_cast<double>(duration_msg.nanosec) / 1e9;
}
}

DatabaseManager::DatabaseManager(const std::string& db_path, rclcpp::Logger logger)
    : db_(nullptr), logger_(logger)
{
    if (sqlite3_open(db_path.c_str(), &db_) != SQLITE_OK) {
        RCLCPP_ERROR(logger_, "无法打开数据库: %s", sqlite3_errmsg(db_));
        db_ = nullptr;
    } else {
        RCLCPP_INFO(logger_, "数据库已成功打开: %s", db_path.c_str());
        initialize_database();
    }
}

DatabaseManager::~DatabaseManager()
{
    if (db_) {
        sqlite3_close(db_);
        RCLCPP_INFO(logger_, "数据库已关闭。");
    }
}

bool DatabaseManager::execute_sql(const char* sql, const char* context_msg)
{
    if (!db_) return false;
    int rc = sqlite3_exec(db_, sql, 0, 0, &db_err_msg_);
    if (rc != SQLITE_OK) {
        RCLCPP_ERROR(logger_, "%s 失败: %s", context_msg, db_err_msg_);
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
    if (!execute_sql(create_system_settings_sql, "创建 system_settings 表")) return;

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
    if (!execute_sql(create_regulator_settings_sql, "创建 regulator_settings 表")) return;

    // 3. 创建 circuit_settings 表
    const char* create_circuit_settings_sql =
        "CREATE TABLE IF NOT EXISTS circuit_settings ("
        "circuit_id INTEGER PRIMARY KEY,"
        "test_start_current_a INTEGER, test_max_current_a INTEGER, test_current_change_range_percent INTEGER, "
        "test_ct_ratio INTEGER, test_start_datetime TEXT, test_cycle_count INTEGER, test_heating_duration REAL, "
        "ref_start_current_a INTEGER, ref_max_current_a INTEGER, ref_current_change_range_percent INTEGER, "
        "ref_ct_ratio INTEGER, ref_start_datetime TEXT, ref_cycle_count INTEGER, ref_heating_duration REAL, "
        "cable_type TEXT, cable_spec TEXT, insulation_material TEXT, insulation_thickness REAL, "
        "update_time DATETIME DEFAULT CURRENT_TIMESTAMP);";
    if (!execute_sql(create_circuit_settings_sql, "创建 circuit_settings 表")) return;

    // 4. 为 system_settings 插入默认行，确保 id=1 始终存在
    const char* insert_default_system_settings_sql =
        "INSERT OR IGNORE INTO system_settings (id) VALUES (1);";
    if (!execute_sql(insert_default_system_settings_sql, "插入默认 system_settings 行")) return;

    RCLCPP_INFO(logger_, "数据库表已成功初始化。");
}

bool DatabaseManager::save_system_settings(const ros2_interfaces::msg::SystemSettings& settings)
{
    if (!db_) return false;

    const char* sql = "INSERT OR REPLACE INTO system_settings "
                      "(id, sample_interval_sec, record_interval_min, keep_record_on_shutdown, update_time) "
                      "VALUES (1, ?, ?, ?, CURRENT_TIMESTAMP);";

    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        RCLCPP_ERROR(logger_, "准备 system_settings SQL 语句失败: %s", sqlite3_errmsg(db_));
        return false;
    }

    sqlite3_bind_int(stmt, 1, settings.sample_interval_sec);
    sqlite3_bind_int(stmt, 2, settings.record_interval_min);
    sqlite3_bind_int(stmt, 3, settings.keep_record_on_shutdown ? 1 : 0);

    bool success = (sqlite3_step(stmt) == SQLITE_DONE);
    if (!success) {
        RCLCPP_ERROR(logger_, "执行 system_settings 保存失败: %s", sqlite3_errmsg(db_));
    } else {
        RCLCPP_INFO(logger_, "系统设置已成功保存到数据库。");
    }

    sqlite3_finalize(stmt);
    return success;
}

bool DatabaseManager::save_regulator_settings(uint8_t regulator_id, const ros2_interfaces::msg::VoltageRegulatorSettings& settings)
{
    if (!db_) return false;

    const char* sql = "INSERT OR REPLACE INTO regulator_settings "
                      "(regulator_id, over_current_a, over_voltage_v, voltage_up_speed_percent, "
                      "voltage_down_speed_percent, over_voltage_protection_mode, update_time) "
                      "VALUES (?, ?, ?, ?, ?, ?, CURRENT_TIMESTAMP);";

    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        RCLCPP_ERROR(logger_, "准备 regulator_settings SQL 语句失败: %s", sqlite3_errmsg(db_));
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
        RCLCPP_ERROR(logger_, "执行 regulator_settings 保存 (ID: %u) 失败: %s", regulator_id, sqlite3_errmsg(db_));
    } else {
        RCLCPP_INFO(logger_, "调压器 %u 的设置已成功保存到数据库。", regulator_id);
    }

    sqlite3_finalize(stmt);
    return success;
}

bool DatabaseManager::save_circuit_settings(uint8_t circuit_id, const ros2_interfaces::msg::CircuitSettings& settings)
{
    if (!db_) return false;

    // 更新 SQL 语句以匹配新的表结构
    const char* sql = "INSERT OR REPLACE INTO circuit_settings "
                      "(circuit_id, "
                      "test_start_current_a, test_max_current_a, test_current_change_range_percent, test_ct_ratio, test_start_datetime, test_cycle_count, test_heating_duration, "
                      "ref_start_current_a, ref_max_current_a, ref_current_change_range_percent, ref_ct_ratio, ref_start_datetime, ref_cycle_count, ref_heating_duration, "
                      "cable_type, cable_spec, insulation_material, insulation_thickness, "
                      "update_time) "
                      "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, CURRENT_TIMESTAMP);";

    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        RCLCPP_ERROR(logger_, "准备 circuit_settings SQL 语句失败: %s", sqlite3_errmsg(db_));
        return false;
    }

    // --- 数据转换 ---
    std::string test_start_iso = time_to_iso_string(settings.test_loop.start_datetime);
    double test_duration_sec = duration_to_seconds(settings.test_loop.heating_duration);
    std::string ref_start_iso = time_to_iso_string(settings.ref_loop.start_datetime);
    double ref_duration_sec = duration_to_seconds(settings.ref_loop.heating_duration);

    // --- 绑定所有19个参数 ---
    int idx = 1;
    sqlite3_bind_int(stmt, idx++, circuit_id);
    // Test Loop
    sqlite3_bind_int(stmt, idx++, settings.test_loop.start_current_a);
    sqlite3_bind_int(stmt, idx++, settings.test_loop.max_current_a);
    sqlite3_bind_int(stmt, idx++, settings.test_loop.current_change_range_percent);
    sqlite3_bind_int(stmt, idx++, settings.test_loop.ct_ratio);
    sqlite3_bind_text(stmt, idx++, test_start_iso.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, idx++, settings.test_loop.cycle_count);
    sqlite3_bind_double(stmt, idx++, test_duration_sec);
    // Ref Loop
    sqlite3_bind_int(stmt, idx++, settings.ref_loop.start_current_a);
    sqlite3_bind_int(stmt, idx++, settings.ref_loop.max_current_a);
    sqlite3_bind_int(stmt, idx++, settings.ref_loop.current_change_range_percent);
    sqlite3_bind_int(stmt, idx++, settings.ref_loop.ct_ratio);
    sqlite3_bind_text(stmt, idx++, ref_start_iso.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, idx++, settings.ref_loop.cycle_count);
    sqlite3_bind_double(stmt, idx++, ref_duration_sec);
    // Sample Params
    sqlite3_bind_text(stmt, idx++, settings.sample_params.cable_type.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, idx++, settings.sample_params.cable_spec.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, idx++, settings.sample_params.insulation_material.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_double(stmt, idx++, settings.sample_params.insulation_thickness);

    bool success = (sqlite3_step(stmt) == SQLITE_DONE);
    if (!success) {
        RCLCPP_ERROR(logger_, "执行 circuit_settings 保存 (ID: %u) 失败: %s", circuit_id, sqlite3_errmsg(db_));
    } else {
        RCLCPP_INFO(logger_, "回路 %u 的设置已成功保存到数据库。", circuit_id);
    }

    sqlite3_finalize(stmt);
    return success;
}
