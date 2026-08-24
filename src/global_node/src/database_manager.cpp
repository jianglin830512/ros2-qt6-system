#include "global_node/database_manager.hpp"
#include <iomanip>
#include <sstream>
#include <ctime>
#include <cmath>
#include <map>

namespace {
std::string safe_column_text(sqlite3_stmt* stmt, int col_idx) {
    const char* text = (const char*)sqlite3_column_text(stmt, col_idx);
    return text ? std::string(text) : std::string("");
}
}

DatabaseManager::DatabaseManager(const std::string& db_path, rclcpp::Logger logger)
    : db_(nullptr), logger_(logger)
{
    if (sqlite3_open(db_path.c_str(), &db_) != SQLITE_OK) {
        RCLCPP_ERROR(logger_, "Could not open global database: %s", sqlite3_errmsg(db_));
        db_ = nullptr;
        update_connection_status(false);
    } else {
        RCLCPP_INFO(logger_, "Global Database opened successfully: %s", db_path.c_str());
        update_connection_status(true);
        initialize_database();
    }
}

DatabaseManager::~DatabaseManager()
{
    if (db_) {
        sqlite3_close(db_);
        RCLCPP_INFO(logger_, "Global Database closed.");
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

std::string DatabaseManager::get_current_time_str()
{
    auto now_sys = std::chrono::system_clock::now();
    time_t t = std::chrono::system_clock::to_time_t(now_sys);
    struct tm tm_struct;
#ifdef _MSC_VER
    localtime_s(&tm_struct, &t);
#else
    localtime_r(&t, &tm_struct);
#endif
    std::stringstream ss;
    ss << std::put_time(&tm_struct, "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

void DatabaseManager::initialize_database()
{
    // 创建电缆表 Cables，新增 voltage_grade(电压等级) 和 system_format(系统制式) 字段
    const char* create_cables_sql =
        "CREATE TABLE IF NOT EXISTS cables ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "name TEXT NOT NULL UNIQUE,"
        "core_diameter REAL,"
        "core_material TEXT,"
        "insulation_thickness REAL,"
        "insulation_material TEXT,"
        "voltage_grade INTEGER,"
        "system_format INTEGER,"
        "notes TEXT,"
        "last_modified DATETIME DEFAULT CURRENT_TIMESTAMP);";
    if (!execute_sql(create_cables_sql, "create cables table")) return;

    RCLCPP_INFO(logger_, "Global Database tables initialized successfully.");
}

bool DatabaseManager::save_cable(const ros2_interfaces::msg::Cable& cable_data, std::string& error_msg)
{
    if (!db_) { error_msg = "Database not connected."; return false; }

    // 1. 检查重名 (名称必须唯一)
    std::string check_sql = "SELECT id FROM cables WHERE name = ? AND id != ?";
    sqlite3_stmt* check_stmt;
    if (sqlite3_prepare_v2(db_, check_sql.c_str(), -1, &check_stmt, nullptr) == SQLITE_OK) {
        sqlite3_bind_text(check_stmt, 1, cable_data.name.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_int(check_stmt, 2, cable_data.id);
        if (sqlite3_step(check_stmt) == SQLITE_ROW) {
            error_msg = "名称已存在";
            sqlite3_finalize(check_stmt);
            return false;
        }
        sqlite3_finalize(check_stmt);
    }

    std::string curr_time = get_current_time_str();
    bool is_new = (cable_data.id < 0);
    std::string sql;

    // 2. 插入或更新，包含新增字段
    if (is_new) {
        sql = "INSERT INTO cables (name, core_diameter, core_material, insulation_thickness, insulation_material, voltage_grade, system_format, notes, last_modified) "
              "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?);";
    } else {
        sql = "UPDATE cables SET name = ?, core_diameter = ?, core_material = ?, insulation_thickness = ?, insulation_material = ?, voltage_grade = ?, system_format = ?, notes = ?, last_modified = ? "
              "WHERE id = ?;";
    }

    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        error_msg = std::string("Prepare SQL failed: ") + sqlite3_errmsg(db_);
        return false;
    }

    sqlite3_bind_text(stmt, 1, cable_data.name.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_double(stmt, 2, cable_data.core_diameter);
    sqlite3_bind_text(stmt, 3, cable_data.core_material.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_double(stmt, 4, cable_data.insulation_thickness);
    sqlite3_bind_text(stmt, 5, cable_data.insulation_material.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 6, cable_data.voltage_grade);
    sqlite3_bind_int(stmt, 7, cable_data.system_format);
    sqlite3_bind_text(stmt, 8, cable_data.notes.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 9, curr_time.c_str(), -1, SQLITE_TRANSIENT);

    if (!is_new) {
        sqlite3_bind_int(stmt, 10, cable_data.id);
    }

    bool success = (sqlite3_step(stmt) == SQLITE_DONE);
    if (!success) {
        error_msg = std::string("Execute SQL failed: ") + sqlite3_errmsg(db_);
    } else {
        error_msg = "Success";
    }
    sqlite3_finalize(stmt);
    return success;
}

bool DatabaseManager::delete_cable(int32_t id, std::string& error_msg)
{
    if (!db_) { error_msg = "Database not connected."; return false; }

    const char* sql = "DELETE FROM cables WHERE id = ?;";
    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        error_msg = "Prepare SQL failed.";
        return false;
    }

    sqlite3_bind_int(stmt, 1, id);
    bool success = (sqlite3_step(stmt) == SQLITE_DONE);
    int changes = sqlite3_changes(db_);

    if (success && changes == 0) {
        error_msg = "ID 不存在";
        success = false;
    } else if (!success) {
        error_msg = std::string("Execute SQL failed: ") + sqlite3_errmsg(db_);
    } else {
        error_msg = "Success";
    }

    sqlite3_finalize(stmt);
    return success;
}

bool DatabaseManager::list_cables(
    const std::string& search_keyword, int32_t page, int32_t page_size, uint8_t sort_column, bool is_ascending,
    std::vector<ros2_interfaces::msg::Cable>& result_cables,
    int32_t& total_pages, int32_t& current_page, std::string& error_msg)
{
    if (!db_) { error_msg = "Database not connected."; return false; }
    if (page < 1) page = 1;
    if (page_size < 1) page_size = 10;

    std::string like_pattern = "%" + search_keyword + "%";

    // 1. 获取总数
    std::string count_sql = "SELECT COUNT(*) FROM cables WHERE name LIKE ?;";
    sqlite3_stmt* count_stmt;
    int total_records = 0;
    if (sqlite3_prepare_v2(db_, count_sql.c_str(), -1, &count_stmt, nullptr) == SQLITE_OK) {
        sqlite3_bind_text(count_stmt, 1, like_pattern.c_str(), -1, SQLITE_TRANSIENT);
        if (sqlite3_step(count_stmt) == SQLITE_ROW) {
            total_records = sqlite3_column_int(count_stmt, 0);
        }
        sqlite3_finalize(count_stmt);
    }

    total_pages = std::ceil(static_cast<double>(total_records) / page_size);
    if (total_pages == 0) total_pages = 1;
    if (page > total_pages) page = total_pages;
    current_page = page;

    int offset = (page - 1) * page_size;

    // 2. 查询数据 (增加提取新字段)
    std::string query_sql = "SELECT id, name, core_diameter, core_material, insulation_thickness, "
                            "insulation_material, voltage_grade, system_format, notes, last_modified "
                            "FROM cables WHERE name LIKE ? ";

    // 动态拼接安全的 ORDER BY
    std::string order_by = "ORDER BY ";
    switch (sort_column) {
    case 1: order_by += "name "; break;
    case 2: order_by += "core_diameter "; break;
    case 3: order_by += "insulation_thickness "; break;
    case 4: order_by += "voltage_grade "; break;
    case 5: order_by += "system_format "; break;
    case 0:
    default: order_by += "last_modified "; break;
    }

    // 拼接升序或降序
    order_by += (is_ascending ? "ASC " : "DESC ");

    query_sql += order_by + "LIMIT ? OFFSET ?;";

    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, query_sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        error_msg = "Prepare query SQL failed.";
        return false;
    }

    sqlite3_bind_text(stmt, 1, like_pattern.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 2, page_size);
    sqlite3_bind_int(stmt, 3, offset);

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        ros2_interfaces::msg::Cable c;
        c.id = sqlite3_column_int(stmt, 0);
        c.name = safe_column_text(stmt, 1);
        c.core_diameter = sqlite3_column_double(stmt, 2);
        c.core_material = safe_column_text(stmt, 3);
        c.insulation_thickness = sqlite3_column_double(stmt, 4);
        c.insulation_material = safe_column_text(stmt, 5);
        c.voltage_grade = sqlite3_column_int(stmt, 6);
        c.system_format = static_cast<int8_t>(sqlite3_column_int(stmt, 7));
        c.notes = safe_column_text(stmt, 8);
        c.last_modified = safe_column_text(stmt, 9);
        result_cables.push_back(c);
    }

    sqlite3_finalize(stmt);
    error_msg = "Success";
    return true;
}

bool DatabaseManager::get_cable_info_batch(
    const std::vector<int32_t>& ids,
    std::vector<ros2_interfaces::msg::Cable>& result_cables,
    std::string& error_msg)
{
    if (!db_) { error_msg = "Database not connected."; return false; }
    if (ids.empty()) { error_msg = "IDs list is empty."; return true; }

    // 拼接 IN 子句: id IN (?, ?, ...)
    std::stringstream ss;
    ss << "SELECT id, name, core_diameter, core_material, insulation_thickness, "
       << "insulation_material, voltage_grade, system_format, notes, last_modified "
       << "FROM cables WHERE id IN (";
    for (size_t i = 0; i < ids.size(); ++i) {
        ss << (i == 0 ? "?" : ", ?");
    }
    ss << ");";

    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, ss.str().c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        error_msg = "Prepare batch SQL failed.";
        return false;
    }

    for (size_t i = 0; i < ids.size(); ++i) {
        sqlite3_bind_int(stmt, i + 1, ids[i]);
    }

    // 暂时用 map 存储，以保证可以按请求顺序返回
    std::map<int32_t, ros2_interfaces::msg::Cable> temp_map;

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        ros2_interfaces::msg::Cable c;
        c.id = sqlite3_column_int(stmt, 0);
        c.name = safe_column_text(stmt, 1);
        c.core_diameter = sqlite3_column_double(stmt, 2);
        c.core_material = safe_column_text(stmt, 3);
        c.insulation_thickness = sqlite3_column_double(stmt, 4);
        c.insulation_material = safe_column_text(stmt, 5);
        c.voltage_grade = sqlite3_column_int(stmt, 6);
        c.system_format = static_cast<int8_t>(sqlite3_column_int(stmt, 7));
        c.notes = safe_column_text(stmt, 8);
        c.last_modified = safe_column_text(stmt, 9);
        temp_map[c.id] = c;
    }
    sqlite3_finalize(stmt);

    // 按照请求的 id 顺序组装返回结果（如果某些ID不存在，会自动忽略跳过）
    for (int32_t req_id : ids) {
        if (temp_map.find(req_id) != temp_map.end()) {
            result_cables.push_back(temp_map[req_id]);
        }
    }

    error_msg = "Success";
    return true;
}
