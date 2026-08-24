#ifndef GLOBAL_DATABASE_MANAGER_HPP
#define GLOBAL_DATABASE_MANAGER_HPP

#include <string>
#include <sqlite3.h>
#include <vector>
#include "rclcpp/rclcpp.hpp"  // IWYU pragma: keep
#include "ros2_interfaces/msg/cable.hpp"

class DatabaseManager
{
public:
    DatabaseManager(const std::string& db_path, rclcpp::Logger logger);
    ~DatabaseManager();

    bool is_connected() const { return database_connected_.load(std::memory_order_relaxed); }

    // --- 电缆管理 ---
    bool save_cable(const ros2_interfaces::msg::Cable& cable_data, std::string& error_msg);
    bool delete_cable(int32_t id, std::string& error_msg);

    bool list_cables(
        const std::string& search_keyword,
        int32_t page,
        int32_t page_size,
        uint8_t sort_column,
        bool is_ascending,
        std::vector<ros2_interfaces::msg::Cable>& result_cables,
        int32_t& total_pages,
        int32_t& current_page,
        std::string& error_msg);

    bool get_cable_info_batch(
        const std::vector<int32_t>& ids,
        std::vector<ros2_interfaces::msg::Cable>& result_cables,
        std::string& error_msg);

private:
    void initialize_database();
    bool execute_sql(const char* sql, const char* context_msg);
    void update_connection_status(bool is_ok) {
        database_connected_.store(is_ok, std::memory_order_relaxed);
    }
    std::string get_current_time_str();

    sqlite3* db_;
    rclcpp::Logger logger_;
    char* db_err_msg_ = nullptr;
    std::atomic<bool> database_connected_{false};
};

#endif // GLOBAL_DATABASE_MANAGER_HPP
