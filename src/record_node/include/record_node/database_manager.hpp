#ifndef DATABASE_MANAGER_HPP
#define DATABASE_MANAGER_HPP

#include <string>
#include <sqlite3.h>
#include "rclcpp/rclcpp.hpp"  // IWYU pragma: keep
#include "ros2_interfaces/msg/system_settings.hpp"
#include "ros2_interfaces/msg/regulator_settings.hpp"
#include "ros2_interfaces/msg/circuit_settings.hpp"
#include "ros2_interfaces/msg/circuit_status.hpp"
#include "ros2_interfaces/msg/regulator_status.hpp"
#include "ros2_interfaces/msg/data_record.hpp"
#include <vector>

class DatabaseManager
{
public:
    DatabaseManager(const std::string& db_path, rclcpp::Logger logger);
    ~DatabaseManager();

    bool is_connected() const { return database_connected_.load(std::memory_order_relaxed); }

    bool save_system_settings(const ros2_interfaces::msg::SystemSettings& settings);
    bool save_regulator_settings(uint8_t regulator_id, const ros2_interfaces::msg::RegulatorSettings& settings);
    bool save_circuit_settings(uint8_t circuit_id, const ros2_interfaces::msg::CircuitSettings& settings);

    /**
     * @brief 插入一条运行数据记录 (更新为新的数据结构)
     */
    bool insert_data_record(
        const std::string& record_time_str,
        uint8_t circuit_id,
        bool auto_on,
        const ros2_interfaces::msg::CircuitStatus& circuit_status,
        const ros2_interfaces::msg::CircuitSettings& circuit_settings,
        const ros2_interfaces::msg::RegulatorStatus& reg1_status,
        const ros2_interfaces::msg::RegulatorStatus& reg2_status);

    bool get_system_settings(ros2_interfaces::msg::SystemSettings& settings);
    bool get_regulator_settings(uint8_t regulator_id, ros2_interfaces::msg::RegulatorSettings& settings);
    bool get_circuit_settings(uint8_t circuit_id, ros2_interfaces::msg::CircuitSettings& settings);

    std::vector<ros2_interfaces::msg::DataRecord> get_data_records(
        const std::string& start_time, const std::string& end_time);

    bool query_data_records(
        const std::vector<std::string>& column_names,
        const std::string& start_time,
        const std::string& end_time,
        int circuit_id_filter,
        std::vector<std::string>& result_header,
        std::vector<std::string>& result_rows
        );

private:
    void initialize_database();
    void ensure_default_settings();
    bool execute_sql(const char* sql, const char* context_msg);
    bool is_valid_column(const std::string& col_name);
    void update_connection_status(bool is_ok) {
        database_connected_.store(is_ok, std::memory_order_relaxed);
    }

    sqlite3* db_;
    rclcpp::Logger logger_;
    char* db_err_msg_ = nullptr;
    std::atomic<bool> database_connected_{false};
};

#endif // DATABASE_MANAGER_HPP
