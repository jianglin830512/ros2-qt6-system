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
    /**
     * @brief 构造函数，打开数据库并初始化表
     * @param db_path 数据库文件的路径
     * @param logger ROS2 日志记录器，用于输出信息和错误
     */
    DatabaseManager(const std::string& db_path, rclcpp::Logger logger);

    /**
     * @brief 析构函数，关闭数据库连接
     */
    ~DatabaseManager();

    /**
     * @brief 保存或更新系统设置 (UPSERT)
     * @param settings 要保存的系统设置
     * @return 如果成功则返回 true，否则返回 false
     */
    bool save_system_settings(const ros2_interfaces::msg::SystemSettings& settings);

    /**
     * @brief 保存或更新调压器设置 (UPSERT)
     * @param regulator_id 调压器 ID
     * @param settings 要保存的调压器设置
     * @return 如果成功则返回 true，否则返回 false
     */
    bool save_regulator_settings(uint8_t regulator_id, const ros2_interfaces::msg::RegulatorSettings& settings);

    /**
     * @brief 保存或更新回路设置 (UPSERT)
     * @param circuit_id 回路 ID
     * @param settings 要保存的回路设置
     * @return 如果成功则返回 true，否则返回 false
     */
    bool save_circuit_settings(uint8_t circuit_id, const ros2_interfaces::msg::CircuitSettings& settings);

    /**
     * @brief 插入一条运行数据记录
     * @param record_time_str 整分钟的时间字符串 (例如 "2023-10-27 09:00:00")
     * @param circuit_status 回路状态消息
     * @param regulator_status 调压器状态消息
     * @return 成功返回 true
     */
    bool insert_data_record(
        const std::string& record_time_str,
        const ros2_interfaces::msg::CircuitStatus& circuit_status,
        const ros2_interfaces::msg::RegulatorStatus& regulator_status);

    bool get_system_settings(ros2_interfaces::msg::SystemSettings& settings);
    bool get_regulator_settings(uint8_t regulator_id, ros2_interfaces::msg::RegulatorSettings& settings);
    bool get_circuit_settings(uint8_t circuit_id, ros2_interfaces::msg::CircuitSettings& settings);

    std::vector<ros2_interfaces::msg::DataRecord> get_data_records(
        const std::string& start_time, const std::string& end_time);

private:
    /**
     * @brief 初始化数据库，创建所有需要的表，并插入初始数据
     */
    void initialize_database();

    /**
     * @brief 确保必要的默认设置存在 (System ID 1, Regulator ID 1/2, Circuit ID 1/2)
     * 如果记录不存在，则插入默认值。
     */
    void ensure_default_settings();

    /**
     * @brief 执行简单的SQL语句并记录错误
     * @param sql 要执行的SQL语句
     * @param context_msg 发生错误时的上下文消息
     * @return 如果成功则返回 true
     */
    bool execute_sql(const char* sql, const char* context_msg);


    sqlite3* db_;
    rclcpp::Logger logger_;
    char* db_err_msg_ = nullptr;
};

#endif // DATABASE_MANAGER_HPP
