#ifndef DATABASE_MANAGER_HPP
#define DATABASE_MANAGER_HPP

#include <string>
#include <sqlite3.h>
#include "rclcpp/rclcpp.hpp"
#include "ros2_interfaces/msg/system_settings.hpp"
#include "ros2_interfaces/msg/voltage_regulator_settings.hpp"
#include "ros2_interfaces/msg/circuit_settings.hpp"

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
    bool save_regulator_settings(uint8_t regulator_id, const ros2_interfaces::msg::VoltageRegulatorSettings& settings);

    /**
     * @brief 保存或更新回路设置 (UPSERT)
     * @param circuit_id 回路 ID
     * @param settings 要保存的回路设置
     * @return 如果成功则返回 true，否则返回 false
     */
    bool save_circuit_settings(uint8_t circuit_id, const ros2_interfaces::msg::CircuitSettings& settings);


private:
    /**
     * @brief 初始化数据库，创建所有需要的表，并插入初始数据
     */
    void initialize_database();

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
