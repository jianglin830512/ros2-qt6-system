#include "record_node/record_node.hpp"
#include "record_node/record_node_constants.hpp"

RecordNode::RecordNode() : Node("record_node")
{
    RCLCPP_INFO(this->get_logger(), "正在初始化 RecordNode...");

    // 1. 声明并获取参数
    this->declare_parameter<std::string>(
        record_node_constants::DB_PATH_PARAM,
        record_node_constants::DEFAULT_DB_PATH);
    this->declare_parameter<std::string>(
        record_node_constants::SAVE_SYSTEM_SETTINGS_SERVICE_PARAM,
        record_node_constants::DEFAULT_SAVE_SYSTEM_SETTINGS_SERVICE);
    this->declare_parameter<std::string>(
        record_node_constants::SAVE_REGULATOR_SETTINGS_SERVICE_PARAM,
        record_node_constants::DEFAULT_SAVE_REGULATOR_SETTINGS_SERVICE);
    this->declare_parameter<std::string>(
        record_node_constants::SAVE_CIRCUIT_SETTINGS_SERVICE_PARAM,
        record_node_constants::DEFAULT_SAVE_CIRCUIT_SETTINGS_SERVICE);

    auto db_path = this->get_parameter(record_node_constants::DB_PATH_PARAM).as_string();
    auto save_system_settings_service_name = this->get_parameter(record_node_constants::SAVE_SYSTEM_SETTINGS_SERVICE_PARAM).as_string();
    auto save_regulator_settings_service_name = this->get_parameter(record_node_constants::SAVE_REGULATOR_SETTINGS_SERVICE_PARAM).as_string();
    auto save_circuit_settings_service_name = this->get_parameter(record_node_constants::SAVE_CIRCUIT_SETTINGS_SERVICE_PARAM).as_string();

    RCLCPP_INFO(this->get_logger(), "数据库路径设置为: %s", db_path.c_str());

    // 2. 初始化 DatabaseManager
    db_manager_ = std::make_unique<DatabaseManager>(db_path, this->get_logger());

    // 3. 创建服务服务器
    save_system_settings_service_ = this->create_service<ros2_interfaces::srv::SetSystemSettings>(
        save_system_settings_service_name,
        std::bind(&RecordNode::save_system_settings_callback, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Create ROS2 Service Server: %s", save_system_settings_service_name.c_str());

    save_regulator_settings_service_ = this->create_service<ros2_interfaces::srv::SetRegulatorSettings>(
        save_regulator_settings_service_name,
        std::bind(&RecordNode::save_regulator_settings_callback, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Create ROS2 Service Server: %s", save_regulator_settings_service_name.c_str());

    save_circuit_settings_service_ = this->create_service<ros2_interfaces::srv::SetCircuitSettings>(
        save_circuit_settings_service_name,
        std::bind(&RecordNode::save_circuit_settings_callback, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Create ROS2 Service Server: %s", save_circuit_settings_service_name.c_str());

    RCLCPP_INFO(this->get_logger(), "RecordNode 初始化完成，服务已就绪。");
}

void RecordNode::save_system_settings_callback(
    const std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Request> request,
    std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "收到保存系统设置的请求。");
    bool success = db_manager_->save_system_settings(request->settings);
    response->success = success;
    response->message = success ? "系统设置成功保存到数据库。" : "保存系统设置到数据库失败。";
}

void RecordNode::save_regulator_settings_callback(
    const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request> request,
    std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "收到为ID %u 保存调压器设置的请求。", request->regulator_id);
    bool success = db_manager_->save_regulator_settings(request->regulator_id, request->settings);
    response->success = success;
    response->message = success ? "调压器设置成功保存到数据库。" : "保存调压器设置到数据库失败。";
}

void RecordNode::save_circuit_settings_callback(
    const std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Request> request,
    std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "收到为ID %u 保存回路设置的请求。", request->circuit_id);
    bool success = db_manager_->save_circuit_settings(request->circuit_id, request->settings);
    response->success = success;
    response->message = success ? "回路设置成功保存到数据库。" : "保存回路设置到数据库失败。";
}
