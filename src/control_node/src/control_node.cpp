#include "control_node/control_node.hpp"
#include "control_node/control_node_constants.hpp"
#include <chrono>
#include <functional>

// 使用 std::chrono_literals 来方便地使用时间单位，例如 1s
using namespace std::chrono_literals;

ControlNode::ControlNode() : Node("control_node")
{
    // === 1. 初始化参数 ===
    // 声明发布器参数
    this->declare_parameter<std::string>(
        control_node_constants::CIRCUIT_STATUS_TOPIC_PARAM,
        control_node_constants::DEFAULT_CIRCUIT_STATUS_TOPIC);
    this->declare_parameter<std::string>(
        control_node_constants::REGULATOR_STATUS_TOPIC_PARAM,
        control_node_constants::DEFAULT_REGULATOR_STATUS_TOPIC);

    // 声明订阅器参数
    this->declare_parameter<std::string>(
        control_node_constants::REGULATOR_COMMAND_TOPIC_PARAM,
        control_node_constants::DEFAULT_REGULATOR_COMMAND_TOPIC);
    this->declare_parameter<std::string>(
        control_node_constants::CIRCUIT_COMMAND_TOPIC_PARAM,
        control_node_constants::DEFAULT_CIRCUIT_COMMAND_TOPIC);
    this->declare_parameter<std::string>(
        control_node_constants::CLEAR_ALARM_TOPIC_PARAM,
        control_node_constants::DEFAULT_CLEAR_ALARM_TOPIC);

    // 声明服务服务器参数
    this->declare_parameter<std::string>(
        control_node_constants::SET_SYSTEM_SETTINGS_SERVICE_PARAM,
        control_node_constants::DEFAULT_SET_SYSTEM_SETTINGS_SERVICE);
    this->declare_parameter<std::string>(
        control_node_constants::SET_REGULATOR_SETTINGS_SERVICE_PARAM,
        control_node_constants::DEFAULT_SET_REGULATOR_SETTINGS_SERVICE);
    this->declare_parameter<std::string>(
        control_node_constants::SET_CIRCUIT_SETTINGS_SERVICE_PARAM,
        control_node_constants::DEFAULT_SET_CIRCUIT_SETTINGS_SERVICE);

    // 获取发布器参数
    this->get_parameter(control_node_constants::CIRCUIT_STATUS_TOPIC_PARAM, circuit_status_topic_);
    this->get_parameter(control_node_constants::REGULATOR_STATUS_TOPIC_PARAM, regulator_status_topic_);
    RCLCPP_INFO(this->get_logger(), "Publishing CircuitStatus to: '%s'", circuit_status_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing VoltageRegulatorStatus to: '%s'", regulator_status_topic_.c_str());

    // 获取订阅器参数
    this->get_parameter(control_node_constants::REGULATOR_COMMAND_TOPIC_PARAM, regulator_command_topic_);
    this->get_parameter(control_node_constants::CIRCUIT_COMMAND_TOPIC_PARAM, circuit_command_topic_);
    this->get_parameter(control_node_constants::CLEAR_ALARM_TOPIC_PARAM, clear_alarm_topic_);
    RCLCPP_INFO(this->get_logger(), "Subscribing to RegulatorCommand on: '%s'", regulator_command_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribing to CircuitCommand on: '%s'", circuit_command_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribing to ClearAlarm on: '%s'", clear_alarm_topic_.c_str());

    // 取服务服务器参数
    this->get_parameter(control_node_constants::SET_SYSTEM_SETTINGS_SERVICE_PARAM, set_system_settings_service_name_);
    this->get_parameter(control_node_constants::SET_REGULATOR_SETTINGS_SERVICE_PARAM, set_regulator_settings_service_name_);
    this->get_parameter(control_node_constants::SET_CIRCUIT_SETTINGS_SERVICE_PARAM, set_circuit_settings_service_name_);
    RCLCPP_INFO(this->get_logger(), "Advertising service: '%s'", set_system_settings_service_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Advertising service: '%s'", set_regulator_settings_service_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Advertising service: '%s'", set_circuit_settings_service_name_.c_str());

    // === 2. 初始化发布器和定时器 ===
    circuit_status_pub_ = this->create_publisher<ros2_interfaces::msg::CircuitStatus>(circuit_status_topic_, 10);
    regulator_status_pub_ = this->create_publisher<ros2_interfaces::msg::VoltageRegulatorStatus>(regulator_status_topic_, 10);
    timer_ = this->create_wall_timer(1s, std::bind(&ControlNode::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "发布器和定时器初始化完成.");

    // === 3. 初始化订阅器 ===
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)); // 定义QoS策略
    regulator_command_sub_ = this->create_subscription<ros2_interfaces::msg::VoltageRegulatorCommand>(
        regulator_command_topic_, qos, std::bind(&ControlNode::regulator_command_callback, this, std::placeholders::_1));
    circuit_command_sub_ = this->create_subscription<ros2_interfaces::msg::CircuitCommand>(
        circuit_command_topic_, qos, std::bind(&ControlNode::circuit_command_callback, this, std::placeholders::_1));
    clear_alarm_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        clear_alarm_topic_, qos, std::bind(&ControlNode::clear_alarm_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "订阅器初始化完成.");

    // === 4. 初始化服务服务器 ===
    set_system_settings_service_ = this->create_service<ros2_interfaces::srv::SetSystemSettings>(
        set_system_settings_service_name_,
        std::bind(&ControlNode::set_system_settings_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_regulator_settings_service_ = this->create_service<ros2_interfaces::srv::SetRegulatorSettings>(
        set_regulator_settings_service_name_,
        std::bind(&ControlNode::set_regulator_settings_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_circuit_settings_service_ = this->create_service<ros2_interfaces::srv::SetCircuitSettings>(
        set_circuit_settings_service_name_,
        std::bind(&ControlNode::set_circuit_settings_callback, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "服务服务器初始化完成.");
}

ControlNode::~ControlNode()
{
    RCLCPP_INFO(this->get_logger(), "控制节点已销毁.");
}

// =======================================================================
//               定时器回调函数的实现
// =======================================================================

// 轮询并发布TOPIC
void ControlNode::timer_callback()
{
    // --- 1. 创建并填充 CircuitStatus 消息 ---
    auto circuit_msg = ros2_interfaces::msg::CircuitStatus();

    // 设置消息头
    circuit_msg.header.stamp = this->get_clock()->now();
    circuit_msg.header.frame_id = "circuit_frame_1";

    // 填充回路数据（此处使用示例值）
    circuit_msg.circuit_id = 1;
    circuit_msg.test_current = 150.5;
    circuit_msg.ref_current = 150.0;

    // 填充温度数组
    for (size_t i = 0; i < 16; ++i) {
        circuit_msg.test_temperature_array[i] = 45.5 + i * 0.1;
    }
    for (size_t i = 0; i < 8; ++i) {
        circuit_msg.ref_temperature_array[i] = 45.0 + i * 0.05;
    }

    circuit_msg.elapsed_heating_time.sec = 600; // 10分钟
    circuit_msg.remaining_heating_time.sec = 1200; // 20分钟
    circuit_msg.completed_cycle_count = 5;
    circuit_msg.remaining_cycle_count = 15;

    // 填充状态
    circuit_msg.control_mode = ros2_interfaces::msg::CircuitStatus::CURR_MODE; // 恒流模式
    circuit_msg.circuit_enabled = true;
    circuit_msg.breaker_closed_switch_ack = true;
    circuit_msg.breaker_opened_switch_ack = false;


    // --- 2. 创建并填充 VoltageRegulatorStatus 消息 ---
    auto regulator_msg = ros2_interfaces::msg::VoltageRegulatorStatus();

    // 设置消息头
    regulator_msg.header.stamp = this->get_clock()->now();
    regulator_msg.header.frame_id = "regulator_frame_1";

    // 填充调压器数据（此处使用示例值）
    regulator_msg.voltage_regulator_id = 1;
    regulator_msg.voltage_reading = 220.5;
    regulator_msg.current_reading = 155.2;

    // 填充状态
    regulator_msg.voltage_direction = ros2_interfaces::msg::VoltageRegulatorStatus::VOLTAGE_STATIC; // 静止
    regulator_msg.breaker_closed_switch_ack = true;
    regulator_msg.breaker_opened_switch_ack = false;
    regulator_msg.upper_limit_switch_on = false;
    regulator_msg.lower_limit_switch_on = false;
    regulator_msg.over_current_on = false;
    regulator_msg.over_voltage_on = false;

    // --- 3. 发布两条消息 ---
    circuit_status_pub_->publish(circuit_msg);
    regulator_status_pub_->publish(regulator_msg);

    // 打印日志信息，确认消息已发布
    RCLCPP_DEBUG(this->get_logger(), "正在发布 CircuitStatus (ID: %d) 和 VoltageRegulatorStatus (ID: %d)",
                circuit_msg.circuit_id, regulator_msg.voltage_regulator_id);
}

// =======================================================================
//               命令回调函数的实现
// =======================================================================

void ControlNode::regulator_command_callback(const ros2_interfaces::msg::VoltageRegulatorCommand::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "接收到调压器命令 -> ID: %u, 命令: %u", msg->regulator_id, msg->command);

    // 在这里，您可以根据 msg->command 的值来执行具体操作
    switch(msg->command) {
    case ros2_interfaces::msg::VoltageRegulatorCommand::CMD_BREAKER_CLOSE:
        RCLCPP_INFO(this->get_logger(), "执行: 调压器 %u 合闸", msg->regulator_id);
        break;
    case ros2_interfaces::msg::VoltageRegulatorCommand::CMD_BREAKER_OPEN:
        RCLCPP_INFO(this->get_logger(), "执行: 调压器 %u 分闸", msg->regulator_id);
        break;
    case ros2_interfaces::msg::VoltageRegulatorCommand::CMD_VOLTAGE_UP:
        RCLCPP_INFO(this->get_logger(), "执行: 调压器 %u 升压", msg->regulator_id);
        break;
    case ros2_interfaces::msg::VoltageRegulatorCommand::CMD_VOLTAGE_DOWN:
        RCLCPP_INFO(this->get_logger(), "执行: 调压器 %u 降压", msg->regulator_id);
        break;
    default:
        RCLCPP_WARN(this->get_logger(), "收到未知的调压器命令: %u", msg->command);
        break;
    }
}

void ControlNode::circuit_command_callback(const ros2_interfaces::msg::CircuitCommand::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "接收到回路命令 -> ID: %u, 命令: %u", msg->circuit_id, msg->command);

    // 在这里，您可以根据 msg->command 的值来执行具体操作
    switch(msg->command) {
    case ros2_interfaces::msg::CircuitCommand::CMD_TEST_BREAKER_CLOSE:
        RCLCPP_INFO(this->get_logger(), "执行: 回路 %u 试验回路合闸", msg->circuit_id);
        break;
    case ros2_interfaces::msg::CircuitCommand::CMD_TEST_BREAKER_OPEN:
        RCLCPP_INFO(this->get_logger(), "执行: 回路 %u 试验回路分闸", msg->circuit_id);
        break;
    case ros2_interfaces::msg::CircuitCommand::CMD_SIM_BREAKER_CLOSE:
        RCLCPP_INFO(this->get_logger(), "执行: 回路 %u 模拟回路合闸", msg->circuit_id);
        break;
    case ros2_interfaces::msg::CircuitCommand::CMD_SIM_BREAKER_OPEN:
        RCLCPP_INFO(this->get_logger(), "执行: 回路 %u 模拟回路分闸", msg->circuit_id);
        break;
    case ros2_interfaces::msg::CircuitCommand::CMD_SET_MANUAL_MODE:
        RCLCPP_INFO(this->get_logger(), "执行: 回路 %u 切换为手动模式", msg->circuit_id);
        break;
    case ros2_interfaces::msg::CircuitCommand::CMD_SET_CONST_CURRENT_MODE:
        RCLCPP_INFO(this->get_logger(), "执行: 回路 %u 切换为恒流模式", msg->circuit_id);
        break;
    case ros2_interfaces::msg::CircuitCommand::CMD_SET_TEMP_CONTROL_MODE:
        RCLCPP_INFO(this->get_logger(), "执行: 回路 %u 切换为温控模式", msg->circuit_id);
        break;
    default:
        RCLCPP_WARN(this->get_logger(), "收到未知的回路命令: %u", msg->command);
        break;
    }
}

void ControlNode::clear_alarm_callback(const std_msgs::msg::Empty::SharedPtr)
{
    RCLCPP_INFO(this->get_logger(), "执行: 消除报警提示");
}

// =======================================================================
//               【新增】服务回调函数的实现
// =======================================================================

void ControlNode::set_system_settings_callback(
    const std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Request> request,
    std::shared_ptr<ros2_interfaces::srv::SetSystemSettings::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "接收到 SetSystemSettings 服务请求:");
    RCLCPP_INFO(this->get_logger(), "  - 采样间隔: %d s", request->settings.sample_interval_sec);
    RCLCPP_INFO(this->get_logger(), "  - 记录间隔: %d min", request->settings.record_interval_min);
    RCLCPP_INFO(this->get_logger(), "  - 关机时保留记录: %s", request->settings.keep_record_on_shutdown ? "是" : "否");

    // --- 在此执行实际的业务逻辑 ---
    // 例如：将设置保存到文件或更新内部状态变量
    // 这里我们仅模拟成功
    bool success = true;
    std::string message = "系统参数已成功应用。";

    // --- 填充响应 ---
    response->success = success;
    response->message = message;
}

void ControlNode::set_regulator_settings_callback(
    const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request> request,
    std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "接收到 SetRegulatorSettings 服务请求 (ID: %u):", request->regulator_id);
    RCLCPP_INFO(this->get_logger(), "  - 过流保护: %d A", request->settings.over_current_a);
    RCLCPP_INFO(this->get_logger(), "  - 过压保护: %d V", request->settings.over_voltage_v);

    // --- 在此执行实际的业务逻辑 ---
    // 例如：通过串口或CAN总线将这些参数发送到硬件调压器控制器
    // 这里我们仅模拟成功
    bool success = true;
    std::string message = "调压器 " + std::to_string(request->regulator_id) + " 的参数已成功应用。";

    // --- 填充响应 ---
    response->success = success;
    response->message = message;
}

void ControlNode::set_circuit_settings_callback(
    const std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Request> request,
    std::shared_ptr<ros2_interfaces::srv::SetCircuitSettings::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "接收到 SetCircuitSettings 服务请求 (ID: %u):", request->circuit_id);
    RCLCPP_INFO(this->get_logger(), "  - 试验回路起始电流: %d A", request->settings.test_loop.start_current_a);
    RCLCPP_INFO(this->get_logger(), "  - 试品电缆类型: %s", request->settings.sample_params.cable_type.c_str());

    // --- 在此执行实际的业务逻辑 ---
    // 例如：更新试验流程控制器的状态机参数
    // 这里我们仅模拟成功
    bool success = true;
    std::string message = "回路 " + std::to_string(request->circuit_id) + " 的参数已成功应用。";

    // --- 填充响应 ---
    response->success = success;
    response->message = message;
}
