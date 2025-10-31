#include "control_node/hardware_coordinator.hpp"
#include "control_node/state_manager.hpp" // For NUM_CIRCUITS and NUM_REGULATORS

HardwareCoordinator::HardwareCoordinator(StateManager* state_manager, std::shared_ptr<ControlNodeContext> context)
    : state_manager_(state_manager), context_(context)
{
    // 在真实项目中，硬件协调器会订阅来自HARDWARE_NODE的状态话题。
    // 这里我们用一个定时器来模拟“定期收到”或“主动轮询”硬件状态。
    // 注意：这里的临时节点是一种简化技巧，更好的方式是在构造时传入一个Node指针。
    auto temp_node_for_timer = rclcpp::Node::make_shared("_hc_timer_node");
    polling_timer_ = temp_node_for_timer->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&HardwareCoordinator::poll_hardware_status, this)
        );
    RCLCPP_INFO(context_->logging_interface->get_logger(), "Hardware Coordinator initialized and will poll status every 1s.");
}

void HardwareCoordinator::poll_hardware_status()
{
    RCLCPP_DEBUG(context_->logging_interface->get_logger(), "Polling hardware statuses for all components...");

    // === 1. 轮询并发布所有回路的状态 (Circuit Statuses) ===
    for (uint8_t id = 1; id <= StateManager::NUM_CIRCUITS; ++id)
    {
        // --- 这是一个模拟 ---
        // 在真实系统中，你会调用一个服务：
        // auto status = hardware_client_->get_circuit_status(id);
        // 或者从一个话题回调中接收数据。

        // 创建模拟的回路状态消息
        auto circuit_msg = ros2_interfaces::msg::CircuitStatus();
        circuit_msg.header.stamp = rclcpp::Clock().now();
        circuit_msg.header.frame_id = "circuit_frame_" + std::to_string(id);
        circuit_msg.circuit_id = id;

        // 赋予一些随ID变化的模拟值，方便调试时区分
        circuit_msg.test_current = 150.0 + (id * 0.5);
        circuit_msg.ref_current = 150.0;

        // 步骤 a: 将轮询到的最新状态更新到 StateManager
        state_manager_->update_circuit_status(id, circuit_msg);

        // 步骤 b: 将最新状态发布到ROS话题，供其他节点（如QT_NODE）使用
        if (context_->circuit_status_publisher) {
            context_->circuit_status_publisher->publish(circuit_msg);
        }
    }

    // === 2. 轮询并发布所有调压器的状态 (Regulator Statuses) ===
    for (uint8_t id = 1; id <= StateManager::NUM_REGULATORS; ++id)
    {
        // --- 同样，这是一个模拟 ---
        auto regulator_msg = ros2_interfaces::msg::VoltageRegulatorStatus();
        regulator_msg.header.stamp = rclcpp::Clock().now();
        regulator_msg.header.frame_id = "regulator_frame_" + std::to_string(id);
        regulator_msg.voltage_regulator_id = id;

        // 赋予一些随ID变化的模拟值
        regulator_msg.voltage_reading = 220.0 + (id * 0.5);
        regulator_msg.current_reading = 155.0 + id;

        // 步骤 a: 更新 StateManager
        state_manager_->update_regulator_status(id, regulator_msg);

        // 步骤 b: 发布到ROS话题
        if (context_->regulator_status_publisher) {
            context_->regulator_status_publisher->publish(regulator_msg);
        }
    }
}

// 设置 RegulatorSettings 的（硬件相关的）参数到硬件
bool HardwareCoordinator::apply_regulator_settings_to_hardware(uint8_t id, const ros2_interfaces::msg::VoltageRegulatorSettings& settings)
{
    // 这个方法封装了所有与 HARDWARE_NODE 通信的细节
    RCLCPP_INFO(context_->logging_interface->get_logger(), "Applying settings to hardware for regulator %u...", id);

    // 假设你有一个叫 "command_hardware" 的服务
    // auto client = context_->command_hardware_client;

    // 1. 检查客户端和服务是否可用 (与 PersistenceCoordinator 中的逻辑类似)
    // if (!client || !client->wait_for_service(1s)) {
    //     RCLCPP_ERROR(context_->logging_interface->get_logger(), "Hardware command service not available.");
    //     return false;
    // }

    // 2. 准备请求
    // auto request = std::make_shared<...>();
    // request->regulator_id = id;
    // 直接传递整个settings

    // 3. 发送请求并阻塞等待结果
    // auto future = client->async_send_request(request);
    // if (rclcpp::spin_until_future_complete(...) != SUCCESS) {
    //     RCLCPP_ERROR(context_->logging_interface->get_logger(), "Failed to call hardware command service.");
    //     return false;
    // }

    // 4. 返回硬件节点的执行结果
    // return future.get()->success;

    // --- 模拟返回 ---
    RCLCPP_INFO(context_->logging_interface->get_logger(), "Mock hardware update successful for regulator %u.", id);
    return true; // 假设总是成功
}

// 设置 CircuitSettings 的（硬件相关的）参数到硬件
bool HardwareCoordinator::apply_circuit_settings_to_hardware(uint8_t id, const ros2_interfaces::msg::CircuitSettings& settings)
{
    // 这个方法封装了所有与 HARDWARE_NODE 通信的细节
    RCLCPP_INFO(context_->logging_interface->get_logger(), "Applying settings to hardware for circuit %u...", id);

    // 假设你有一个叫 "command_hardware" 的服务
    // auto client = context_->command_hardware_client;

    // 1. 检查客户端和服务是否可用 (与 PersistenceCoordinator 中的逻辑类似)
    // if (!client || !client->wait_for_service(1s)) {
    //     RCLCPP_ERROR(context_->logging_interface->get_logger(), "Hardware command service not available.");
    //     return false;
    // }

    // 2. 准备请求
    // auto request = std::make_shared<...>();
    // request->circuit_id = id;
    /* 不要传递整个settings，取每个loop里面的3个字段（共6个）：最大电流、设定电流、电流允许误差
    settings.test_loop.max_current_a;
    settings.test_loop.start_current_a;
    settings.test_loop.current_change_range_percent;
    settings.ref_loop.max_current_a;
    settings.ref_loop.start_current_a;
    settings.ref_loop.current_change_range_percent;
    */

    // 3. 发送请求并阻塞等待结果
    // auto future = client->async_send_request(request);
    // if (rclcpp::spin_until_future_complete(...) != SUCCESS) {
    //     RCLCPP_ERROR(context_->logging_interface->get_logger(), "Failed to call hardware command service.");
    //     return false;
    // }

    // 4. 返回硬件节点的执行结果
    // return future.get()->success;

    // --- 模拟返回 ---
    RCLCPP_INFO(context_->logging_interface->get_logger(), "Mock hardware update successful for circuit %u.", id);
    return true; // 假设总是成功
}
