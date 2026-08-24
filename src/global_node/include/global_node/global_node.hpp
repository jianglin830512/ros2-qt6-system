#ifndef GLOBAL_NODE_HPP_
#define GLOBAL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "global_node/database_manager.hpp"

// Service interfaces
#include "ros2_interfaces/srv/save_cable.hpp"
#include "ros2_interfaces/srv/delete_cable.hpp"
#include "ros2_interfaces/srv/list_cables.hpp"
#include "ros2_interfaces/srv/get_cable_info_batch.hpp"

#include <memory>

class GlobalNode : public rclcpp::Node
{
public:
    GlobalNode();

private:
    // --- 服务回调函数 ---
    void save_cable_callback(
        const std::shared_ptr<ros2_interfaces::srv::SaveCable::Request> request,
        std::shared_ptr<ros2_interfaces::srv::SaveCable::Response> response);

    void delete_cable_callback(
        const std::shared_ptr<ros2_interfaces::srv::DeleteCable::Request> request,
        std::shared_ptr<ros2_interfaces::srv::DeleteCable::Response> response);

    void list_cables_callback(
        const std::shared_ptr<ros2_interfaces::srv::ListCables::Request> request,
        std::shared_ptr<ros2_interfaces::srv::ListCables::Response> response);

    void get_cable_info_batch_callback(
        const std::shared_ptr<ros2_interfaces::srv::GetCableInfoBatch::Request> request,
        std::shared_ptr<ros2_interfaces::srv::GetCableInfoBatch::Response> response);

    // --- 核心组件 ---
    std::unique_ptr<DatabaseManager> db_manager_;

    // --- ROS 服务服务器 ---
    rclcpp::Service<ros2_interfaces::srv::SaveCable>::SharedPtr save_cable_service_;
    rclcpp::Service<ros2_interfaces::srv::DeleteCable>::SharedPtr delete_cable_service_;
    rclcpp::Service<ros2_interfaces::srv::ListCables>::SharedPtr list_cables_service_;
    rclcpp::Service<ros2_interfaces::srv::GetCableInfoBatch>::SharedPtr get_cable_info_batch_service_;
};

#endif // GLOBAL_NODE_HPP_
