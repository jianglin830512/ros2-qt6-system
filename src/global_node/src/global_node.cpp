#include "global_node/global_node.hpp"
#include "global_node/global_node_constants.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

GlobalNode::GlobalNode() : Node("global_node")
{
    RCLCPP_INFO(this->get_logger(), "Initializing GlobalNode...");

    // 1. 初始化 DatabaseManager
    auto db_path = this->declare_parameter<std::string>(
        global_node_constants::DB_PATH_PARAM,
        global_node_constants::DEFAULT_DB_PATH);
    db_manager_ = std::make_unique<DatabaseManager>(db_path, this->get_logger());

    // 2. 注册服务：保存/更新电缆
    auto save_svc_name = this->declare_parameter<std::string>(
        global_node_constants::SAVE_CABLE_SERVICE_PARAM,
        global_node_constants::DEFAULT_SAVE_CABLE_SERVICE);
    save_cable_service_ = this->create_service<ros2_interfaces::srv::SaveCable>(
        save_svc_name, std::bind(&GlobalNode::save_cable_callback, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Service initialized: %s", save_svc_name.c_str());

    // 3. 注册服务：删除电缆
    auto delete_svc_name = this->declare_parameter<std::string>(
        global_node_constants::DELETE_CABLE_SERVICE_PARAM,
        global_node_constants::DEFAULT_DELETE_CABLE_SERVICE);
    delete_cable_service_ = this->create_service<ros2_interfaces::srv::DeleteCable>(
        delete_svc_name, std::bind(&GlobalNode::delete_cable_callback, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Service initialized: %s", delete_svc_name.c_str());

    // 4. 注册服务：分页查询电缆列表
    auto list_svc_name = this->declare_parameter<std::string>(
        global_node_constants::LIST_CABLES_SERVICE_PARAM,
        global_node_constants::DEFAULT_LIST_CABLES_SERVICE);
    list_cables_service_ = this->create_service<ros2_interfaces::srv::ListCables>(
        list_svc_name, std::bind(&GlobalNode::list_cables_callback, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Service initialized: %s", list_svc_name.c_str());

    // 5. 注册服务：批量查询电缆 (用于 Control Node 通信)
    auto batch_svc_name = this->declare_parameter<std::string>(
        global_node_constants::GET_CABLE_INFO_BATCH_SERVICE_PARAM,
        global_node_constants::DEFAULT_GET_CABLE_INFO_BATCH_SERVICE);
    get_cable_info_batch_service_ = this->create_service<ros2_interfaces::srv::GetCableInfoBatch>(
        batch_svc_name, std::bind(&GlobalNode::get_cable_info_batch_callback, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Service initialized: %s", batch_svc_name.c_str());

    RCLCPP_INFO(this->get_logger(), "GlobalNode initialization complete.");
}

void GlobalNode::save_cable_callback(
    const std::shared_ptr<ros2_interfaces::srv::SaveCable::Request> request,
    std::shared_ptr<ros2_interfaces::srv::SaveCable::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received SaveCable request for ID: %d, Name: %s",
                request->cable_data.id, request->cable_data.name.c_str());

    std::string error_msg;
    bool success = db_manager_->save_cable(request->cable_data, error_msg);

    response->success = success;
    response->message = error_msg;
}

void GlobalNode::delete_cable_callback(
    const std::shared_ptr<ros2_interfaces::srv::DeleteCable::Request> request,
    std::shared_ptr<ros2_interfaces::srv::DeleteCable::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received DeleteCable request for ID: %d", request->id);

    std::string error_msg;
    bool success = db_manager_->delete_cable(request->id, error_msg);

    response->success = success;
    response->message = error_msg;
}

void GlobalNode::list_cables_callback(
    const std::shared_ptr<ros2_interfaces::srv::ListCables::Request> request,
    std::shared_ptr<ros2_interfaces::srv::ListCables::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received ListCables request. Keyword: '%s', Page: %d",
                request->search_keyword.c_str(), request->page);

    std::string error_msg;
    std::vector<ros2_interfaces::msg::Cable> cables;
    int32_t total_pages = 0;
    int32_t current_page = request->page;

    // 传入新增的排序参数
    bool success = db_manager_->list_cables(
        request->search_keyword,
        request->page,
        request->page_size,
        request->sort_column,
        request->is_ascending,
        cables,
        total_pages,
        current_page,
        error_msg
        );

    response->success = success;
    response->message = error_msg;
    response->total_pages = total_pages;
    response->current_page = current_page;
    response->cables = cables;
}

void GlobalNode::get_cable_info_batch_callback(
    const std::shared_ptr<ros2_interfaces::srv::GetCableInfoBatch::Request> request,
    std::shared_ptr<ros2_interfaces::srv::GetCableInfoBatch::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received GetCableInfoBatch request for %zu IDs.", request->ids.size());

    std::string error_msg;
    std::vector<ros2_interfaces::msg::Cable> cables;

    bool success = db_manager_->get_cable_info_batch(request->ids, cables, error_msg);

    // 如果未查询到任何存在的ID，也可以将 success 置为 false，此处若查询过程无系统报错视为 true
    if (success && cables.empty() && !request->ids.empty()) {
        error_msg = "None of the requested IDs were found in the database.";
    }

    response->success = success;
    response->message = error_msg;
    response->cables = cables;
}
