#ifndef GLOBAL_NODE_CONSTANTS_HPP_
#define GLOBAL_NODE_CONSTANTS_HPP_

namespace global_node_constants
{
// 参数名称
[[maybe_unused]] constexpr const char* DB_PATH_PARAM = "db_path";
[[maybe_unused]] constexpr const char* DEFAULT_DB_PATH = "global.db";

// 服务名称参数
[[maybe_unused]] constexpr const char* SAVE_CABLE_SERVICE_PARAM = "save_cable_service";
[[maybe_unused]] constexpr const char* DELETE_CABLE_SERVICE_PARAM = "delete_cable_service";
[[maybe_unused]] constexpr const char* LIST_CABLES_SERVICE_PARAM = "list_cables_service";
[[maybe_unused]] constexpr const char* GET_CABLE_INFO_BATCH_SERVICE_PARAM = "get_cable_info_batch_service";

// 服务默认名称 (以 / 开头代表全局)
[[maybe_unused]] constexpr const char* DEFAULT_SAVE_CABLE_SERVICE = "/global/cable_manager/save";
[[maybe_unused]] constexpr const char* DEFAULT_DELETE_CABLE_SERVICE = "/global/cable_manager/delete";
[[maybe_unused]] constexpr const char* DEFAULT_LIST_CABLES_SERVICE = "/global/cable_manager/list";
[[maybe_unused]] constexpr const char* DEFAULT_GET_CABLE_INFO_BATCH_SERVICE = "/global/cable_manager/get_batch";

} // namespace global_node_constants

#endif // GLOBAL_NODE_CONSTANTS_HPP_
