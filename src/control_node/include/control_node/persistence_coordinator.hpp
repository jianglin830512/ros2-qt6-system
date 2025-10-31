// --- START OF FILE persistence_coordinator.hpp ---

#ifndef PERSISTENCE_COORDINATOR_HPP
#define PERSISTENCE_COORDINATOR_HPP

#include "control_node/state_manager.hpp"
#include "control_node/control_node_context.hpp"
#include <memory>
#include <functional> // 用于 std::function
#include <string>

// 定义一个通用的回调函数类型，用于报告异步操作的结果
// 参数: bool success - 操作是否成功, const std::string& message - 附带的消息
using PersistenceCallback = std::function<void(bool success, const std::string& message)>;

/**
 * @class PersistenceCoordinator
 * @brief 负责与持久化服务（如RECORD_NODE）的所有通信。
 *
 * 这个类封装了调用ROS服务的异步逻辑，
 * 为上层提供非阻塞的、基于回调的方法调用接口。
 */
class PersistenceCoordinator
{
public:
    /**
     * @brief 构造函数，注入依赖项。
     * @param state_manager 指向StateManager的指针，用于获取要保存的数据。
     * @param context 指向共享上下文的指针，用于获取ROS服务客户端。
     */
    PersistenceCoordinator(
        StateManager* state_manager,
        std::shared_ptr<ControlNodeContext> context);

    /**
     * @brief 异步保存系统设置。
     * @param callback 操作完成后的回调函数。
     */
    void save_system_settings(PersistenceCallback callback);

    /**
     * @brief 异步保存特定ID的调压器设置。
     * @param regulator_id 要保存的调压器ID。
     * @param callback 操作完成后的回调函数。
     */
    void save_regulator_settings(uint8_t regulator_id, PersistenceCallback callback);

    /**
     * @brief 异步保存特定ID的回路设置。
     * @param circuit_id 要保存的回路ID。
     * @param callback 操作完成后的回调函数。
     */
    void save_circuit_settings(uint8_t circuit_id, PersistenceCallback callback);

private:
    StateManager* state_manager_;
    std::shared_ptr<ControlNodeContext> context_;
};

#endif // PERSISTENCE_COORDINATOR_HPP
