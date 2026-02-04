#include "control_node/control_node.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include <iostream> // 引入 IO 流

int main(int argc, char * argv[])
{
    // 增加全局异常捕获
    try {
        rclcpp::init(argc, argv);

        // 1. 创建节点实例
        auto control_node = std::make_shared<ControlNode>();

        // 2. 调用新的初始化方法
        control_node->initialize_components();

        // 3. 创建多线程执行器
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 10);

        // 4. 将节点添加到执行器
        executor.add_node(control_node);

        // 5. 启动执行器
        RCLCPP_INFO(control_node->get_logger(), "Executor starting...");
        executor.spin();

        rclcpp::shutdown();
    }
    catch (const std::exception& e) {
        // 捕获标准异常（如 vector越界、空指针逻辑错误等）
        std::cerr << "CRITICAL ERROR: Uncaught exception in main: " << e.what() << std::endl;
        return 1;
    }
    catch (...) {
        // 捕获未知异常
        std::cerr << "CRITICAL ERROR: Unknown exception occurred!" << std::endl;
        return 1;
    }

    return 0;
}
