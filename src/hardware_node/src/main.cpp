#include "hardware_node/hardware_node.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // 1. 创建节点实例
    auto hardware_node = std::make_shared<HardwareNode>();

    // 2. 调用新的初始化方法来构建所有组件和ROS句柄
    hardware_node->initialize_components();

    // 3. 创建多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor;

    // 4. 将节点添加到执行器
    executor.add_node(hardware_node);

    // 5. 启动执行器
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
