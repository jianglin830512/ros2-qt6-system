#include "control_node/control_node.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp" // 1. 包含頭文件

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto control_node = std::make_shared<ControlNode>();

    // 2. 創建一個多線程執行器
    // 您可以指定線程數，如果為0，它會根據CPU核心數自動決定
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 10);

    // 3. 將您的節點添加到執行器中
    executor.add_node(control_node);

    // 4. 讓執行器開始工作（spin）
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
