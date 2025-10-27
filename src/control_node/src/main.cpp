// control_node/src/main.cpp

#include "control_node/control_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>(); // 创建您的节点对象
    rclcpp::spin(node); // 阻塞并等待回调，直到节点关闭
    rclcpp::shutdown();
    return 0;
}