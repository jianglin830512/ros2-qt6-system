#include "global_node/global_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
