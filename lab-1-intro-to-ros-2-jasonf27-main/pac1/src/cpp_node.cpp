#include "pac1/cpp_header.hpp"
// int main(int argc, char **argv) { return 0 }

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("my_node_name");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
