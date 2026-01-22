#include "unitree_interface/unitree_interface.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    auto node = unitree_interface::UnitreeInterface::make_shared(options);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}