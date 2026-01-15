#include "joint_cmd_mux/joint_cmd_mux.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    auto node = std::make_shared<joint_cmd_mux::JointCmdMux>(options);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}