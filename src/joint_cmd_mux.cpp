#include "joint_cmd_mux/joint_cmd_mux.hpp"
#include "joint_cmd_mux/control_modes.hpp"
#include "joint_cmd_mux/unitree_interface.hpp"

namespace joint_cmd_mux {

    JointCmdMux::JointCmdMux(const rclcpp::NodeOptions& options)
        : Node("joint_cmd_mux", options),
          current_mode_(std::monostate{}), // TODO: Check if we should start with a certain state
          unitree_interface_("eth0") {
    }

    void JointCmdMux::change_control_mode(const ControlMode& requested_mode) {
        if (can_transition(current_mode_, requested_mode)) {
            transition(current_mode_, requested_mode, unitree_interface_);
        }
    }

} // namespace joint_cmd_mux