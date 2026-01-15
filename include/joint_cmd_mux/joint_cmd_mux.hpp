#ifndef VECTOR_JOINT_CMD_MUX_HPP
#define VECTOR_JOINT_CMD_MUX_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>

#include "joint_cmd_mux/control_modes.hpp"
#include "joint_cmd_mux/unitree_interface.hpp"

namespace joint_cmd_mux {

    class JointCmdMux : public rclcpp::Node {
    public:
        JointCmdMux(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        ~JointCmdMux() = default;

    private:
        void change_control_mode(const ControlMode& requested_mode);

        ControlMode current_mode_;
        UnitreeInterface unitree_interface_;
    };

} // namespace joint_cmd_mux

#endif