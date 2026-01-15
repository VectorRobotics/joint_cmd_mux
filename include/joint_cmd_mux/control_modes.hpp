#ifndef VECTOR_CONTROL_MODES_HPP
#define VECTOR_CONTROL_MODES_HPP

#include "joint_cmd_mux/unitree_interface.hpp"

#include <rclcpp/time.hpp>

#include <variant>

namespace joint_cmd_mux {

    // Forward declaration
    class UnitreeInterface;

    // TODO: figure out if encoding the FSM as types is appropriate

    struct IdleMode {
        rclcpp::Time stamp;
    };

    struct HighLevelMode {
        rclcpp::Time stamp;
        // TODO: Add high level control data
    };

    struct LowLevelMode {
        rclcpp::Time stamp;
        // TODO: Add low level control data
    };

    struct EmergencyStop {
        rclcpp::Time stamp;
    };

    // clang-format off
    using ControlMode = std::variant<
        std::monostate,
        IdleMode,
        HighLevelMode,
        LowLevelMode,
        EmergencyStop
    >;
    // clang-format on

    bool can_transition(const ControlMode& from, const ControlMode& to);

    void transition(const ControlMode& from, const ControlMode& to, UnitreeInterface& unitree_interface);

} // namespace joint_cmd_mux

#endif