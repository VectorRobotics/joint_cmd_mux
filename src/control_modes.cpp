#include "unitree_interface/control_modes.hpp"

#include "unitree_interface/mode_transitions.hpp"
#include "unitree_interface/unitree_sdk_wrapper.hpp"

#include <rclcpp/logging.hpp>

#include <type_traits>

namespace unitree_interface {

    // ========== HighLevelMode ==========
    void HighLevelMode::send_velocity_command(
        UnitreeSDKWrapper& sdk_wrapper,
        const geometry_msgs::msg::Twist& message
    ) {
        sdk_wrapper.send_velocity_command_impl(message);
    }

    void HighLevelMode::send_speech_command(
        UnitreeSDKWrapper& sdk_wrapper,
        const std::string& message
    ) {
        sdk_wrapper.send_speech_command_impl(message);
    }

    // ========== LowLevelMode ==========
    void LowLevelMode::set_joint_motor_gains(UnitreeSDKWrapper& sdk_wrapper) {
        // TODO: Implement this
    }

    void LowLevelMode::send_joint_control_command(UnitreeSDKWrapper& sdk_wrapper) {
        // TODO: Implement this
    }

    // ========== EmergencyStop ==========
    bool EmergencyMode::damp(UnitreeSDKWrapper& sdk_wrapper) {
        return sdk_wrapper.damp();
    }

    // Runtime helpers
    bool can_transition(const ControlMode& from, const ControlMode& to) {
        return std::visit(
            [](auto&& f, auto&& t) -> bool {
                using FromType = std::decay_t<decltype(f)>;
                using ToType = std::decay_t<decltype(t)>;

                return Transition<FromType, ToType>::allowed;
            },
            from, to
        );
    }

    ControlMode execute_transition(
        const ControlMode& from,
        const ControlMode& to,
        UnitreeSDKWrapper& sdk_wrapper
    ) {
        if (!sdk_wrapper.is_initialized()) {
            RCLCPP_WARN(
                sdk_wrapper.get_logger(),
                "Attempted to execute a transition with an uninitialized sdk_wrapper"
            );
            return from;
        }

        return std::visit(
            [&](auto&& f, auto&& t) -> ControlMode {
                using FromType = std::decay_t<decltype(f)>;
                using ToType = std::decay_t<decltype(t)>;

                if constexpr (Transition<FromType, ToType>::allowed) {
                    RCLCPP_INFO(
                        sdk_wrapper.get_logger(),
                        "Attempting to execute transition from %s to %s",
                        ControlModeTraits<FromType>::name(),
                        ControlModeTraits<ToType>::name()
                    );
                    return Transition<FromType, ToType>::execute(sdk_wrapper);
                }

                // If the transition is not allowed
                RCLCPP_WARN(
                    sdk_wrapper.get_logger(),
                    "Attempted to execute an illegal transition from %s to %s",
                    ControlModeTraits<FromType>::name(),
                    ControlModeTraits<ToType>::name()
                );
                return from;
            },
            from, to
        );
    }

} // namespace unitree_interface