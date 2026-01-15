#include "joint_cmd_mux/control_modes.hpp"

#include "joint_cmd_mux/unitree_interface.hpp"

namespace joint_cmd_mux {

    namespace detail {

        template <class... Ts>
        struct overload : Ts... {
            using Ts::operator()...;
        };

        template <class... Ts>
        overload(Ts...) -> overload<Ts...>;

    } // namespace detail

    bool can_transition(const ControlMode& from, const ControlMode& to) {
        return std::visit(
            detail::overload{
                // std::monostate
                [](const std::monostate&, const IdleMode&) { return true; },
                [](const std::monostate&, const HighLevelMode&) { return true; },
                [](const std::monostate&, const LowLevelMode&) { return true; },
                [](const std::monostate&, const EmergencyStop&) { return true; },

                // IdleMode
                [](const IdleMode&, const HighLevelMode&) { return true; },
                [](const IdleMode&, const LowLevelMode&) { return true; },
                [](const IdleMode&, const EmergencyStop&) { return true; },

                // HighLevelMode
                [](const HighLevelMode&, const IdleMode&) { return true; },
                [](const HighLevelMode&, const LowLevelMode&) { return true; },
                [](const HighLevelMode&, const EmergencyStop&) { return true; },

                // LowLevelMode
                [](const LowLevelMode&, const IdleMode&) { return true; },
                [](const LowLevelMode&, const HighLevelMode&) { return true; },
                [](const LowLevelMode&, const EmergencyStop&) { return true; },

                // EmergencyStop
                // The fallback would catch this case anyways. This just makes it clear
                // that we cannot transition away from EmergencyStop
                [](const EmergencyStop&, auto&&) { return false; },

                // fallback: deny anything else
                [](auto&&, auto&&) { return false; }
            },
            from, to
        );
    }

    // TODO: Handle transition errors
    void transition(const ControlMode& from, const ControlMode& to, UnitreeInterface& unitree_interface) {
        if (!unitree_interface.is_initialized()) {
            // Log error but don't throw - let caller handle it
            return;
        }

        std::visit(
            detail::overload{// std::monostate -> IdleMode: No action needed
                [](const std::monostate&, const IdleMode&) {
                    // No API call needed, just set state
                },

                // std::monostate -> HighLevelMode: Select high-level mode
                [&unitree_interface](const std::monostate&, const HighLevelMode&) {
                    unitree_interface.select_mode("sport_mode");
                },

                // std::monostate -> LowLevelMode: Release any active modes
                [&unitree_interface](const std::monostate&, const LowLevelMode&) {
                    unitree_interface.release_mode();
                },

                // std::monostate -> EmergencyStop: Emergency stop
                [&unitree_interface](const std::monostate&, const EmergencyStop&) {
                    unitree_interface.emergency_stop();
                },

                // IdleMode -> HighLevelMode: Select high-level mode
                [&unitree_interface](const IdleMode&, const HighLevelMode&) {
                    unitree_interface.select_mode("sport_mode");
                },

                // IdleMode -> LowLevelMode: Release any active modes, ready for low-level
                [&unitree_interface](const IdleMode&, const LowLevelMode&) {
                    unitree_interface.release_mode();
                },

                // IdleMode -> EmergencyStop: Stop all motion immediately
                [&unitree_interface](const IdleMode&, const EmergencyStop&) {
                    unitree_interface.emergency_stop();
                },

                // HighLevelMode -> IdleMode: Release high-level control
                [&unitree_interface](const HighLevelMode&, const IdleMode&) {
                    unitree_interface.release_mode();
                    unitree_interface.damp(); // Safe transition to idle
                },

                // HighLevelMode -> LowLevelMode: Release high-level motion control
                [&unitree_interface](const HighLevelMode&, const LowLevelMode&) {
                    unitree_interface.release_mode();
                },

                // HighLevelMode -> EmergencyStop: Stop all motion immediately
                [&unitree_interface](const HighLevelMode&, const EmergencyStop&) {
                    unitree_interface.emergency_stop();
                },

                // LowLevelMode -> IdleMode: Stop sending commands
                [&unitree_interface](const LowLevelMode&, const IdleMode&) {
                    // No API call needed - just stop publishing low-level commands
                    // Optionally damp for safety
                    unitree_interface.damp();
                },

                // LowLevelMode -> HighLevelMode: Select high-level motion control mode
                [&unitree_interface](const LowLevelMode&, const HighLevelMode&) {
                    unitree_interface.select_mode("sport_mode");
                },

                // LowLevelMode -> EmergencyStop: Stop all motion immediately
                [&unitree_interface](const LowLevelMode&, const EmergencyStop&) {
                    unitree_interface.emergency_stop();
                },

                // EmergencyStop -> Any: Cannot transition from emergency stop
                [](const EmergencyStop&, auto&&) {
                    // No action - transitions from EmergencyStop are not allowed
                    // (This case should be caught by can_transition check before calling transition)
                },

                // Fallback: no-op for any other transitions
                [](auto&&, auto&&) {
                    // No action needed
                }
            },
            from, to
        );
    }
} // namespace joint_cmd_mux