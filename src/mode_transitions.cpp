#include "unitree_interface/mode_transitions.hpp"

#include "unitree_interface/control_modes.hpp"
#include "unitree_interface/unitree_sdk_wrapper.hpp"

namespace unitree_interface {

    // TODO: Figure out if transitions need to be idempotent
    // TODO: Figure out error handling on state transitions

    // ========== std::monostate ==========
    ControlMode Transition<std::monostate, IdleMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        return sdk_wrapper.create_idle_mode();
    }

    ControlMode Transition<std::monostate, EmergencyMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        /*
        We only hold std::monostate when the system has been freshly initialized. The hope is that
        we initially do have high-level control services enabled to allow for the direct execution of
        the emergency stop procedure.
        */

        auto emergency_mode = sdk_wrapper.create_emergency_mode();

        if (emergency_mode.damp(sdk_wrapper)) {
            return emergency_mode;
        }

        return std::monostate{};
    }

    // ========== IdleMode ==========
    ControlMode Transition<IdleMode, HighLevelMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        // TODO: Check if the mode string here is appropriate for the version of the Motion
        // Control Service we're running with
        if (sdk_wrapper.has_active_mode() || sdk_wrapper.select_mode("mcf")) {
            return sdk_wrapper.create_high_level_mode();
        }

        return sdk_wrapper.create_idle_mode();
    }

    ControlMode Transition<IdleMode, LowLevelMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        if (!sdk_wrapper.has_active_mode() || sdk_wrapper.release_mode()) {
            return sdk_wrapper.create_low_level_mode();
        }

        return sdk_wrapper.create_idle_mode();
    }

    ControlMode Transition<IdleMode, EmergencyMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        /*
        Our modes don't truly reflect unitree's internal controller state. We can land up in
        a situation wherein we transition from LowLevelMode to IdleMode and attempt to transition
        to EmergencyMode. No longer knowing whether the hardware itself is in high-level or
        low-level mode, if we erroneously attempt to enter damping mode without high-level
        control services active, we'll land up in some real shit.
        */

        if (!sdk_wrapper.has_active_mode()) {
            // Not in high-level mode - attempt to transition
            auto possible_high_level_mode = Transition<IdleMode, HighLevelMode>::execute(sdk_wrapper);

            if (!std::holds_alternative<HighLevelMode>(possible_high_level_mode)) {
                return sdk_wrapper.create_idle_mode();
            }
        }

        auto emergency_mode = sdk_wrapper.create_emergency_mode();

        if (!emergency_mode.damp(sdk_wrapper)) {
            return sdk_wrapper.create_idle_mode();
        }

        return emergency_mode;
    }

    // ========== HighLevelMode ==========
    ControlMode Transition<HighLevelMode, IdleMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        return sdk_wrapper.create_idle_mode();
    }

    ControlMode Transition<HighLevelMode, LowLevelMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        if (!sdk_wrapper.has_active_mode() || sdk_wrapper.release_mode()) {
            return sdk_wrapper.create_low_level_mode();
        }

        return sdk_wrapper.create_high_level_mode();
    }

    ControlMode Transition<HighLevelMode, EmergencyMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        auto emergency_mode = sdk_wrapper.create_emergency_mode();

        if (emergency_mode.damp(sdk_wrapper)) {
            return emergency_mode;
        }

        return sdk_wrapper.create_high_level_mode();
    }

    // ========== LowLevelMode ==========
    ControlMode Transition<LowLevelMode, IdleMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        return sdk_wrapper.create_idle_mode();
    }

    ControlMode Transition<LowLevelMode, HighLevelMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        // TODO: Check if the mode string here is appropriate for the version of the Motion
        // Control Service we're running with
        if (sdk_wrapper.has_active_mode() || sdk_wrapper.select_mode("mcf")) {
            return sdk_wrapper.create_high_level_mode();
        }

        return sdk_wrapper.create_low_level_mode();
    }

    ControlMode Transition<LowLevelMode, EmergencyMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        auto possible_high_level_mode = Transition<LowLevelMode, HighLevelMode>::execute(sdk_wrapper);

        if (!std::holds_alternative<HighLevelMode>(possible_high_level_mode)) {
            return sdk_wrapper.create_low_level_mode();
        }

        auto emergency_mode = sdk_wrapper.create_emergency_mode();

        if (!emergency_mode.damp(sdk_wrapper)) {
            return sdk_wrapper.create_low_level_mode();
        }

        return emergency_mode;
    }

} // namespace unitree_interface