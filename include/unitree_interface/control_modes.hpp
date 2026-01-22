#ifndef VECTOR_CONTROL_MODES_HPP
#define VECTOR_CONTROL_MODES_HPP

#include <geometry_msgs/msg/twist.hpp>

#include <string>
#include <variant>

namespace unitree_interface {

    class UnitreeSDKWrapper;

    // ========== Control modes ==========
    class IdleMode {
    public:
        [[nodiscard]]
        static constexpr const char* name() { return "Idle"; }

    private:
        friend class UnitreeSDKWrapper;

        IdleMode() = default;
    };

    class HighLevelMode {
    public:
        [[nodiscard]]
        static constexpr const char* name() { return "HighLevel"; }

        void send_velocity_command(
            UnitreeSDKWrapper& sdk_wrapper,
            const geometry_msgs::msg::Twist& message
        );

        void send_speech_command(
            UnitreeSDKWrapper& sdk_wrapper,
            const std::string& message
        );

    private:
        friend class UnitreeSDKWrapper;

        HighLevelMode() = default;
    };

    class LowLevelMode {
    public:
        [[nodiscard]]
        static constexpr const char* name() { return "LowLevel"; }

        void set_joint_motor_gains(
            UnitreeSDKWrapper& sdk_wrapper
        );

        void send_joint_control_command(
            UnitreeSDKWrapper& sdk_wrapper
        );

    private:
        friend class UnitreeSDKWrapper;

        LowLevelMode() = default;
    };

    class EmergencyMode {
    public:
        [[nodiscard]]
        static constexpr const char* name() { return "EmergencyStop"; }

        bool damp(UnitreeSDKWrapper& sdk_wrapper);

    private:
        friend class UnitreeSDKWrapper;

        EmergencyMode() = default;
    };

    // clang-format off
    using ControlMode = std::variant<
        std::monostate,
        IdleMode,
        HighLevelMode,
        LowLevelMode,
        EmergencyMode
    >;
    // clang-format on

    // ========== Control mode traits ==========
    template <typename T>
    struct ControlModeTraits {
        static constexpr const char* name() { return T::name(); }
    };

    template <>
    struct ControlModeTraits<std::monostate> {
        static constexpr const char* name() { return "std::monostate"; }
    };

    // ========== Helper functions ==========
    bool can_transition(const ControlMode& from, const ControlMode& to);

    ControlMode execute_transition(
        const ControlMode& from,
        const ControlMode& to,
        UnitreeSDKWrapper& sdk_wrapper
    );

} // namespace unitree_interface

#endif // VECTOR_CONTROL_MODES_HPP