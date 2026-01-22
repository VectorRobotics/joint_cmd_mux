#ifndef VECTOR_UNITREE_SDK_WRAPPER_HPP
#define VECTOR_UNITREE_SDK_WRAPPER_HPP

#include <rclcpp/node.hpp>
#include <rclcpp/logging.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <memory>
#include <string>

// ========== Forward declarations ==========
namespace unitree::robot::b2 {
    class MotionSwitcherClient;
}

namespace unitree::robot::g1 {
    class LocoClient;
}

namespace unitree_interface {

    // ========== I swear they exist ==========
    class IdleMode;
    class HighLevelMode;
    class LowLevelMode;
    class EmergencyMode;
    template <typename From, typename To> struct Transition;

    class UnitreeSDKWrapper {
    public:
        explicit UnitreeSDKWrapper(
            const rclcpp::Node::SharedPtr& node,
            std::string network_interface,
            rclcpp::Logger logger = rclcpp::get_logger("unitree_interface")
        );

        // Can't copy
        UnitreeSDKWrapper(const UnitreeSDKWrapper&) = delete;
        UnitreeSDKWrapper& operator=(const UnitreeSDKWrapper&) = delete;

        // Can move
        UnitreeSDKWrapper(UnitreeSDKWrapper&&) noexcept;
        UnitreeSDKWrapper& operator=(UnitreeSDKWrapper&&) noexcept;

        ~UnitreeSDKWrapper();

        [[nodiscard]]
        rclcpp::Logger get_logger() const { return logger_; }

        bool initialize();

        [[nodiscard]]
        bool is_initialized() const { return initialized_; }

        [[nodiscard]]
        std::pair<std::string, std::string> get_current_mode() const;

        [[nodiscard]]
        bool has_active_mode() const;

    private:
        // ========== Internal capabilities ==========
        bool release_mode();

        bool select_mode(const std::string& mode_name);

        bool damp();

        // ========== High-level capabilities ==========
        bool send_velocity_command_impl(const geometry_msgs::msg::Twist& message);

        bool send_speech_command_impl(const std::string& message);

        // TODO: Figure out if any other high-level capabilities need to be added

        // ========== Low-level capabilities ==========
        bool set_joint_motor_gains_impl();

        bool send_joint_control_command_impl();

        // TODO: Figure out if any other low-level capabilities need to be added

        // ========== Mode creation ==========
        [[nodiscard]]
        IdleMode create_idle_mode() const;

        [[nodiscard]]
        HighLevelMode create_high_level_mode() const;

        [[nodiscard]]
        LowLevelMode create_low_level_mode() const;

        [[nodiscard]]
        EmergencyMode create_emergency_mode() const;

        // ========== Look mom, I have friends! ==========
        friend class IdleMode;
        friend class HighLevelMode;
        friend class LowLevelMode;
        friend class EmergencyMode;
        template <typename From, typename To> friend struct Transition;

        std::string network_interface_;
        rclcpp::Logger logger_;
        bool initialized_;

        std::unique_ptr<unitree::robot::b2::MotionSwitcherClient> msc_;
        std::unique_ptr<unitree::robot::g1::LocoClient> loco_client_;
    };

} // namespace unitree_interface

#endif // VECTOR_UNITREE_SDK_WRAPPER_HPP
