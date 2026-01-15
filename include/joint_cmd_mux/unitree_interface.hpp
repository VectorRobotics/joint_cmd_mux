#ifndef VECTOR_UNITREE_INTERFACE_HPP
#define VECTOR_UNITREE_INTERFACE_HPP

#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp> // Seems misleading, but is actually correct
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

#include <rclcpp/logger.hpp>

#include <memory>
#include <string>
#include <utility>

namespace joint_cmd_mux {

    class UnitreeInterface {
    public:
        using MotionSwitcherClientPtr = std::unique_ptr<unitree::robot::b2::MotionSwitcherClient>;
        using LocoClientPtr = std::unique_ptr<unitree::robot::g1::LocoClient>;

        explicit UnitreeInterface(
            std::string network_interface,
            rclcpp::Logger logger = rclcpp::get_logger("unitree_interface")
        );

        ~UnitreeInterface() = default;

        // Can't copy
        UnitreeInterface(const UnitreeInterface&) = delete;
        UnitreeInterface& operator=(const UnitreeInterface&) = delete;

        // Can move
        UnitreeInterface(UnitreeInterface&&);
        UnitreeInterface& operator=(UnitreeInterface&&);

        bool initialize();

        bool is_initialized() const {
            return initialized_;
        }

        std::pair<std::string, std::string> get_current_mode() const;

        bool has_active_mode() const;

        bool release_mode();

        bool select_mode(const std::string& mode_name);

        bool damp();

        bool emergency_stop();

    private:
        std::string network_interface_;
        rclcpp::Logger logger_;
        bool initialized_;

        MotionSwitcherClientPtr msc_;
        LocoClientPtr loco_client_;
    };

} // namespace joint_cmd_mux

#endif // VECTOR_UNITREE_INTERFACE_HPP
