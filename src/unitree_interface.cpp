#include "joint_cmd_mux/unitree_interface.hpp"

#include <rclcpp/logging.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

#include <utility>

namespace joint_cmd_mux {

    UnitreeInterface::UnitreeInterface(std::string network_interface, rclcpp::Logger logger)
        : network_interface_(network_interface),
          logger_(logger),
          initialized_(false) {
    }

    UnitreeInterface::UnitreeInterface(UnitreeInterface&& other)
        : network_interface_(std::move(other.network_interface_)),
          logger_(other.logger_),
          initialized_(other.initialized_),
          msc_(std::move(other.msc_)),
          loco_client_(std::move(other.loco_client_)) {
        other.initialized_ = false;
    }

    UnitreeInterface& UnitreeInterface::operator=(UnitreeInterface&& other) {
        if (this != &other) {
            network_interface_ = std::move(other.network_interface_);
            logger_ = other.logger_;
            initialized_ = other.initialized_;
            msc_ = std::move(other.msc_);
            loco_client_ = std::move(other.loco_client_);

            other.initialized_ = false;
        }

        return *this;
    }

    // TODO: Make error handling more robust (tl::expected?)

    bool UnitreeInterface::initialize() {
        if (initialized_) {
            RCLCPP_WARN(logger_, "UnitreeInterface already initialized");
            return true;
        }

        try {
            // Initialize ChannelFactory
            unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);
            RCLCPP_INFO(logger_, "ChannelFactory initialized with interface: %s", network_interface_.c_str());

            // Initialize MotionSwitcherClient
            msc_ = std::make_unique<unitree::robot::b2::MotionSwitcherClient>();
            msc_->SetTimeout(5.0f);
            msc_->Init();
            RCLCPP_INFO(logger_, "MotionSwitcherClient initialized");

            // Initialize LocoClient
            loco_client_ = std::make_unique<unitree::robot::g1::LocoClient>();
            loco_client_->SetTimeout(10.0f);
            loco_client_->Init();
            RCLCPP_INFO(logger_, "LocoClient initialized");

            initialized_ = true;
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Failed to initialize UnitreeInterface: %s", e.what());
            initialized_ = false;
            return false;
        }
    }

    std::pair<std::string, std::string> UnitreeInterface::get_current_mode() const {
        if (!initialized_ || !msc_) {
            RCLCPP_ERROR(logger_, "UnitreeInterface not initialized");
            return {"", ""};
        }

        std::string form, name;
        msc_->CheckMode(form, name);

        return {form, name};
    }

    bool UnitreeInterface::has_active_mode() const {
        auto [form, name] = get_current_mode();

        return !name.empty();
    }

    bool UnitreeInterface::release_mode() {
        if (!initialized_ || !msc_) {
            RCLCPP_ERROR(logger_, "UnitreeInterface not initialized");
            return false;
        }

        std::string form, name;
        msc_->CheckMode(form, name);

        if (name.empty()) {
            RCLCPP_INFO(logger_, "No active motion control mode - already released");
            return true;
        }
    
        RCLCPP_INFO(logger_, "Attempting to release mode: %s (form: %s)", name.c_str(), form.c_str());
        const int32_t ret = msc_->ReleaseMode();
        
        if (ret == 0) {
            RCLCPP_INFO(logger_, "ReleaseMode succeeded");
            return true;
        }
        
        RCLCPP_WARN(logger_, "ReleaseMode failed with error code: %d", ret);
        return false;
    }

    bool UnitreeInterface::select_mode(const std::string& mode_name) {
        if (!initialized_ || !msc_) {
            RCLCPP_ERROR(logger_, "UnitreeInterface not initialized");
            return false;
        }

        if (mode_name.empty()) {
            RCLCPP_ERROR(logger_, "Mode name cannot be empty");
            return false;
        }

        RCLCPP_INFO(logger_, "Selecting mode: %s", mode_name.c_str());
        const int32_t ret = msc_->SelectMode(mode_name);

        if (ret == 0) {
            RCLCPP_INFO(logger_, "Successfully selected mode: %s", mode_name.c_str());
            return true;
        } else {
            RCLCPP_ERROR(logger_, "SelectMode failed with error code: %d", ret);
            return false;
        }
    }

    bool UnitreeInterface::damp() {
        if (!initialized_ || !loco_client_) {
            RCLCPP_ERROR(logger_, "UnitreeInterface not initialized");
            return false;
        }

        RCLCPP_INFO(logger_, "Entering damp mode");
        const int32_t ret = loco_client_->Damp();

        if (ret == 0) {
            RCLCPP_INFO(logger_, "Damp mode activated");
            return true;
        } else {
            RCLCPP_WARN(logger_, "Damp failed with error code: %d", ret);
            return false;
        }
    }

    bool UnitreeInterface::emergency_stop() {
        if (!initialized_) {
            RCLCPP_ERROR(logger_, "UnitreeInterface not initialized");
            return false;
        }

        RCLCPP_WARN(logger_, "Emergency stop activated");

        bool success = true;

        if (msc_) {
            // Don't wait for retries in emergency stop
            std::string form, name;
            msc_->CheckMode(form, name);
            if (!name.empty()) {
                const int32_t ret = msc_->ReleaseMode();
                if (ret != 0) {
                    RCLCPP_ERROR(logger_, "Emergency stop: ReleaseMode failed with code: %d", ret);
                    success = false;
                }
            }
        }

        // Then, try to damp
        if (loco_client_) {
            const int32_t ret = loco_client_->Damp();
            if (ret != 0) {
                RCLCPP_ERROR(logger_, "Emergency stop: Damp failed with code: %d", ret);
                success = false;
            }
        }

        return success;
    }

} // namespace joint_cmd_mux
