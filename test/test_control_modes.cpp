// #include "joint_cmd_mux/control_modes.hpp"

// namespace test {

// // Tests for control_modes functionality

// #include <gtest/gtest.h>
// #include <rclcpp/rclcpp.hpp>
// #include <type_traits>
// #include <variant>

// // Dummy/mock UnitreeInterface for testing transition side effects
// class MockUnitreeInterface {
// public:
//     MockUnitreeInterface() : initialized(true), 
//         select_mode_called(false), 
//         release_mode_called(false), 
//         emergency_stop_called(false), 
//         damp_called(false) {}

//     bool is_initialized() const { return initialized; }
//     void select_mode(const std::string& mode) { select_mode_called = true; last_mode = mode; }
//     void release_mode() { release_mode_called = true; }
//     void emergency_stop() { emergency_stop_called = true; }
//     void damp() { damp_called = true; }

//     void reset() {
//         select_mode_called = false;
//         release_mode_called = false;
//         emergency_stop_called = false;
//         damp_called = false;
//         last_mode = "";
//     }

//     // For simulating uninitialized interface
//     void set_initialized(bool val) { initialized = val; }

//     // Flags for checking which calls were made
//     bool select_mode_called;
//     bool release_mode_called;
//     bool emergency_stop_called;
//     bool damp_called;
//     std::string last_mode;

// private:
//     bool initialized;
// };

// // Adapter for signature-compatibility
// struct MockUnitreeInterfaceAdaptor : public joint_cmd_mux::UnitreeInterface {
//     MockUnitreeInterfaceAdaptor(MockUnitreeInterface& mock) : mock_(mock) {}
//     bool is_initialized() const override { return mock_.is_initialized(); }
//     void select_mode(const std::string& mode) override { mock_.select_mode(mode); }
//     void release_mode() override { mock_.release_mode(); }
//     void emergency_stop() override { mock_.emergency_stop(); }
//     void damp() override { mock_.damp(); }
//     MockUnitreeInterface& mock_;
// };

// // Helper to create rclcpp::Time (zero for tests)
// inline rclcpp::Time make_time() { return rclcpp::Time(0, 0, RCL_ROS_TIME); }

// // Helper to create control modes
// inline joint_cmd_mux::IdleMode make_idle() {
//     joint_cmd_mux::IdleMode m; m.stamp = make_time(); return m;
// }
// inline joint_cmd_mux::HighLevelMode make_high() {
//     joint_cmd_mux::HighLevelMode m; m.stamp = make_time(); return m;
// }
// inline joint_cmd_mux::LowLevelMode make_low() {
//     joint_cmd_mux::LowLevelMode m; m.stamp = make_time(); return m;
// }
// inline joint_cmd_mux::EmergencyStop make_emer() {
//     joint_cmd_mux::EmergencyStop m; m.stamp = make_time(); return m;
// }

// // can_transition tests
// TEST(ControlModes, CanTransitionSimpleCases) {
//     using joint_cmd_mux::ControlMode;

//     // From monostate to others
//     EXPECT_TRUE(joint_cmd_mux::can_transition(ControlMode{std::monostate{}}, ControlMode{make_idle()}));
//     EXPECT_TRUE(joint_cmd_mux::can_transition(ControlMode{std::monostate{}}, ControlMode{make_high()}));
//     EXPECT_TRUE(joint_cmd_mux::can_transition(ControlMode{std::monostate{}}, ControlMode{make_low()}));
//     EXPECT_TRUE(joint_cmd_mux::can_transition(ControlMode{std::monostate{}}, ControlMode{make_emer()}));

//     // Between modes
//     EXPECT_TRUE(joint_cmd_mux::can_transition(ControlMode{make_idle()}, ControlMode{make_high()}));
//     EXPECT_TRUE(joint_cmd_mux::can_transition(ControlMode{make_idle()}, ControlMode{make_emer()}));
//     EXPECT_TRUE(joint_cmd_mux::can_transition(ControlMode{make_high()}, ControlMode{make_idle()}));
//     EXPECT_TRUE(joint_cmd_mux::can_transition(ControlMode{make_high()}, ControlMode{make_low()}));
//     EXPECT_TRUE(joint_cmd_mux::can_transition(ControlMode{make_low()}, ControlMode{make_idle()}));
//     EXPECT_TRUE(joint_cmd_mux::can_transition(ControlMode{make_low()}, ControlMode{make_high()}));

//     // Can't transition OUT OF EmergencyStop
//     EXPECT_FALSE(joint_cmd_mux::can_transition(ControlMode{make_emer()}, ControlMode{make_idle()}));
//     EXPECT_FALSE(joint_cmd_mux::can_transition(ControlMode{make_emer()}, ControlMode{make_high()}));
//     EXPECT_FALSE(joint_cmd_mux::can_transition(ControlMode{make_emer()}, ControlMode{make_low()}));
//     EXPECT_FALSE(joint_cmd_mux::can_transition(ControlMode{make_emer()}, ControlMode{make_emer()}));
// }

// // can_transition: fallback cases
// TEST(ControlModes, CanTransitionFallback) {
//     using joint_cmd_mux::ControlMode;
//     // From IdleMode to monostate (for example), should be false
//     EXPECT_FALSE(joint_cmd_mux::can_transition(ControlMode{make_idle()}, ControlMode{std::monostate{}}));
//     // HighLevelMode to monostate
//     EXPECT_FALSE(joint_cmd_mux::can_transition(ControlMode{make_high()}, ControlMode{std::monostate{}}));
// }

// // transition side effects tests
// TEST(ControlModes, TransitionCallsUnitreeInterface) {
//     using joint_cmd_mux::ControlMode;

//     MockUnitreeInterface mock;
//     MockUnitreeInterfaceAdaptor iface(mock);

//     // std::monostate -> IdleMode: nothing called
//     mock.reset();
//     joint_cmd_mux::transition(ControlMode{std::monostate{}}, ControlMode{make_idle()}, iface);
//     EXPECT_FALSE(mock.select_mode_called);
//     EXPECT_FALSE(mock.release_mode_called);
//     EXPECT_FALSE(mock.emergency_stop_called);
//     EXPECT_FALSE(mock.damp_called);

//     // std::monostate -> HighLevelMode: select_mode called
//     mock.reset();
//     joint_cmd_mux::transition(ControlMode{std::monostate{}}, ControlMode{make_high()}, iface);
//     EXPECT_TRUE(mock.select_mode_called);
//     EXPECT_EQ(mock.last_mode, "sport_mode");

//     // std::monostate -> LowLevelMode: release_mode called
//     mock.reset();
//     joint_cmd_mux::transition(ControlMode{std::monostate{}}, ControlMode{make_low()}, iface);
//     EXPECT_TRUE(mock.release_mode_called);

//     // std::monostate -> EmergencyStop: emergency_stop called
//     mock.reset();
//     joint_cmd_mux::transition(ControlMode{std::monostate{}}, ControlMode{make_emer()}, iface);
//     EXPECT_TRUE(mock.emergency_stop_called);

//     // IdleMode -> HighLevelMode: select_mode
//     mock.reset();
//     joint_cmd_mux::transition(ControlMode{make_idle()}, ControlMode{make_high()}, iface);
//     EXPECT_TRUE(mock.select_mode_called);

//     // IdleMode -> EmergencyStop: emergency_stop
//     mock.reset();
//     joint_cmd_mux::transition(ControlMode{make_idle()}, ControlMode{make_emer()}, iface);
//     EXPECT_TRUE(mock.emergency_stop_called);

//     // HighLevelMode -> IdleMode: release_mode + damp
//     mock.reset();
//     joint_cmd_mux::transition(ControlMode{make_high()}, ControlMode{make_idle()}, iface);
//     EXPECT_TRUE(mock.release_mode_called);
//     EXPECT_TRUE(mock.damp_called);

//     // LowLevelMode -> IdleMode: damp
//     mock.reset();
//     joint_cmd_mux::transition(ControlMode{make_low()}, ControlMode{make_idle()}, iface);
//     EXPECT_TRUE(mock.damp_called);

//     // LowLevelMode -> HighLevelMode: select_mode
//     mock.reset();
//     joint_cmd_mux::transition(ControlMode{make_low()}, ControlMode{make_high()}, iface);
//     EXPECT_TRUE(mock.select_mode_called);
// }

// TEST(ControlModes, TransitionFromEmergencyStopNoAction) {
//     using joint_cmd_mux::ControlMode;

//     MockUnitreeInterface mock;
//     MockUnitreeInterfaceAdaptor iface(mock);

//     // EmergencyStop -> IdleMode etc: should do nothing
//     mock.reset();
//     joint_cmd_mux::transition(ControlMode{make_emer()}, ControlMode{make_idle()}, iface);
//     EXPECT_FALSE(mock.release_mode_called);
//     EXPECT_FALSE(mock.damp_called);
//     EXPECT_FALSE(mock.select_mode_called);
//     EXPECT_FALSE(mock.emergency_stop_called);
// }

// TEST(ControlModes, TransitionUninitializedInterface) {
//     using joint_cmd_mux::ControlMode;

//     MockUnitreeInterface mock;
//     mock.set_initialized(false);
//     MockUnitreeInterfaceAdaptor iface(mock);

//     mock.reset();
//     // Should not call any methods if interface not initialized
//     joint_cmd_mux::transition(ControlMode{std::monostate{}}, ControlMode{make_high()}, iface);
//     EXPECT_FALSE(mock.select_mode_called);
// }

// //
// // Additional type and variant checks
// //

// TEST(ControlModes, ControlModeVariantHoldsExpectedTypes) {
//     using joint_cmd_mux::ControlMode;
//     ControlMode mode;
//     mode = std::monostate{};
//     EXPECT_TRUE(std::holds_alternative<std::monostate>(mode));
//     mode = make_idle();
//     EXPECT_TRUE(std::holds_alternative<joint_cmd_mux::IdleMode>(mode));
//     mode = make_high();
//     EXPECT_TRUE(std::holds_alternative<joint_cmd_mux::HighLevelMode>(mode));
//     mode = make_low();
//     EXPECT_TRUE(std::holds_alternative<joint_cmd_mux::LowLevelMode>(mode));
//     mode = make_emer();
//     EXPECT_TRUE(std::holds_alternative<joint_cmd_mux::EmergencyStop>(mode));
// }


// }