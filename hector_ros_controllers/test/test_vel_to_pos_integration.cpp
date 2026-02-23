// Integration test for VelocityToPositionCommandController with DynamicsMockHardware.
//
// Uses a 2-joint URDF with asymmetric torque/velocity limits to test
// realistic closed-loop behavior: controller writes position commands,
// hardware simulates dynamics, controller reads resulting state.

#include <cmath>
#include <string>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#define private public
#define protected public
#include <controller_interface/chainable_controller_interface.hpp>
#include <velocity_to_position_command_controller/velocity_to_position_command_controller.hpp>
#undef protected
#undef private

#include <controller_interface/controller_interface_params.hpp>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#pragma GCC diagnostic pop
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace
{

using VelToPosController =
    velocity_to_position_command_controller::VelocityToPositionCommandController;
using MoveState = velocity_to_position_command_controller::MoveState;

// ---------------------------------------------------------------------------
// Test constants
// ---------------------------------------------------------------------------

const auto TIME = rclcpp::Time( 0 );
const double PERIOD_SEC = 0.02; // 50 Hz like the real system
const auto PERIOD = rclcpp::Duration::from_seconds( PERIOD_SEC );

// ---------------------------------------------------------------------------
// 2-joint URDF with DynamicsMockHardware
// ---------------------------------------------------------------------------

// Two continuous joints on a heavy base, no gravity.
// flipper_strong: max_torque=5.0, max_velocity=2.0
// flipper_weak:   max_torque=2.0, max_velocity=1.2
const char *const URDF = R"(
<?xml version="1.0" encoding="utf-8"?>
<robot name="FlipperTestRobot">
  <link name="world"/>
  <joint name="base_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="world"/>
    <child link="base_link"/>
  </joint>
  <link name="base_link">
    <inertial>
      <mass value="100.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1.0" ixy="0" ixz="0" iyy="1.0" iyz="0" izz="1.0"/>
    </inertial>
  </link>

  <joint name="flipper_strong" type="continuous">
    <origin rpy="0 0 0" xyz="0 0.3 0"/>
    <parent link="base_link"/>
    <child link="link_strong"/>
    <axis xyz="0 1 0"/>
    <limit effort="10.0" velocity="10.0"/>
  </joint>
  <link name="link_strong">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="flipper_weak" type="continuous">
    <origin rpy="0 0 0" xyz="0 -0.3 0"/>
    <parent link="base_link"/>
    <child link="link_weak"/>
    <axis xyz="0 1 0"/>
    <limit effort="10.0" velocity="10.0"/>
  </joint>
  <link name="link_weak">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <ros2_control name="DynMockSystem" type="system">
    <hardware>
      <plugin>dynamics_mock_hardware/DynamicsMockHardware</plugin>
      <param name="gravity_x">0.0</param>
      <param name="gravity_y">0.0</param>
      <param name="gravity_z">0.0</param>
      <param name="position_kp">100.0</param>
      <param name="position_kd">20.0</param>
      <param name="velocity_kp">10.0</param>
      <param name="integration_dt">0.001</param>
    </hardware>
    <joint name="flipper_strong">
      <param name="max_torque">5.0</param>
      <param name="max_velocity">2.0</param>
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="flipper_weak">
      <param name="max_torque">2.0</param>
      <param name="max_velocity">1.2</param>
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
</robot>
)";

// ---------------------------------------------------------------------------
// ResourceManager helper (same pattern as dynamics_mock_hardware tests)
// ---------------------------------------------------------------------------

class TestableResourceManager : public hardware_interface::ResourceManager
{
public:
  explicit TestableResourceManager( rclcpp::Node::SharedPtr node, const std::string &urdf,
                                    bool activate_all = false, unsigned int cm_update_rate = 50 )
      : hardware_interface::ResourceManager( urdf, node->get_node_clock_interface(),
                                             node->get_node_logging_interface(), activate_all,
                                             cm_update_rate )
  {
  }
};

void activate_components( TestableResourceManager &rm,
                          const std::vector<std::string> &components = { "DynMockSystem" } )
{
  for ( const auto &component : components ) {
    rclcpp_lifecycle::State state( lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                   hardware_interface::lifecycle_state_names::ACTIVE );
    rm.set_component_state( component, state );
  }
}

} // namespace

// ===========================================================================
// Test fixture
// ===========================================================================

class VelToPosIntegrationTest : public ::testing::Test
{
protected:
  static constexpr unsigned int kUpdateRate = 50;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<TestableResourceManager> rm_;
  std::shared_ptr<VelToPosController> controller_;

  std::vector<std::string> joints_{ "flipper_strong", "flipper_weak" };

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>( "test_vel_to_pos_integration" );
    rm_ = std::make_unique<TestableResourceManager>( node_, std::string( URDF ) );
    activate_components( *rm_ );
    controller_ = std::make_shared<VelToPosController>();
  }

  void TearDown() override
  {
    controller_.reset();
    rm_.reset();
    node_.reset();
  }

  void initController( const std::vector<std::string> &sync_groups = {}, double kp_sync = 1.0 )
  {
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "test_vel_to_pos";
    params.robot_description = std::string( URDF );
    params.update_rate = kUpdateRate;
    params.controller_manager_update_rate = kUpdateRate;
    params.node_namespace = "";

    std::vector<rclcpp::Parameter> overrides = {
        rclcpp::Parameter( "joints", joints_ ),
        rclcpp::Parameter( "kp", 1.0 ),
        rclcpp::Parameter( "kd", 1.0 ),
        rclcpp::Parameter( "kp_sync", kp_sync ),
        rclcpp::Parameter( "braking_deceleration", 5.0 ),
        rclcpp::Parameter( "stopping_velocity_threshold", 0.005 ),
        rclcpp::Parameter( "passthrough_controller", std::string( "" ) ),
        rclcpp::Parameter( "e_stop_topic", std::string( "estop_board/hard_estop" ) ),
    };
    if ( !sync_groups.empty() ) {
      overrides.emplace_back( "synchronous_groups", sync_groups );
    } else {
      overrides.emplace_back( "synchronous_groups", std::vector<std::string>{} );
    }

    rclcpp::NodeOptions opts;
    opts.parameter_overrides( overrides );
    params.node_options = opts;

    auto result = controller_->init( params );
    ASSERT_EQ( result, controller_interface::return_type::OK );

    // Configure
    rclcpp_lifecycle::State unconfigured( lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                          "unconfigured" );
    auto cb = controller_->on_configure( unconfigured );
    ASSERT_EQ( cb, controller_interface::CallbackReturn::SUCCESS );
  }

  void wireInterfaces()
  {
    // Claim interfaces from ResourceManager and assign to controller
    controller_->command_interfaces_.clear();
    controller_->state_interfaces_.clear();

    for ( const auto &joint : joints_ ) {
      controller_->command_interfaces_.push_back(
          rm_->claim_command_interface( joint + "/position" ) );
    }
    for ( const auto &joint : joints_ ) {
      controller_->state_interfaces_.push_back( rm_->claim_state_interface( joint + "/position" ) );
      controller_->state_interfaces_.push_back( rm_->claim_state_interface( joint + "/velocity" ) );
    }
  }

  void activateController()
  {
    rclcpp_lifecycle::State inactive( lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                      "inactive" );
    auto cb = controller_->on_activate( inactive );
    ASSERT_EQ( cb, controller_interface::CallbackReturn::SUCCESS );
  }

  // Run one full closed-loop cycle: controller update → hardware write → hardware read
  void runCycle()
  {
    controller_->update_and_write_commands( TIME, PERIOD );
    ASSERT_EQ( rm_->write( TIME, PERIOD ).result, hardware_interface::return_type::OK );
    ASSERT_EQ( rm_->read( TIME, PERIOD ).result, hardware_interface::return_type::OK );
  }

  // Run N closed-loop cycles
  void runCycles( int n )
  {
    for ( int i = 0; i < n; ++i ) { runCycle(); }
  }

  // Set velocity command for both joints
  void setVelocityCommands( double vel0, double vel1 )
  {
    controller_->reference_interfaces_[0] = vel0;
    controller_->reference_interfaces_[1] = vel1;
  }

  // Read current position from hardware state
  double getPosition( size_t joint_idx ) const
  {
    return controller_->joint_position_states_[joint_idx];
  }
};

// ===========================================================================
// Tests
// ===========================================================================

// Both joints move when velocity is commanded
TEST_F( VelToPosIntegrationTest, BothJointsMoveWithVelocityCommand )
{
  initController();
  wireInterfaces();
  // Size reference_interfaces_ (normally done by CM calling on_export_reference_interfaces)
  controller_->reference_interfaces_.assign( joints_.size(),
                                             std::numeric_limits<double>::quiet_NaN() );
  activateController();

  // Command moderate velocity (within both joints' limits)
  setVelocityCommands( 1.0, 1.0 );
  runCycles( 100 ); // 2 seconds at 50 Hz

  const double pos_strong = getPosition( 0 );
  const double pos_weak = getPosition( 1 );

  // Both should have moved significantly
  EXPECT_GT( pos_strong, 0.5 ) << "Strong joint should have moved forward";
  EXPECT_GT( pos_weak, 0.5 ) << "Weak joint should have moved forward";
}

// Weak joint lags behind when velocity exceeds its capability
TEST_F( VelToPosIntegrationTest, WeakJointLagsUnderHighVelocity )
{
  initController();
  wireInterfaces();
  controller_->reference_interfaces_.assign( joints_.size(),
                                             std::numeric_limits<double>::quiet_NaN() );
  activateController();

  // Command high velocity (strong can handle it, weak cannot)
  setVelocityCommands( -2.0, -2.0 );
  runCycles( 200 ); // 4 seconds

  const double pos_strong = getPosition( 0 );
  const double pos_weak = getPosition( 1 );

  // Both should have moved in the negative direction
  EXPECT_LT( pos_strong, -1.0 ) << "Strong joint should have moved significantly";
  EXPECT_LT( pos_weak, -1.0 ) << "Weak joint should have moved";

  // Strong joint should have moved further (more torque/velocity headroom)
  EXPECT_LT( pos_strong, pos_weak )
      << "Strong joint should be further (more negative) than weak joint";
}

// Both joints brake to a stop independently
TEST_F( VelToPosIntegrationTest, IndependentBrakingAfterMovement )
{
  initController();
  wireInterfaces();
  controller_->reference_interfaces_.assign( joints_.size(),
                                             std::numeric_limits<double>::quiet_NaN() );
  activateController();

  // Move both for 1 second
  setVelocityCommands( 1.0, 1.0 );
  runCycles( 50 );

  EXPECT_EQ( controller_->move_states_[0], MoveState::MOVING );
  EXPECT_EQ( controller_->move_states_[1], MoveState::MOVING );

  // Stop
  setVelocityCommands( 0.0, 0.0 );
  runCycles( 50 ); // Enough cycles for braking + settling

  // Both should have stopped
  EXPECT_EQ( controller_->move_states_[0], MoveState::STOPPED );
  EXPECT_EQ( controller_->move_states_[1], MoveState::STOPPED );

  // Record positions, then run more cycles — positions should stay stable
  const double hold_pos0 = getPosition( 0 );
  const double hold_pos1 = getPosition( 1 );
  runCycles( 50 );

  EXPECT_NEAR( hold_pos0, getPosition( 0 ), 0.05 ) << "Strong joint should hold position";
  EXPECT_NEAR( hold_pos1, getPosition( 1 ), 0.05 ) << "Weak joint should hold position";
}

// Sync correction reduces drift between joints
TEST_F( VelToPosIntegrationTest, SyncCorrectionReducesDrift )
{
  // Run without sync
  initController( {}, 0.0 ); // No sync groups, kp_sync doesn't matter
  wireInterfaces();
  controller_->reference_interfaces_.assign( joints_.size(),
                                             std::numeric_limits<double>::quiet_NaN() );
  activateController();

  setVelocityCommands( -1.5, -1.5 );
  runCycles( 200 );

  const double drift_no_sync = std::abs( getPosition( 0 ) - getPosition( 1 ) );

  // Reset: new controller with sync enabled
  controller_.reset();
  rm_.reset();
  rm_ = std::make_unique<TestableResourceManager>( node_, std::string( URDF ) );
  activate_components( *rm_ );
  controller_ = std::make_shared<VelToPosController>();

  std::vector<std::string> sync_groups = { "front", "front" };
  initController( sync_groups, 1.0 );
  wireInterfaces();
  controller_->reference_interfaces_.assign( joints_.size(),
                                             std::numeric_limits<double>::quiet_NaN() );
  activateController();

  setVelocityCommands( -1.5, -1.5 );
  runCycles( 200 );

  const double drift_with_sync = std::abs( getPosition( 0 ) - getPosition( 1 ) );

  // With sync correction, drift should be smaller
  EXPECT_LT( drift_with_sync, drift_no_sync )
      << "Sync correction should reduce position drift (no_sync=" << drift_no_sync
      << ", with_sync=" << drift_with_sync << ")";
}

// ===========================================================================
// Entry point
// ===========================================================================

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  ::testing::InitGoogleTest( &argc, argv );
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
