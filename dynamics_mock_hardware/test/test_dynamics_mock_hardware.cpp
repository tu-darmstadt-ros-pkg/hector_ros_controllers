// Tests for the dynamics_mock_hardware pinocchio-based hardware interface.
//
// Uses the ResourceManager integration test pattern from ros2_control.
// Each test creates a URDF with the DynamicsMockHardware plugin, loads it
// into a ResourceManager, and drives the read/write loop directly.

#include <cmath>
#include <string>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

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

// ---------------------------------------------------------------------------
// Test constants
// ---------------------------------------------------------------------------

const auto TIME = rclcpp::Time( 0 );
const double PERIOD_SEC = 0.01;
const auto PERIOD = rclcpp::Duration::from_seconds( PERIOD_SEC );
const double DELTA = 1e-3; // tolerance for floating-point comparisons

// ---------------------------------------------------------------------------
// Minimal URDF fragments
// ---------------------------------------------------------------------------

// A single revolute pendulum hanging in -Z gravity.
// link1 has significant mass (1 kg) and a 0.5m offset so gravity creates torque.
// joint1 axis is Y (horizontal), so the pendulum swings in the XZ plane.
const char *const URDF_HEAD = R"(
<?xml version="1.0" encoding="utf-8"?>
<robot name="TestPendulum">
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
  <joint name="joint1" type="revolute">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 1 0"/>
    <limit effort="100.0" lower="-3.14159" upper="3.14159" velocity="10.0"/>
  </joint>
  <link name="link1">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.01"/>
    </inertial>
  </link>
)";

const char *const URDF_TAIL = R"(
</robot>
)";

// Hardware block: full interfaces, zero gravity, large integration_dt for speed
std::string makeHardwareBlock( double gravity_z = 0.0, double max_torque = 100.0,
                               double position_kp = 100.0, double position_kd = 20.0,
                               double velocity_kp = 10.0, double integration_dt = 0.001,
                               double initial_pos = 0.0 )
{
  return R"(
  <ros2_control name="DynMockSystem" type="system">
    <hardware>
      <plugin>dynamics_mock_hardware/DynamicsMockHardware</plugin>
      <param name="gravity_x">0.0</param>
      <param name="gravity_y">0.0</param>
      <param name="gravity_z">)" +
         std::to_string( gravity_z ) + R"(</param>
      <param name="position_kp">)" +
         std::to_string( position_kp ) + R"(</param>
      <param name="position_kd">)" +
         std::to_string( position_kd ) + R"(</param>
      <param name="velocity_kp">)" +
         std::to_string( velocity_kp ) + R"(</param>
      <param name="integration_dt">)" +
         std::to_string( integration_dt ) + R"(</param>
    </hardware>
    <joint name="joint1">
      <param name="max_torque">)" +
         std::to_string( max_torque ) + R"(</param>
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <command_interface name="acceleration"/>
      <command_interface name="effort"/>
      <state_interface name="position">
        <param name="initial_value">)" +
         std::to_string( initial_pos ) + R"(</param>
      </state_interface>
      <state_interface name="velocity"/>
      <state_interface name="acceleration"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>
)";
}

// ---------------------------------------------------------------------------
// Test helper: ResourceManager wrapper
// ---------------------------------------------------------------------------

class TestableResourceManager : public hardware_interface::ResourceManager
{
public:
  explicit TestableResourceManager( rclcpp::Node::SharedPtr node, const std::string &urdf,
                                    bool activate_all = false, unsigned int cm_update_rate = 100 )
      : hardware_interface::ResourceManager( urdf, node->get_node_clock_interface(),
                                             node->get_node_logging_interface(), activate_all,
                                             cm_update_rate )
  {
  }
};

void set_components_state( TestableResourceManager &rm, const std::vector<std::string> &components,
                           uint8_t state_id, const std::string &state_name )
{
  for ( const auto &component : components ) {
    rclcpp_lifecycle::State state( state_id, state_name );
    rm.set_component_state( component, state );
  }
}

void activate_components( TestableResourceManager &rm,
                          const std::vector<std::string> &components = { "DynMockSystem" } )
{
  set_components_state( rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                        hardware_interface::lifecycle_state_names::ACTIVE );
}

// Run N read/write cycles
void run_cycles( TestableResourceManager &rm, int n,
                 rclcpp::Duration period = rclcpp::Duration::from_seconds( PERIOD_SEC ) )
{
  for ( int i = 0; i < n; ++i ) {
    ASSERT_EQ( rm.read( TIME, period ).result, hardware_interface::return_type::OK );
    ASSERT_EQ( rm.write( TIME, period ).result, hardware_interface::return_type::OK );
  }
}

} // namespace

// ===========================================================================
// Test fixture
// ===========================================================================

class DynamicsMockHardwareTest : public ::testing::Test
{
protected:
  void SetUp() override { node_ = std::make_shared<rclcpp::Node>( "test_dynamics_mock" ); }
  rclcpp::Node::SharedPtr node_;
};

// ===========================================================================
// Test: plugin loads and basic lifecycle works
// ===========================================================================

TEST_F( DynamicsMockHardwareTest, LoadAndActivate )
{
  const std::string urdf = std::string( URDF_HEAD ) + makeHardwareBlock() + std::string( URDF_TAIL );
  ASSERT_NO_THROW( TestableResourceManager rm( node_, urdf ) );

  TestableResourceManager rm( node_, urdf );
  activate_components( rm );

  EXPECT_TRUE( rm.state_interface_exists( "joint1/position" ) );
  EXPECT_TRUE( rm.state_interface_exists( "joint1/velocity" ) );
  EXPECT_TRUE( rm.state_interface_exists( "joint1/acceleration" ) );
  EXPECT_TRUE( rm.state_interface_exists( "joint1/effort" ) );
  EXPECT_TRUE( rm.command_interface_exists( "joint1/position" ) );
  EXPECT_TRUE( rm.command_interface_exists( "joint1/velocity" ) );
  EXPECT_TRUE( rm.command_interface_exists( "joint1/effort" ) );
}

// ===========================================================================
// Test: initial values are correct
// ===========================================================================

TEST_F( DynamicsMockHardwareTest, InitialValues )
{
  const double init_pos = 0.5;
  const std::string urdf = std::string( URDF_HEAD ) +
                           makeHardwareBlock( 0.0, 100.0, 100.0, 20.0, 10.0, 0.001, init_pos ) +
                           std::string( URDF_TAIL );
  TestableResourceManager rm( node_, urdf );
  activate_components( rm );

  auto pos_s = rm.claim_state_interface( "joint1/position" );
  auto vel_s = rm.claim_state_interface( "joint1/velocity" );

  EXPECT_NEAR( init_pos, pos_s.get_optional().value(), DELTA );
  EXPECT_NEAR( 0.0, vel_s.get_optional().value(), DELTA );
}

// ===========================================================================
// Test: zero gravity, no command -> joint stays still (position mode holds)
// ===========================================================================

TEST_F( DynamicsMockHardwareTest, PositionMode_HoldsPosition_NoGravity )
{
  const std::string urdf =
      std::string( URDF_HEAD ) + makeHardwareBlock( 0.0 ) + std::string( URDF_TAIL );
  TestableResourceManager rm( node_, urdf );
  activate_components( rm );

  auto pos_s = rm.claim_state_interface( "joint1/position" );
  auto vel_s = rm.claim_state_interface( "joint1/velocity" );

  // on_activate seeds position command = current state = 0.0
  // PD should hold it at 0 with zero gravity
  run_cycles( rm, 100 );

  EXPECT_NEAR( 0.0, pos_s.get_optional().value(), DELTA );
  EXPECT_NEAR( 0.0, vel_s.get_optional().value(), DELTA );
}

// ===========================================================================
// Test: position mode holds position even with gravity
// ===========================================================================

TEST_F( DynamicsMockHardwareTest, PositionMode_HoldsPosition_Gravity )
{
  // With gravity and sufficient torque, the PD controller should hold position.
  // Start at 0.5 rad where gravity torque = m*g*L*sin(0.5) ~= 1*9.81*0.5*0.479 = 2.35 Nm
  // max_torque=100 >> 2.35 so the PD controller can easily counteract gravity.
  const double init_pos = 0.5;
  const std::string urdf = std::string( URDF_HEAD ) +
                           makeHardwareBlock( -9.81, 100.0, 100.0, 20.0, 10.0, 0.001, init_pos ) +
                           std::string( URDF_TAIL );
  TestableResourceManager rm( node_, urdf );
  activate_components( rm );

  auto pos_s = rm.claim_state_interface( "joint1/position" );
  auto vel_s = rm.claim_state_interface( "joint1/velocity" );

  // on_activate seeds position command = current state = init_pos
  // PD should hold it at init_pos despite gravity
  run_cycles( rm, 200 );

  EXPECT_NEAR( init_pos, pos_s.get_optional().value(), 0.05 );
  EXPECT_NEAR( 0.0, vel_s.get_optional().value(), 0.1 );
}

// ===========================================================================
// Test: position mode tracks a commanded position
// ===========================================================================

TEST_F( DynamicsMockHardwareTest, PositionMode_TracksTarget )
{
  // No gravity, generous torque, small integration_dt for accuracy
  const std::string urdf = std::string( URDF_HEAD ) +
                           makeHardwareBlock( 0.0, 100.0, 100.0, 20.0, 10.0, 0.0005 ) +
                           std::string( URDF_TAIL );
  TestableResourceManager rm( node_, urdf );
  activate_components( rm );

  auto pos_s = rm.claim_state_interface( "joint1/position" );
  auto vel_s = rm.claim_state_interface( "joint1/velocity" );
  auto pos_c = rm.claim_command_interface( "joint1/position" );

  const double target = 0.5;
  ASSERT_TRUE( pos_c.set_value( target ) );

  // Run for 2 simulated seconds (200 * 0.01s) to let PD settle
  run_cycles( rm, 200 );

  EXPECT_NEAR( target, pos_s.get_optional().value(), 0.05 );
  EXPECT_NEAR( 0.0, vel_s.get_optional().value(), 0.1 );
}

// ===========================================================================
// Test: position mode with torque limit prevents reaching target
// ===========================================================================

TEST_F( DynamicsMockHardwareTest, PositionMode_TorqueLimitSaturates )
{
  // Start at 1.0 rad (away from equilibrium) with gravity and tiny max torque.
  // Gravitational torque = m*g*L*sin(1.0) ~= 1*9.81*0.5*0.841 = 4.13 Nm >> 0.1 Nm
  // The PD controller is saturated at 0.1 Nm and cannot hold position.
  const double init_pos = 1.0;
  const std::string urdf = std::string( URDF_HEAD ) +
                           makeHardwareBlock( -9.81, 0.1, 100.0, 20.0, 10.0, 0.001, init_pos ) +
                           std::string( URDF_TAIL );
  TestableResourceManager rm( node_, urdf );
  activate_components( rm );

  auto pos_s = rm.claim_state_interface( "joint1/position" );
  auto eff_s = rm.claim_state_interface( "joint1/effort" );
  auto pos_c = rm.claim_command_interface( "joint1/position" );

  ASSERT_TRUE( pos_c.set_value( init_pos ) ); // try to hold at initial position

  // Run for 2 simulated seconds
  run_cycles( rm, 200 );

  // The joint should have drifted significantly from the target due to gravity
  const double pos = pos_s.get_optional().value();
  EXPECT_GT( std::abs( pos - init_pos ), 0.1 )
      << "Joint should drift under gravity with saturated torque";

  // Effort should be clamped at max_torque
  const double eff = eff_s.get_optional().value();
  EXPECT_LE( std::abs( eff ), 0.1 + DELTA );
}

// ===========================================================================
// Test: effort mode - direct torque command
// ===========================================================================

TEST_F( DynamicsMockHardwareTest, EffortMode_AppliedTorqueAccelerates )
{
  // Zero gravity so we can reason about torque -> acceleration cleanly
  const std::string urdf =
      std::string( URDF_HEAD ) + makeHardwareBlock( 0.0 ) + std::string( URDF_TAIL );
  TestableResourceManager rm( node_, urdf );
  activate_components( rm );

  // Switch to effort mode
  rm.prepare_command_mode_switch( { "joint1/effort" }, { "joint1/position" } );
  rm.perform_command_mode_switch( { "joint1/effort" }, { "joint1/position" } );

  auto pos_s = rm.claim_state_interface( "joint1/position" );
  auto vel_s = rm.claim_state_interface( "joint1/velocity" );
  auto acc_s = rm.claim_state_interface( "joint1/acceleration" );
  auto eff_s = rm.claim_state_interface( "joint1/effort" );
  auto eff_c = rm.claim_command_interface( "joint1/effort" );

  // Apply constant torque
  const double applied_torque = 1.0;
  ASSERT_TRUE( eff_c.set_value( applied_torque ) );

  run_cycles( rm, 50 );

  // With positive torque and zero gravity, joint should have moved in positive direction
  EXPECT_GT( pos_s.get_optional().value(), 0.0 ) << "Joint should move under applied torque";
  EXPECT_GT( vel_s.get_optional().value(), 0.0 ) << "Velocity should be positive";

  // Reported effort should match applied torque (within max_torque)
  EXPECT_NEAR( applied_torque, eff_s.get_optional().value(), DELTA );
}

// ===========================================================================
// Test: effort mode - torque is clamped to max_torque
// ===========================================================================

TEST_F( DynamicsMockHardwareTest, EffortMode_TorqueClamped )
{
  const double max_torque = 2.0;
  const std::string urdf =
      std::string( URDF_HEAD ) + makeHardwareBlock( 0.0, max_torque ) + std::string( URDF_TAIL );
  TestableResourceManager rm( node_, urdf );
  activate_components( rm );

  rm.prepare_command_mode_switch( { "joint1/effort" }, { "joint1/position" } );
  rm.perform_command_mode_switch( { "joint1/effort" }, { "joint1/position" } );

  auto eff_s = rm.claim_state_interface( "joint1/effort" );
  auto eff_c = rm.claim_command_interface( "joint1/effort" );

  // Command torque much larger than max
  ASSERT_TRUE( eff_c.set_value( 50.0 ) );
  run_cycles( rm, 1 );

  EXPECT_NEAR( max_torque, eff_s.get_optional().value(), DELTA );

  // Negative direction
  ASSERT_TRUE( eff_c.set_value( -50.0 ) );
  run_cycles( rm, 1 );

  EXPECT_NEAR( -max_torque, eff_s.get_optional().value(), DELTA );
}

// ===========================================================================
// Test: effort mode - zero torque with zero gravity stays still
// ===========================================================================

TEST_F( DynamicsMockHardwareTest, EffortMode_ZeroTorqueNoGravity_StaysStill )
{
  const std::string urdf =
      std::string( URDF_HEAD ) + makeHardwareBlock( 0.0 ) + std::string( URDF_TAIL );
  TestableResourceManager rm( node_, urdf );
  activate_components( rm );

  rm.prepare_command_mode_switch( { "joint1/effort" }, { "joint1/position" } );
  rm.perform_command_mode_switch( { "joint1/effort" }, { "joint1/position" } );

  auto pos_s = rm.claim_state_interface( "joint1/position" );
  auto vel_s = rm.claim_state_interface( "joint1/velocity" );
  auto eff_c = rm.claim_command_interface( "joint1/effort" );

  ASSERT_TRUE( eff_c.set_value( 0.0 ) );
  run_cycles( rm, 100 );

  EXPECT_NEAR( 0.0, pos_s.get_optional().value(), DELTA );
  EXPECT_NEAR( 0.0, vel_s.get_optional().value(), DELTA );
}

// ===========================================================================
// Test: velocity mode tracks a commanded velocity
// ===========================================================================

TEST_F( DynamicsMockHardwareTest, VelocityMode_TracksTarget )
{
  const std::string urdf = std::string( URDF_HEAD ) +
                           makeHardwareBlock( 0.0, 100.0, 100.0, 20.0, 50.0, 0.001 ) +
                           std::string( URDF_TAIL );
  TestableResourceManager rm( node_, urdf );
  activate_components( rm );

  rm.prepare_command_mode_switch( { "joint1/velocity" }, { "joint1/position" } );
  rm.perform_command_mode_switch( { "joint1/velocity" }, { "joint1/position" } );

  auto pos_s = rm.claim_state_interface( "joint1/position" );
  auto vel_s = rm.claim_state_interface( "joint1/velocity" );
  auto vel_c = rm.claim_command_interface( "joint1/velocity" );

  const double target_vel = 1.0;
  ASSERT_TRUE( vel_c.set_value( target_vel ) );

  // Run for 1 simulated second to let P controller settle
  run_cycles( rm, 100 );

  EXPECT_NEAR( target_vel, vel_s.get_optional().value(), 0.2 );
  // Position should have moved roughly target_vel * time
  EXPECT_GT( pos_s.get_optional().value(), 0.3 );
}

// ===========================================================================
// Test: velocity mode - zero command stops the joint
// ===========================================================================

TEST_F( DynamicsMockHardwareTest, VelocityMode_ZeroCommand_StopsJoint )
{
  const std::string urdf = std::string( URDF_HEAD ) +
                           makeHardwareBlock( 0.0, 100.0, 100.0, 20.0, 50.0, 0.001 ) +
                           std::string( URDF_TAIL );
  TestableResourceManager rm( node_, urdf );
  activate_components( rm );

  rm.prepare_command_mode_switch( { "joint1/velocity" }, { "joint1/position" } );
  rm.perform_command_mode_switch( { "joint1/velocity" }, { "joint1/position" } );

  auto vel_s = rm.claim_state_interface( "joint1/velocity" );
  auto vel_c = rm.claim_command_interface( "joint1/velocity" );

  // First accelerate
  ASSERT_TRUE( vel_c.set_value( 2.0 ) );
  run_cycles( rm, 50 );

  EXPECT_GT( std::abs( vel_s.get_optional().value() ), 0.5 );

  // Now command zero -> should decelerate
  ASSERT_TRUE( vel_c.set_value( 0.0 ) );
  run_cycles( rm, 200 );

  EXPECT_NEAR( 0.0, vel_s.get_optional().value(), 0.1 );
}

// ===========================================================================
// Test: effort mode - gravity makes pendulum fall
// ===========================================================================

TEST_F( DynamicsMockHardwareTest, EffortMode_GravityPendulumFalls )
{
  // Start at position 0 (pendulum pointing down along -Z). A small offset
  // of 0.1 rad should let gravity pull it further.
  const std::string urdf = std::string( URDF_HEAD ) +
                           makeHardwareBlock( -9.81, 100.0, 100.0, 20.0, 10.0, 0.001, 0.1 ) +
                           std::string( URDF_TAIL );
  TestableResourceManager rm( node_, urdf );
  activate_components( rm );

  rm.prepare_command_mode_switch( { "joint1/effort" }, { "joint1/position" } );
  rm.perform_command_mode_switch( { "joint1/effort" }, { "joint1/position" } );

  auto pos_s = rm.claim_state_interface( "joint1/position" );
  auto vel_s = rm.claim_state_interface( "joint1/velocity" );
  auto eff_c = rm.claim_command_interface( "joint1/effort" );

  ASSERT_TRUE( eff_c.set_value( 0.0 ) ); // no motor torque

  const double initial_pos = pos_s.get_optional().value();

  run_cycles( rm, 100 );

  // Gravity should have changed the position
  EXPECT_NE( initial_pos, pos_s.get_optional().value() ) << "Pendulum should move under gravity";
  // Velocity should be non-zero
  EXPECT_NE( 0.0, vel_s.get_optional().value() ) << "Pendulum should have non-zero velocity";
}

// ===========================================================================
// Test: position limits are enforced
// ===========================================================================

TEST_F( DynamicsMockHardwareTest, PositionLimits_Enforced )
{
  // URDF limits: [-3.14159, 3.14159]
  // Apply huge effort mode torque to push past limit
  const std::string urdf =
      std::string( URDF_HEAD ) + makeHardwareBlock( 0.0 ) + std::string( URDF_TAIL );
  TestableResourceManager rm( node_, urdf );
  activate_components( rm );

  rm.prepare_command_mode_switch( { "joint1/effort" }, { "joint1/position" } );
  rm.perform_command_mode_switch( { "joint1/effort" }, { "joint1/position" } );

  auto pos_s = rm.claim_state_interface( "joint1/position" );
  auto vel_s = rm.claim_state_interface( "joint1/velocity" );
  auto eff_c = rm.claim_command_interface( "joint1/effort" );

  // Push hard in positive direction
  ASSERT_TRUE( eff_c.set_value( 50.0 ) );
  run_cycles( rm, 500 );

  // Should be clamped at the upper limit
  EXPECT_LE( pos_s.get_optional().value(), 3.14159 + DELTA );
  // Velocity should be zeroed at the limit (bounce prevention)
  EXPECT_LE( vel_s.get_optional().value(), DELTA );
}

// ===========================================================================
// Test: mode switching rejects multiple command interfaces per joint
// ===========================================================================

TEST_F( DynamicsMockHardwareTest, ModeSwitching_RejectsMultipleInterfaces )
{
  const std::string urdf =
      std::string( URDF_HEAD ) + makeHardwareBlock( 0.0 ) + std::string( URDF_TAIL );
  TestableResourceManager rm( node_, urdf );
  activate_components( rm );

  EXPECT_FALSE( rm.prepare_command_mode_switch( { "joint1/position", "joint1/velocity" }, {} ) );
}

// ===========================================================================
// Test: different torque limits produce different behavior (sync drift)
// ===========================================================================

// This test uses a 2-joint URDF with different max torques to verify
// the core use case: asymmetric motor torques cause position drift.

const char *const URDF_HEAD_2DOF = R"(
<?xml version="1.0" encoding="utf-8"?>
<robot name="TwoJointRobot">
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
  <joint name="joint_strong" type="revolute">
    <origin rpy="0 0 0" xyz="0 0.5 0"/>
    <parent link="base_link"/>
    <child link="link_strong"/>
    <axis xyz="0 1 0"/>
    <limit effort="100.0" lower="-3.14159" upper="3.14159" velocity="10.0"/>
  </joint>
  <link name="link_strong">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="joint_weak" type="revolute">
    <origin rpy="0 0 0" xyz="0 -0.5 0"/>
    <parent link="base_link"/>
    <child link="link_weak"/>
    <axis xyz="0 1 0"/>
    <limit effort="100.0" lower="-3.14159" upper="3.14159" velocity="10.0"/>
  </joint>
  <link name="link_weak">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.01"/>
    </inertial>
  </link>
)";

TEST_F( DynamicsMockHardwareTest, AsymmetricTorqueLimits_CausesDrift )
{
  // Strong motor: high max torque -> tracks well
  // Weak motor: low max torque -> saturates -> drifts under gravity
  std::string hw_block = R"(
  <ros2_control name="DynMockSystem" type="system">
    <hardware>
      <plugin>dynamics_mock_hardware/DynamicsMockHardware</plugin>
      <param name="gravity_z">-9.81</param>
      <param name="position_kp">100.0</param>
      <param name="position_kd">20.0</param>
      <param name="integration_dt">0.001</param>
    </hardware>
    <joint name="joint_strong">
      <param name="max_torque">50.0</param>
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0.3</param></state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="joint_weak">
      <param name="max_torque">0.5</param>
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0.3</param></state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>
)";

  const std::string urdf = std::string( URDF_HEAD_2DOF ) + hw_block + std::string( URDF_TAIL );
  TestableResourceManager rm( node_, urdf );
  activate_components( rm );

  auto pos_strong = rm.claim_state_interface( "joint_strong/position" );
  auto pos_weak = rm.claim_state_interface( "joint_weak/position" );
  auto eff_strong = rm.claim_state_interface( "joint_strong/effort" );
  auto eff_weak = rm.claim_state_interface( "joint_weak/effort" );
  auto cmd_strong = rm.claim_command_interface( "joint_strong/position" );
  auto cmd_weak = rm.claim_command_interface( "joint_weak/position" );

  // Both commanded to hold at 0.3 rad
  ASSERT_TRUE( cmd_strong.set_value( 0.3 ) );
  ASSERT_TRUE( cmd_weak.set_value( 0.3 ) );

  // Simulate for 2 seconds
  run_cycles( rm, 200 );

  const double strong_pos = pos_strong.get_optional().value();
  const double weak_pos = pos_weak.get_optional().value();
  const double strong_eff = std::abs( eff_strong.get_optional().value() );
  const double weak_eff = std::abs( eff_weak.get_optional().value() );

  // Strong motor should hold position well
  EXPECT_NEAR( 0.3, strong_pos, 0.05 ) << "Strong motor should hold position";

  // Weak motor should have drifted (gravity torque ~= 1*9.81*0.5*sin(0.3) ~= 1.45 Nm >> 0.5 Nm)
  EXPECT_GT( std::abs( weak_pos - 0.3 ), 0.1 )
      << "Weak motor should drift significantly from target";

  // The two joints should have different positions (drift)
  EXPECT_GT( std::abs( strong_pos - weak_pos ), 0.1 )
      << "Asymmetric torque limits should cause position drift between joints";

  // Weak motor effort should be saturated at its limit
  EXPECT_NEAR( 0.5, weak_eff, DELTA ) << "Weak motor should be at torque limit";
  // Strong motor should not be saturated
  EXPECT_LT( strong_eff, 50.0 - 1.0 ) << "Strong motor should not be saturated";
}

// ===========================================================================
// Test: acceleration mode
// ===========================================================================

TEST_F( DynamicsMockHardwareTest, AccelerationMode_AppliesDesiredAcceleration )
{
  const std::string urdf =
      std::string( URDF_HEAD ) + makeHardwareBlock( 0.0, 1000.0 ) + std::string( URDF_TAIL );
  TestableResourceManager rm( node_, urdf );
  activate_components( rm );

  rm.prepare_command_mode_switch( { "joint1/acceleration" }, { "joint1/position" } );
  rm.perform_command_mode_switch( { "joint1/acceleration" }, { "joint1/position" } );

  auto pos_s = rm.claim_state_interface( "joint1/position" );
  auto vel_s = rm.claim_state_interface( "joint1/velocity" );
  auto acc_c = rm.claim_command_interface( "joint1/acceleration" );

  // Constant acceleration
  const double desired_accel = 2.0;
  ASSERT_TRUE( acc_c.set_value( desired_accel ) );

  const int N = 50;
  const double t = N * PERIOD_SEC; // 0.5 seconds
  run_cycles( rm, N );

  // Expected: v = a*t = 2.0 * 0.5 = 1.0, pos = 0.5*a*t^2 = 0.25
  // Due to dynamics coupling, results may not be exact, but should be close
  // with high max_torque (RNEA can generate whatever it needs)
  EXPECT_NEAR( desired_accel * t, vel_s.get_optional().value(), 0.3 );
  EXPECT_GT( pos_s.get_optional().value(), 0.1 );
}

// ===========================================================================
// Test: mimic joint tracks its reference joint
// ===========================================================================

const char *const URDF_HEAD_MIMIC = R"(
<?xml version="1.0" encoding="utf-8"?>
<robot name="MimicTestRobot">
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
  <joint name="servo_joint" type="revolute">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="base_link"/>
    <child link="servo_link"/>
    <axis xyz="0 1 0"/>
    <limit effort="100.0" lower="0.0" upper="1.1" velocity="5.0"/>
  </joint>
  <link name="servo_link">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 -0.1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="mimic_joint" type="revolute">
    <origin rpy="0 0 0" xyz="0 0.1 0"/>
    <parent link="base_link"/>
    <child link="mimic_link"/>
    <axis xyz="0 1 0"/>
    <limit effort="10.0" lower="0.0" upper="1.1" velocity="5.0"/>
    <mimic joint="servo_joint" multiplier="0.909" offset="0.0"/>
  </joint>
  <link name="mimic_link">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 -0.05"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
)";

TEST_F( DynamicsMockHardwareTest, MimicJoint_TracksReference )
{
  // Servo joint has command + state interfaces; mimic joint has state only
  std::string hw_block = R"(
  <ros2_control name="DynMockSystem" type="system">
    <hardware>
      <plugin>dynamics_mock_hardware/DynamicsMockHardware</plugin>
      <param name="gravity_z">0.0</param>
      <param name="position_kp">100.0</param>
      <param name="position_kd">20.0</param>
      <param name="integration_dt">0.001</param>
    </hardware>
    <joint name="servo_joint">
      <param name="max_torque">100.0</param>
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="mimic_joint">
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>
)";

  const std::string urdf = std::string( URDF_HEAD_MIMIC ) + hw_block + std::string( URDF_TAIL );
  TestableResourceManager rm( node_, urdf );
  activate_components( rm );

  // Verify interfaces exist
  EXPECT_TRUE( rm.state_interface_exists( "servo_joint/position" ) );
  EXPECT_TRUE( rm.state_interface_exists( "mimic_joint/position" ) );
  EXPECT_TRUE( rm.state_interface_exists( "mimic_joint/velocity" ) );
  EXPECT_TRUE( rm.command_interface_exists( "servo_joint/position" ) );
  EXPECT_FALSE( rm.command_interface_exists( "mimic_joint/position" ) );

  auto servo_pos_s = rm.claim_state_interface( "servo_joint/position" );
  auto servo_vel_s = rm.claim_state_interface( "servo_joint/velocity" );
  auto mimic_pos_s = rm.claim_state_interface( "mimic_joint/position" );
  auto mimic_vel_s = rm.claim_state_interface( "mimic_joint/velocity" );
  auto servo_pos_c = rm.claim_command_interface( "servo_joint/position" );

  // Command servo to a target position
  const double target = 0.8;
  ASSERT_TRUE( servo_pos_c.set_value( target ) );

  // Run for 2 simulated seconds to let PD settle
  run_cycles( rm, 200 );

  const double servo_pos = servo_pos_s.get_optional().value();
  const double mimic_pos = mimic_pos_s.get_optional().value();
  const double servo_vel = servo_vel_s.get_optional().value();
  const double mimic_vel = mimic_vel_s.get_optional().value();

  // Servo should have reached target
  EXPECT_NEAR( target, servo_pos, 0.05 ) << "Servo should reach target position";
  EXPECT_NEAR( 0.0, servo_vel, 0.1 ) << "Servo should have settled";

  // Mimic should track: mimic_pos = 0.909 * servo_pos + 0.0
  EXPECT_NEAR( 0.909 * servo_pos, mimic_pos, 0.05 )
      << "Mimic joint should track reference with multiplier=0.909";

  // Mimic velocity should track: mimic_vel = 0.909 * servo_vel
  EXPECT_NEAR( 0.909 * servo_vel, mimic_vel, 0.1 )
      << "Mimic velocity should track reference velocity";
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
