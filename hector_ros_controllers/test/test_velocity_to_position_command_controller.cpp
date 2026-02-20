#include "test_helpers.hpp"

using VelToPosController =
    velocity_to_position_command_controller::VelocityToPositionCommandController;
using MoveState = velocity_to_position_command_controller::MoveState;

// ============================================================================
// Test Fixture
// ============================================================================

class VelocityToPositionCommandControllerTest : public ::testing::Test
{
protected:
  static constexpr unsigned int kUpdateRate = 100;

  std::vector<std::string> joints_{ "joint1", "joint2", "joint3" };

  std::shared_ptr<VelToPosController> controller_;

  // Hardware interface backing storage
  // State interfaces: interleaved [pos0, vel0, pos1, vel1, ...]
  std::vector<double> hw_cmd_values_;
  std::vector<double> hw_state_values_;
  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> cmd_ifaces_;
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> state_ifaces_;

  void SetUp() override { controller_ = std::make_shared<VelToPosController>(); }

  void TearDown() override
  {
    controller_.reset();
    rtest::StaticMocksRegistry::instance().reset();
  }

  void initController( const std::vector<std::string> &sync_groups = {},
                       const std::string &passthrough = "" )
  {
    const auto urdf = hector_test::loadUrdfFile( "test_robot.urdf" );

    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "test_vel_to_pos";
    params.robot_description = urdf;
    params.update_rate = kUpdateRate;
    params.controller_manager_update_rate = kUpdateRate;
    params.node_namespace = "";

    std::vector<rclcpp::Parameter> overrides = {
        rclcpp::Parameter( "joints", joints_ ),
        rclcpp::Parameter( "kp", 2.0 ),
        rclcpp::Parameter( "kd", 0.1 ),
        rclcpp::Parameter( "kp_sync", 1.0 ),
        rclcpp::Parameter( "stopping_velocity_threshold", 0.005 ),
        rclcpp::Parameter( "passthrough_controller", passthrough ),
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
  }

  void configureController()
  {
    rclcpp_lifecycle::State unconfigured( lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                          "unconfigured" );
    auto cb = controller_->on_configure( unconfigured );
    ASSERT_EQ( cb, controller_interface::CallbackReturn::SUCCESS );

    // on_export_reference_interfaces is normally called by CM; we must size reference_interfaces_
    controller_->reference_interfaces_.assign( joints_.size(),
                                               std::numeric_limits<double>::quiet_NaN() );
  }

  void setupHardwareInterfaces()
  {
    hw_cmd_values_.assign( joints_.size(), 0.0 );
    // State interfaces: [pos0, vel0, pos1, vel1, pos2, vel2]
    hw_state_values_.assign( joints_.size() * 2, 0.0 );

    cmd_ifaces_.clear();
    state_ifaces_.clear();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    for ( size_t i = 0; i < joints_.size(); ++i ) {
      cmd_ifaces_.push_back( std::make_shared<hardware_interface::CommandInterface>(
          joints_[i], "position", &hw_cmd_values_[i] ) );
      state_ifaces_.push_back( std::make_shared<hardware_interface::StateInterface>(
          joints_[i], "position", &hw_state_values_[2 * i] ) );
      state_ifaces_.push_back( std::make_shared<hardware_interface::StateInterface>(
          joints_[i], "velocity", &hw_state_values_[2 * i + 1] ) );
    }
#pragma GCC diagnostic pop

    controller_->command_interfaces_.clear();
    controller_->state_interfaces_.clear();

    for ( auto &ci : cmd_ifaces_ ) {
      controller_->command_interfaces_.emplace_back( ci, []() { } );
    }
    for ( auto &si : state_ifaces_ ) { controller_->state_interfaces_.emplace_back( si ); }
  }

  void activateController()
  {
    rclcpp_lifecycle::State inactive( lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                      "inactive" );
    auto cb = controller_->on_activate( inactive );
    ASSERT_EQ( cb, controller_interface::CallbackReturn::SUCCESS );
  }

  void deactivateController()
  {
    rclcpp_lifecycle::State active( lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active" );
    auto cb = controller_->on_deactivate( active );
    ASSERT_EQ( cb, controller_interface::CallbackReturn::SUCCESS );
  }

  controller_interface::return_type callUpdate()
  {
    rclcpp::Time now( 0, 0, RCL_ROS_TIME );
    rclcpp::Duration period( std::chrono::milliseconds( 10 ) ); // dt = 0.01s
    return controller_->update_and_write_commands( now, period );
  }

  // Set position state for a joint by index
  void setPosition( size_t idx, double value ) { hw_state_values_[2 * idx] = value; }

  // Set velocity state for a joint by index
  void setVelocity( size_t idx, double value ) { hw_state_values_[2 * idx + 1] = value; }
};

// ============================================================================
// Lifecycle & Init Tests
// ============================================================================

// Verify controller initializes and parameter listener is created
TEST_F( VelocityToPositionCommandControllerTest, OnInitSucceeds )
{
  initController();
  EXPECT_TRUE( controller_->param_listener_ != nullptr );
}

// Verify configure populates joints and PID gains from parameters
TEST_F( VelocityToPositionCommandControllerTest, OnConfigureSucceeds )
{
  initController();
  configureController();
  EXPECT_EQ( controller_->joints_.size(), 3u );
  EXPECT_DOUBLE_EQ( controller_->kp_, 2.0 );
  EXPECT_DOUBLE_EQ( controller_->kd_, 0.1 );
  EXPECT_DOUBLE_EQ( controller_->kp_sync_, 1.0 );
}

// Verify configure fails when no joints are specified
TEST_F( VelocityToPositionCommandControllerTest, OnConfigureFailsEmptyJoints )
{
  joints_ = {};
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "test_ctrl";
  params.robot_description = hector_test::loadUrdfFile( "test_robot.urdf" );
  params.update_rate = kUpdateRate;
  params.controller_manager_update_rate = kUpdateRate;
  rclcpp::NodeOptions opts;
  opts.parameter_overrides( {
      rclcpp::Parameter( "joints", std::vector<std::string>{} ),
      rclcpp::Parameter( "passthrough_controller", std::string( "" ) ),
      rclcpp::Parameter( "e_stop_topic", std::string( "estop" ) ),
      rclcpp::Parameter( "synchronous_groups", std::vector<std::string>{} ),
  } );
  params.node_options = opts;
  controller_->init( params );

  rclcpp_lifecycle::State unconfigured( lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                        "unconfigured" );
  auto cb = controller_->on_configure( unconfigured );
  EXPECT_EQ( cb, controller_interface::CallbackReturn::ERROR );
}

// ============================================================================
// Basic Position Integration Tests
// ============================================================================

// Verify position command = desired_pos + vel_p when vel_cmd=1.0, vel_actual=0
TEST_F( VelocityToPositionCommandControllerTest, BasicPositionIntegration )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  activateController();

  // All joints at position 0, velocity 0
  // Command velocity = 1.0 rad/s on joint 0
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  auto ret = callUpdate();
  EXPECT_EQ( ret, controller_interface::return_type::OK );

  // Expected: desired_pos = 0 + 1.0 * 0.01 = 0.01
  // vel_p = 2.0 * (1.0 - 0.0) * 0.01 = 0.02
  // vel_d = 0.1 * (0.0 - 0.0) = 0 (no acceleration)
  // pos_cmd = 0.01 + 0.02 - 0 = 0.03
  EXPECT_NEAR( hw_cmd_values_[0], 0.03, 1e-9 );
}

// Verify negative velocity integrates in the correct direction
TEST_F( VelocityToPositionCommandControllerTest, NegativeVelocityIntegration )
{
  initController();
  configureController();
  setupHardwareInterfaces();

  // Start at position 1.0
  setPosition( 0, 1.0 );
  activateController();

  controller_->reference_interfaces_[0] = -0.5;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  auto ret = callUpdate();
  EXPECT_EQ( ret, controller_interface::return_type::OK );

  // desired_pos = 1.0 + (-0.5) * 0.01 = 0.995
  // vel_p = 2.0 * (-0.5 - 0.0) * 0.01 = -0.01
  // vel_d = 0 (no acceleration)
  // pos_cmd = 0.995 - 0.01 = 0.985
  EXPECT_NEAR( hw_cmd_values_[0], 0.985, 1e-9 );
}

// ============================================================================
// Velocity Tracking P-Term Tests
// ============================================================================

// Verify P-term correction when actual velocity lags behind commanded velocity
TEST_F( VelocityToPositionCommandControllerTest, VelocityTrackingPTerm )
{
  initController();
  configureController();
  setupHardwareInterfaces();

  // Actual velocity is 0.5, command is 1.0 -> velocity error drives P correction
  setVelocity( 0, 0.5 );
  activateController();

  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  auto ret = callUpdate();
  EXPECT_EQ( ret, controller_interface::return_type::OK );

  // desired_pos = 0 + 1.0 * 0.01 = 0.01
  // vel_p = 2.0 * (1.0 - 0.5) * 0.01 = 0.01
  // vel_d = 0.1 * 0.5 * 0.01 = 0.0005 (damping proportional to actual velocity)
  // pos_cmd = 0.01 + 0.01 - 0.0005 = 0.0195
  EXPECT_NEAR( hw_cmd_values_[0], 0.0195, 1e-9 );
}

// ============================================================================
// D-Term Damping Tests
// ============================================================================

// Verify D-term pulls back position command proportional to measured velocity
TEST_F( VelocityToPositionCommandControllerTest, DTermDampsProportionalToVelocity )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  activateController();

  // vel_cmd = 1.0, vel_actual = 0 -> D-term = 0 (no motion to damp)
  setVelocity( 0, 0.0 );
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;
  callUpdate();

  // desired_pos = 0.01, vel_p = 0.02, vel_d = 0.1 * 0.0 * 0.01 = 0
  double cmd_no_vel = hw_cmd_values_[0];
  EXPECT_NEAR( cmd_no_vel, 0.03, 1e-9 );

  // Now set high measured velocity -> D-term should pull back
  setVelocity( 0, 5.0 );
  controller_->reference_interfaces_[0] = 1.0;
  callUpdate();

  // desired_pos = 0.02, vel_p = 2.0*(1.0-5.0)*0.01 = -0.08, vel_d = 0.1*5.0*0.01 = 0.005
  // pos_cmd = 0.02 + (-0.08) - 0.005 = -0.065
  double cmd_high_vel = hw_cmd_values_[0];
  EXPECT_NEAR( cmd_high_vel, -0.065, 1e-9 );

  // D-term contribution (0.005) is in the same direction as P-term: both resist fast motion
  EXPECT_LT( cmd_high_vel, cmd_no_vel );
}

// Verify D-term is zero when measured velocity is zero (joint at rest)
TEST_F( VelocityToPositionCommandControllerTest, DTermZeroWhenJointAtRest )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  activateController();

  // vel_actual = 0 -> D-term must be exactly 0
  setVelocity( 0, 0.0 );
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;
  callUpdate();

  // desired_pos = 0.01, vel_p = 2.0*(1.0-0.0)*0.01 = 0.02, vel_d = 0
  // pos_cmd = 0.01 + 0.02 - 0 = 0.03
  EXPECT_NEAR( hw_cmd_values_[0], 0.03, 1e-9 );

  // Second update, vel_actual still 0
  callUpdate();

  // desired_pos = 0.02, vel_p = 0.02, vel_d = 0
  // pos_cmd = 0.02 + 0.02 = 0.04
  EXPECT_NEAR( hw_cmd_values_[0], 0.04, 1e-9 );
}

// ============================================================================
// State Machine Tests
// ============================================================================

// Verify STOPPED state holds position and does not integrate
TEST_F( VelocityToPositionCommandControllerTest, StoppedHoldsPosition )
{
  initController();
  configureController();
  setupHardwareInterfaces();

  setPosition( 0, 0.5 );
  activateController();

  // Joint starts in STOPPED state after activation, vel_cmd = 0
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  auto ret = callUpdate();
  EXPECT_EQ( ret, controller_interface::return_type::OK );

  // In STOPPED state, should hold position
  EXPECT_DOUBLE_EQ( hw_cmd_values_[0], 0.5 );
  EXPECT_EQ( controller_->move_states_[0], MoveState::STOPPED );
}

// Verify state machine transition MOVING -> STOPPING -> STOPPED
TEST_F( VelocityToPositionCommandControllerTest, MovingToStoppingToStoppedTransition )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  activateController();

  // Start moving
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;
  callUpdate();
  EXPECT_EQ( controller_->move_states_[0], MoveState::MOVING );

  // Stop commanding (velocity goes to 0) but joint still has velocity
  setVelocity( 0, 0.5 ); // joint still moving
  controller_->reference_interfaces_[0] = 0.0;
  callUpdate();
  EXPECT_EQ( controller_->move_states_[0], MoveState::STOPPING );

  // Joint velocity drops below threshold -> STOPPED
  setVelocity( 0, 0.001 );
  controller_->reference_interfaces_[0] = 0.0;
  callUpdate();
  EXPECT_EQ( controller_->move_states_[0], MoveState::STOPPED );
}

// Verify desired_positions re-syncs to actual position on STOPPED -> MOVING transition
TEST_F( VelocityToPositionCommandControllerTest, StoppedToMovingResyncsDesiredPosition )
{
  initController();
  configureController();
  setupHardwareInterfaces();

  setPosition( 0, 1.0 );
  activateController();

  // State is STOPPED, hold at 1.0
  EXPECT_DOUBLE_EQ( controller_->desired_positions_[0], 1.0 );
  EXPECT_DOUBLE_EQ( controller_->hold_positions_[0], 1.0 );

  // Simulate external perturbation: joint was pushed to 1.5
  setPosition( 0, 1.5 );

  // Now start moving -> desired_positions should re-sync to actual position (1.5)
  controller_->reference_interfaces_[0] = 0.5;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;
  callUpdate();

  EXPECT_EQ( controller_->move_states_[0], MoveState::MOVING );
  // desired_positions was re-synced to 1.5 before integration
  // desired_pos = 1.5 + 0.5 * 0.01 = 1.505
  EXPECT_NEAR( controller_->desired_positions_[0], 1.505, 1e-9 );
}

// Verify desired_positions re-syncs to actual position on STOPPING -> MOVING transition
TEST_F( VelocityToPositionCommandControllerTest, StoppingToMovingResyncsDesiredPosition )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  activateController();

  // Start moving
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;
  callUpdate();
  EXPECT_EQ( controller_->move_states_[0], MoveState::MOVING );

  // Go to STOPPING
  setVelocity( 0, 0.5 );
  controller_->reference_interfaces_[0] = 0.0;
  callUpdate();
  EXPECT_EQ( controller_->move_states_[0], MoveState::STOPPING );

  // Resume moving from STOPPING -> should re-sync
  double pos_before = controller_->joint_position_states_[0];
  controller_->reference_interfaces_[0] = 1.0;
  callUpdate();
  EXPECT_EQ( controller_->move_states_[0], MoveState::MOVING );
  // desired_positions was re-synced to actual position
  EXPECT_NEAR( controller_->desired_positions_[0], pos_before + 1.0 * 0.01, 1e-9 );
}

// Verify hold_positions tracks desired_positions (not actual joint state)
TEST_F( VelocityToPositionCommandControllerTest, HoldPositionUsesDesiredNotActual )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  activateController();

  // Move for several cycles
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  for ( int i = 0; i < 5; i++ ) { callUpdate(); }

  // hold_positions should track desired_positions, not joint_position_states
  EXPECT_DOUBLE_EQ( controller_->hold_positions_[0], controller_->desired_positions_[0] );
}

// ============================================================================
// E-Stop Tests
// ============================================================================

// Verify e-stop freezes all joints in STOPPED state with positions reset
TEST_F( VelocityToPositionCommandControllerTest, EStopHoldsAndResets )
{
  initController();
  configureController();
  setupHardwareInterfaces();

  setPosition( 0, 0.5 );
  setPosition( 1, -0.3 );
  setPosition( 2, 1.0 );
  activateController();

  // Start moving
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 1.0;
  controller_->reference_interfaces_[2] = 1.0;
  callUpdate();

  // Engage e-stop
  controller_->e_stop_active_.writeFromNonRT( true );
  callUpdate();

  // All joints should be STOPPED with hold positions = current positions
  for ( size_t i = 0; i < joints_.size(); i++ ) {
    EXPECT_EQ( controller_->move_states_[i], MoveState::STOPPED );
    EXPECT_DOUBLE_EQ( controller_->hold_positions_[i], controller_->joint_position_states_[i] );
    EXPECT_DOUBLE_EQ( controller_->desired_positions_[i], controller_->joint_position_states_[i] );
  }
}

// ============================================================================
// NaN Reference Tests
// ============================================================================

// Verify NaN velocity references do not overwrite command interfaces
TEST_F( VelocityToPositionCommandControllerTest, NaNReferenceSkipsWriting )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  activateController();

  controller_->reference_interfaces_[0] = std::numeric_limits<double>::quiet_NaN();
  controller_->reference_interfaces_[1] = std::numeric_limits<double>::quiet_NaN();
  controller_->reference_interfaces_[2] = std::numeric_limits<double>::quiet_NaN();

  // Set command values to known value to verify they don't change
  hw_cmd_values_[0] = 99.0;
  hw_cmd_values_[1] = 99.0;
  hw_cmd_values_[2] = 99.0;

  auto ret = callUpdate();
  EXPECT_EQ( ret, controller_interface::return_type::OK );

  // Commands should NOT have been overwritten
  EXPECT_DOUBLE_EQ( hw_cmd_values_[0], 99.0 );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[1], 99.0 );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[2], 99.0 );
}

// ============================================================================
// Repeated Activation/Deactivation Cycle Tests
// ============================================================================

// Verify clean state reset across multiple activate/deactivate cycles
TEST_F( VelocityToPositionCommandControllerTest, RepeatedActivateDeactivateCycles )
{
  initController();
  configureController();
  setupHardwareInterfaces();

  for ( int cycle = 0; cycle < 3; ++cycle ) {
    activateController();

    // Verify state reset
    for ( size_t i = 0; i < joints_.size(); ++i ) {
      EXPECT_EQ( controller_->move_states_[i], MoveState::STOPPED )
          << "Cycle " << cycle << ": joint " << i << " not STOPPED after activate";
    }

    // Verify functional after activation
    controller_->reference_interfaces_[0] = 0.5;
    controller_->reference_interfaces_[1] = 0.0;
    controller_->reference_interfaces_[2] = 0.0;
    auto ret = callUpdate();
    EXPECT_EQ( ret, controller_interface::return_type::OK );

    deactivateController();
  }
}

// Verify controller is functional after e-stop + deactivate + reactivate cycle
TEST_F( VelocityToPositionCommandControllerTest, ReactivateAfterEstop )
{
  initController();
  configureController();
  setupHardwareInterfaces();

  activateController();

  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;
  callUpdate();

  controller_->e_stop_active_.writeFromNonRT( true );
  callUpdate();

  deactivateController();
  activateController();

  // All joints should be in clean STOPPED state
  for ( size_t i = 0; i < joints_.size(); ++i ) {
    EXPECT_EQ( controller_->move_states_[i], MoveState::STOPPED );
  }

  // Verify functional after re-activation
  controller_->reference_interfaces_[0] = 0.5;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;
  auto ret = callUpdate();
  EXPECT_EQ( ret, controller_interface::return_type::OK );
}

// ============================================================================
// Synchronization Tests
// ============================================================================

// Verify joints in the same sync group are marked as synchronized
TEST_F( VelocityToPositionCommandControllerTest, SynchronizationCorrection )
{
  // Put joint1 and joint2 in same sync group
  std::vector<std::string> sync_groups = { "group1", "group1", "group2" };
  initController( sync_groups );
  configureController();
  setupHardwareInterfaces();
  activateController();

  // Both joints start at 0 with same velocity -> synchronized
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 1.0;
  controller_->reference_interfaces_[2] = 0.0;
  callUpdate();

  // Joint0 and joint1 should be synced
  EXPECT_TRUE( controller_->sync_states_[0] );
  EXPECT_TRUE( controller_->sync_states_[1] );
  EXPECT_FALSE( controller_->sync_states_[2] );
}

// Verify joints with different velocities in same group are not synchronized
TEST_F( VelocityToPositionCommandControllerTest, SyncGroupDifferentVelocitiesNotSynced )
{
  std::vector<std::string> sync_groups = { "group1", "group1", "group2" };
  initController( sync_groups );
  configureController();
  setupHardwareInterfaces();
  activateController();

  // Different velocities -> not synchronized
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 0.5;
  controller_->reference_interfaces_[2] = 0.0;
  callUpdate();

  EXPECT_FALSE( controller_->sync_states_[0] );
  EXPECT_FALSE( controller_->sync_states_[1] );
}

// ============================================================================
// Chained Mode Tests
// ============================================================================

// Verify on_set_chained_mode accepts both true and false
TEST_F( VelocityToPositionCommandControllerTest, ChainedModeAccepted )
{
  initController();
  configureController();

  EXPECT_TRUE( controller_->on_set_chained_mode( true ) );
  EXPECT_TRUE( controller_->on_set_chained_mode( false ) );
}

// ============================================================================
// Multi-Cycle Integration Test
// ============================================================================

// Verify desired_positions integrates correctly over multiple update cycles
TEST_F( VelocityToPositionCommandControllerTest, MultiCyclePositionTracking )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  activateController();

  // Run 10 update cycles at constant velocity 1.0 rad/s, dt=0.01s
  // Expected final desired_pos = 0 + 10 * 1.0 * 0.01 = 0.1
  for ( int i = 0; i < 10; i++ ) {
    controller_->reference_interfaces_[0] = 1.0;
    controller_->reference_interfaces_[1] = 0.0;
    controller_->reference_interfaces_[2] = 0.0;
    callUpdate();
  }

  EXPECT_NEAR( controller_->desired_positions_[0], 0.1, 1e-9 );
}

// ============================================================================
// main
// ============================================================================

int main( int argc, char **argv )
{
  testing::InitGoogleMock( &argc, argv );
  rclcpp::init( argc, argv );
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
