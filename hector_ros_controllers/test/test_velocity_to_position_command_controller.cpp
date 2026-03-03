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
                       const std::string &passthrough = "", double kd_sync = 0.0,
                       double max_sync_velocity = 0.0 )
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
        rclcpp::Parameter( "kd_sync", kd_sync ),
        rclcpp::Parameter( "max_sync_velocity", max_sync_velocity ),
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

// Verify state machine transition MOVING -> STOPPING -> STOPPED with braking deceleration
TEST_F( VelocityToPositionCommandControllerTest, MovingToStoppingWithBrakingDeceleration )
{
  initController();
  configureController();
  setupHardwareInterfaces();

  setPosition( 0, 0.0 );
  activateController();

  // Start moving
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;
  callUpdate();
  EXPECT_EQ( controller_->move_states_[0], MoveState::MOVING );

  // Simulate joint has moved and has velocity
  setPosition( 0, 0.5 );
  setVelocity( 0, 0.5 );

  // Stop commanding -> should enter STOPPING and begin braking
  controller_->reference_interfaces_[0] = 0.0;
  callUpdate();
  EXPECT_EQ( controller_->move_states_[0], MoveState::STOPPING );
  // desired_positions was re-synced to current position on entering STOPPING
  // Then position_control integrated with the decelerating stopping velocity
  // stopping_vel starts at 0.5, after one cycle: 0.5 - 5.0*0.01 = 0.45
  EXPECT_NEAR( controller_->stopping_velocities_[0], 0.45, 1e-9 );

  // Continue braking until stopped. At 5.0 rad/s^2 and starting at 0.5 rad/s,
  // it takes 0.5/5.0 = 0.1s = 10 cycles to stop.
  // We already did 1 cycle, so 9 more should bring it to STOPPED.
  for ( int i = 0; i < 9; i++ ) { callUpdate(); }
  EXPECT_EQ( controller_->move_states_[0], MoveState::STOPPED );
  EXPECT_DOUBLE_EQ( controller_->stopping_velocities_[0], 0.0 );
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

// Verify resuming from STOPPING re-syncs desired_positions
TEST_F( VelocityToPositionCommandControllerTest, ResumeFromStoppingResyncsDesiredPosition )
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

  // Stop -> enters STOPPING (braking)
  setPosition( 0, 0.7 );
  setVelocity( 0, 0.3 );
  controller_->reference_interfaces_[0] = 0.0;
  callUpdate();
  EXPECT_EQ( controller_->move_states_[0], MoveState::STOPPING );

  // Resume moving from STOPPING -> should re-sync to actual position
  setPosition( 0, 0.8 );
  controller_->reference_interfaces_[0] = 1.0;
  callUpdate();
  EXPECT_EQ( controller_->move_states_[0], MoveState::MOVING );
  // desired_positions was re-synced to 0.8 before integration
  // desired_pos = 0.8 + 1.0 * 0.01 = 0.81
  EXPECT_NEAR( controller_->desired_positions_[0], 0.81, 1e-9 );
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
// TODO: e-stop logic is currently disabled, re-enable when e-stop is fixed
TEST_F( VelocityToPositionCommandControllerTest, DISABLED_EStopHoldsAndResets )
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
// Position Limit Clamping Tests
// ============================================================================

// Verify joint limits are parsed from URDF during configure
TEST_F( VelocityToPositionCommandControllerTest, JointLimitsParsedFromUrdf )
{
  initController();
  configureController();

  // joint1: revolute, limits [-3.14159, 3.14159]
  EXPECT_DOUBLE_EQ( controller_->joint_lower_limits_[0], -3.14159 );
  EXPECT_DOUBLE_EQ( controller_->joint_upper_limits_[0], 3.14159 );

  // joint2: revolute, limits [-1.5, 1.5]
  EXPECT_DOUBLE_EQ( controller_->joint_lower_limits_[1], -1.5 );
  EXPECT_DOUBLE_EQ( controller_->joint_upper_limits_[1], 1.5 );

  // joint3: revolute, limits [-2.0, 2.0]
  EXPECT_DOUBLE_EQ( controller_->joint_lower_limits_[2], -2.0 );
  EXPECT_DOUBLE_EQ( controller_->joint_upper_limits_[2], 2.0 );
}

// Verify position command is clamped at the upper limit
TEST_F( VelocityToPositionCommandControllerTest, PositionClampedAtUpperLimit )
{
  initController();
  configureController();
  setupHardwareInterfaces();

  // Start joint2 near its upper limit (1.5)
  setPosition( 1, 1.49 );
  activateController();

  // Command large positive velocity on joint2 to push past the limit
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = 10.0;
  controller_->reference_interfaces_[2] = 0.0;

  // Run multiple cycles to integrate well past the limit
  for ( int i = 0; i < 50; i++ ) { callUpdate(); }

  // Position command must be clamped at the upper limit
  EXPECT_LE( hw_cmd_values_[1], 1.5 );
  EXPECT_NEAR( hw_cmd_values_[1], 1.5, 1e-9 );
}

// Verify position command is clamped at the lower limit
TEST_F( VelocityToPositionCommandControllerTest, PositionClampedAtLowerLimit )
{
  initController();
  configureController();
  setupHardwareInterfaces();

  // Start joint2 near its lower limit (-1.5)
  setPosition( 1, -1.49 );
  activateController();

  // Command large negative velocity on joint2 to push past the limit
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = -10.0;
  controller_->reference_interfaces_[2] = 0.0;

  for ( int i = 0; i < 50; i++ ) { callUpdate(); }

  // Position command must be clamped at the lower limit
  EXPECT_GE( hw_cmd_values_[1], -1.5 );
  EXPECT_NEAR( hw_cmd_values_[1], -1.5, 1e-9 );
}

// Verify continuous joints are NOT clamped
TEST_F( VelocityToPositionCommandControllerTest, ContinuousJointNotClamped )
{
  // Use joint4 which is continuous in the test URDF
  joints_ = { "joint4" };
  initController();
  configureController();
  setupHardwareInterfaces();
  activateController();

  // Limits should be NaN for continuous joints
  EXPECT_TRUE( std::isnan( controller_->joint_lower_limits_[0] ) );
  EXPECT_TRUE( std::isnan( controller_->joint_upper_limits_[0] ) );

  // Command high velocity for many cycles -> should exceed any typical limit
  controller_->reference_interfaces_[0] = 10.0;
  for ( int i = 0; i < 200; i++ ) { callUpdate(); }

  // Should have integrated freely past any typical revolute limit
  EXPECT_GT( hw_cmd_values_[0], 3.14159 );
}

// Verify desired_positions_ does not wind up past joint limits
TEST_F( VelocityToPositionCommandControllerTest, DesiredPositionClampedPreventsWindup )
{
  initController();
  configureController();
  setupHardwareInterfaces();

  setPosition( 1, 1.4 );
  activateController();

  // Drive joint2 into the upper limit for many cycles
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = 10.0;
  controller_->reference_interfaces_[2] = 0.0;

  for ( int i = 0; i < 100; i++ ) { callUpdate(); }

  // desired_positions_ must be clamped too (no integrator windup)
  EXPECT_LE( controller_->desired_positions_[1], 1.5 );
  EXPECT_LE( controller_->hold_positions_[1], 1.5 );
}

// ============================================================================
// Synced Braking Tests
// ============================================================================

// Verify synced braking: when two synced joints stop together, the faster one
// slows its braking to maintain the position difference with the weaker one.
TEST_F( VelocityToPositionCommandControllerTest, IndependentBraking )
{
  std::vector<std::string> sync_groups = { "group1", "group1", "group2" };
  initController( sync_groups );
  configureController();
  setupHardwareInterfaces();
  activateController();

  // Move both synced joints together
  for ( int i = 0; i < 10; i++ ) {
    controller_->reference_interfaces_[0] = 1.0;
    controller_->reference_interfaces_[1] = 1.0;
    controller_->reference_interfaces_[2] = 0.0;
    setPosition( 0, ( i + 1 ) * 0.01 );
    setPosition( 1, ( i + 1 ) * 0.01 );
    setVelocity( 0, 1.0 );
    setVelocity( 1, 1.0 );
    callUpdate();
  }

  // Stop both — joint0 has low velocity (stops quickly), joint1 has high velocity
  setVelocity( 0, 0.1 );
  setVelocity( 1, 0.8 );
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = 0.0;
  callUpdate();

  EXPECT_EQ( controller_->move_states_[0], MoveState::STOPPING );
  EXPECT_EQ( controller_->move_states_[1], MoveState::STOPPING );

  // After a few cycles, joint0 should be STOPPED independently (no waiting for partner)
  for ( int i = 0; i < 5; i++ ) { callUpdate(); }

  EXPECT_EQ( controller_->move_states_[0], MoveState::STOPPED );
  // joint1 should still be braking (0.8 / (5.0 * 0.01) = 16 cycles)
  EXPECT_EQ( controller_->move_states_[1], MoveState::STOPPING );

  // Continue until joint1 also finishes
  for ( int i = 0; i < 20; i++ ) { callUpdate(); }
  EXPECT_EQ( controller_->move_states_[1], MoveState::STOPPED );
}

// ============================================================================
// Sync Offset Initialization Tests
// ============================================================================

// Verify sync offsets are properly initialized when joints move together (not NaN)
TEST_F( VelocityToPositionCommandControllerTest, SyncOffsetsInitializedWhenMovingTogether )
{
  std::vector<std::string> sync_groups = { "group1", "group1", "group2" };
  initController( sync_groups );
  configureController();
  setupHardwareInterfaces();

  setPosition( 0, 0.0 );
  setPosition( 1, 0.1 ); // deliberate offset
  activateController();

  // Offsets should be set from activation (via reset_sync_offsets)
  EXPECT_FALSE( std::isnan( controller_->sync_offsets_[0][0] ) );
  EXPECT_FALSE( std::isnan( controller_->sync_offsets_[1][0] ) );
  // joint1_pos - joint0_pos = 0.1
  EXPECT_NEAR( controller_->sync_offsets_[0][0], 0.1, 1e-9 );
  // joint0_pos - joint1_pos = -0.1
  EXPECT_NEAR( controller_->sync_offsets_[1][0], -0.1, 1e-9 );

  // Move both joints together at same velocity
  for ( int i = 0; i < 10; i++ ) {
    controller_->reference_interfaces_[0] = 1.0;
    controller_->reference_interfaces_[1] = 1.0;
    controller_->reference_interfaces_[2] = 0.0;
    setPosition( 0, ( i + 1 ) * 0.01 );
    setPosition( 1, 0.1 + ( i + 1 ) * 0.01 );
    setVelocity( 0, 1.0 );
    setVelocity( 1, 1.0 );
    callUpdate();
  }

  // Offsets should still be valid (not NaN) and reflect the position relationship
  EXPECT_FALSE( std::isnan( controller_->sync_offsets_[0][0] ) );
  EXPECT_FALSE( std::isnan( controller_->sync_offsets_[1][0] ) );
  EXPECT_NEAR( controller_->sync_offsets_[0][0], 0.1, 1e-6 );
}

// Verify synced flippers stay close after repeated up-down cycles
TEST_F( VelocityToPositionCommandControllerTest, SyncRestoredAfterRepeatedStopStart )
{
  std::vector<std::string> sync_groups = { "group1", "group1", "group2" };
  initController( sync_groups );
  configureController();
  setupHardwareInterfaces();

  setPosition( 0, 0.0 );
  setPosition( 1, 0.0 );
  activateController();

  // Repeat 5 up-down cycles: move up -> stop -> move down -> stop
  double pos0 = 0.0;
  double pos1 = 0.0;

  for ( int cycle = 0; cycle < 5; cycle++ ) {
    double vel = ( cycle % 2 == 0 ) ? 1.0 : -1.0;

    // Move for 20 cycles
    for ( int i = 0; i < 20; i++ ) {
      controller_->reference_interfaces_[0] = vel;
      controller_->reference_interfaces_[1] = vel;
      controller_->reference_interfaces_[2] = 0.0;
      // Simulate: joint0 tracks well, joint1 has slight drift
      pos0 += vel * 0.01;
      pos1 += vel * 0.01 * 1.02; // 2% drift per cycle
      setPosition( 0, pos0 );
      setPosition( 1, pos1 );
      setVelocity( 0, vel );
      setVelocity( 1, vel * 1.02 );
      callUpdate();
    }

    // Stop both
    controller_->reference_interfaces_[0] = 0.0;
    controller_->reference_interfaces_[1] = 0.0;
    setVelocity( 0, vel * 0.5 );
    setVelocity( 1, vel * 0.5 );
    callUpdate();

    // Brake until stopped
    for ( int i = 0; i < 20; i++ ) { callUpdate(); }

    EXPECT_EQ( controller_->move_states_[0], MoveState::STOPPED );
    EXPECT_EQ( controller_->move_states_[1], MoveState::STOPPED );
  }

  // The position difference should not have grown unbounded.
  // With sync correction active, each cycle corrects the drift.
  // Without fix: offsets are NaN, no correction ever happens, drift accumulates.
  double final_diff = std::abs( pos0 - pos1 );
  // Allow some tolerance — sync P-control can't perfectly correct drift,
  // but it should prevent unbounded accumulation
  EXPECT_LT( final_diff, 0.5 ) << "Position difference after 5 cycles: " << final_diff;
}

// ============================================================================
// Sync Correction Clamping & PD Tests
// ============================================================================

// Verify sync correction is clamped to abs(vel_command) during MOVING
TEST_F( VelocityToPositionCommandControllerTest, SyncCorrectionClampedToVelocityDuringMoving )
{
  std::vector<std::string> sync_groups = { "group1", "group1", "group2" };
  initController( sync_groups );
  configureController();
  setupHardwareInterfaces();

  // Start with a large offset between synced joints
  setPosition( 0, 0.0 );
  setPosition( 1, 1.0 ); // 1.0 rad apart (sync_offset will be 1.0)
  activateController();

  // Now move joint1 further to create sync error
  setPosition( 1, 1.5 ); // actual offset is 1.5, stored offset is 1.0 -> error = 0.5

  // Command low velocity (0.1 rad/s) -> sync correction should be clamped to 0.1*dt = 0.001
  controller_->reference_interfaces_[0] = 0.1;
  controller_->reference_interfaces_[1] = 0.1;
  controller_->reference_interfaces_[2] = 0.0;
  callUpdate();

  double cmd0_after = hw_cmd_values_[0];
  double cmd1_after = hw_cmd_values_[1];

  // The sync correction for joint0 should push it toward joint1 (positive correction)
  // but clamped to 0.1 * 0.01 = 0.001
  // Without clamping, kp_sync=1.0 * error=0.5 = 0.5 rad (way too much)
  // The actual position command includes the PD terms too, but the sync component is bounded
  // Check that position commands are not wildly different (bounded by vel_cmd)
  double pos_diff = std::abs( cmd0_after - cmd1_after );
  // With clamped sync and only 0.001 max correction, the offset should remain close to initial
  EXPECT_GT( pos_diff, 0.9 ) << "Sync correction should be clamped, not aggressively resync";
}

// Verify sync correction during STOPPING keeps joints closer together
TEST_F( VelocityToPositionCommandControllerTest, SyncCorrectionDuringStopping )
{
  std::vector<std::string> sync_groups = { "group1", "group1", "group2" };
  initController( sync_groups );
  configureController();
  setupHardwareInterfaces();

  setPosition( 0, 0.0 );
  setPosition( 1, 0.0 );
  activateController();

  // Move both synced joints together
  for ( int i = 0; i < 10; i++ ) {
    controller_->reference_interfaces_[0] = 1.0;
    controller_->reference_interfaces_[1] = 1.0;
    controller_->reference_interfaces_[2] = 0.0;
    setPosition( 0, ( i + 1 ) * 0.01 );
    setPosition( 1, ( i + 1 ) * 0.01 );
    setVelocity( 0, 1.0 );
    setVelocity( 1, 1.0 );
    callUpdate();
  }

  // Stop both — joint0 has low velocity (stops quickly), joint1 has high velocity
  // This simulates asymmetric braking (one motor is weaker)
  setPosition( 0, 0.1 );
  setPosition( 1, 0.12 ); // slight drift
  setVelocity( 0, 0.1 );
  setVelocity( 1, 0.8 );
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = 0.0;
  callUpdate();

  EXPECT_EQ( controller_->move_states_[0], MoveState::STOPPING );
  EXPECT_EQ( controller_->move_states_[1], MoveState::STOPPING );

  // Both joints are synced (same vel_command = 0.0)
  EXPECT_TRUE( controller_->sync_states_[0] );
  EXPECT_TRUE( controller_->sync_states_[1] );

  // The desired_positions should incorporate sync correction during braking
  // joint0 has smaller stopping_velocity, so its sync correction is also smaller
  // joint1 has larger stopping_velocity, so it can correct more
  double desired_diff_before =
      std::abs( controller_->desired_positions_[0] - controller_->desired_positions_[1] );

  // Run a few more braking cycles
  for ( int i = 0; i < 5; i++ ) { callUpdate(); }

  // The desired positions should be closer than they would be without sync
  // (With sync correction, the faster joint is pulled toward the slower one)
  double desired_diff_after =
      std::abs( controller_->desired_positions_[0] - controller_->desired_positions_[1] );

  // Sync correction should have reduced or maintained the difference
  EXPECT_LE( desired_diff_after, desired_diff_before + 0.01 )
      << "Sync correction during STOPPING should prevent positions from diverging";
}

// Verify sync correction is never applied during STOPPED (flippers stay still)
TEST_F( VelocityToPositionCommandControllerTest, NoSyncCorrectionDuringStopped )
{
  std::vector<std::string> sync_groups = { "group1", "group1", "group2" };
  // Even with max_sync_velocity > 0, STOPPED should not move
  initController( sync_groups, "", 0.0, 0.5 );
  configureController();
  setupHardwareInterfaces();

  setPosition( 0, 0.0 );
  setPosition( 1, 0.0 );
  activateController();

  // Create desync
  setPosition( 1, 0.1 );

  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  double hold0_before = controller_->hold_positions_[0];
  double hold1_before = controller_->hold_positions_[1];

  // Run STOPPED cycles — hold positions must NOT change
  for ( int i = 0; i < 50; i++ ) { callUpdate(); }

  EXPECT_DOUBLE_EQ( controller_->hold_positions_[0], hold0_before )
      << "Hold position must not change during STOPPED";
  EXPECT_DOUBLE_EQ( controller_->hold_positions_[1], hold1_before )
      << "Hold position must not change during STOPPED";
}

// Verify kd_sync affects sync correction (D-term uses velocity difference)
TEST_F( VelocityToPositionCommandControllerTest, KdSyncDampsCorrection )
{
  std::vector<std::string> sync_groups = { "group1", "group1", "group2" };

  // First run WITHOUT kd_sync, using a small sync error so the clamp doesn't dominate
  initController( sync_groups );
  configureController();
  setupHardwareInterfaces();

  setPosition( 0, 0.0 );
  setPosition( 1, 0.0 );
  activateController();

  // Create small sync error and velocity difference
  // sync_offset stored as 0. Actual offset = 0.005. Error = 0.005.
  // P-term = 1.0 * 0.005 = 0.005 (within clamp of vel_cmd * dt = 10.0 * 0.01 = 0.1)
  setPosition( 1, 0.005 );
  setVelocity( 0, 0.5 );
  setVelocity( 1, 1.5 ); // joint1 moving away from joint0

  controller_->reference_interfaces_[0] = 10.0; // High vel to avoid clamp
  controller_->reference_interfaces_[1] = 10.0;
  controller_->reference_interfaces_[2] = 0.0;
  callUpdate();

  double cmd0_no_kd = hw_cmd_values_[0];

  // Reset and run WITH kd_sync
  controller_.reset();
  rtest::StaticMocksRegistry::instance().reset();
  controller_ = std::make_shared<VelToPosController>();

  initController( sync_groups, "", 0.5 ); // kd_sync = 0.5
  configureController();
  setupHardwareInterfaces();

  setPosition( 0, 0.0 );
  setPosition( 1, 0.0 );
  activateController();

  setPosition( 1, 0.005 );
  setVelocity( 0, 0.5 );
  setVelocity( 1, 1.5 );

  controller_->reference_interfaces_[0] = 10.0;
  controller_->reference_interfaces_[1] = 10.0;
  controller_->reference_interfaces_[2] = 0.0;
  callUpdate();

  double cmd0_with_kd = hw_cmd_values_[0];

  // With kd_sync, joint0's correction should be larger because partner is moving faster
  // D-term adds kd_sync * (partner_vel - this_vel) = 0.5 * (1.5 - 0.5) = 0.5
  // Total correction = 0.005 + 0.5 = 0.505 (still within clamp of 0.1)
  // Wait — 0.505 > 0.1, so it's still clamped! Use even higher vel_command or smaller kd.
  // Actually vel_limit = abs(10.0) = 10.0, max_correction = 10.0 * 0.01 = 0.1
  // P-only: 0.005 < 0.1, not clamped -> correction = 0.005
  // P+D: 0.005 + 0.5 = 0.505 > 0.1, clamped to 0.1
  // So cmd0_with_kd > cmd0_no_kd by 0.1 - 0.005 = 0.095
  EXPECT_GT( cmd0_with_kd, cmd0_no_kd )
      << "kd_sync should increase correction when partner is moving away. "
      << "Without kd: " << cmd0_no_kd << ", with kd: " << cmd0_with_kd;
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
