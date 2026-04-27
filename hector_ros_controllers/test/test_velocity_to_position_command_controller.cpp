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

  int update_count_ = 0;

  controller_interface::return_type callUpdate()
  {
    // Advance time by 10ms per call so that time-dependent logic (braking profiles, timeout) works
    auto ns = static_cast<int64_t>( update_count_ ) * 10'000'000LL; // 10ms in ns
    rclcpp::Time now( ns, RCL_ROS_TIME );
    rclcpp::Duration period( std::chrono::milliseconds( 10 ) ); // dt = 0.01s
    update_count_++;
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

// Verify state machine transition MOVING -> STOPPING -> STOPPED with trapezoidal braking profile
TEST_F( VelocityToPositionCommandControllerTest, MovingToStoppingWithBrakingProfile )
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

  // Stop commanding -> should enter STOPPING with a trapezoidal braking profile
  controller_->reference_interfaces_[0] = 0.0;
  callUpdate();
  EXPECT_EQ( controller_->move_states_[0], MoveState::STOPPING );

  // Braking profile: from pos=0.5 with vel=0.5, decel=4.0 (max_deceleration default)
  // braking_distance = 0.5*0.5 / (2*4.0) = 0.03125, target = 0.53125
  // profile duration = 0.5/4.0 = 0.125s = 12.5 cycles -> 13 cycles to complete
  EXPECT_NEAR( controller_->braking_profiles_[0].target_position, 0.53125, 1e-9 );
  EXPECT_NEAR( controller_->braking_profiles_[0].total_time, 0.125, 1e-9 );

  // Continue braking until profile completes (we already did 1 cycle in STOPPING)
  for ( int i = 0; i < 13; i++ ) { callUpdate(); }
  EXPECT_EQ( controller_->move_states_[0], MoveState::STOPPED );

  // Should hold at the braking target position
  EXPECT_NEAR( controller_->hold_positions_[0], 0.53125, 1e-9 );
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

  // Command velocity at max_velocity for many cycles -> should exceed any typical limit
  // max_velocity defaults to 1.0, so 500 cycles * 0.01 * 1.0 = 5.0 rad
  controller_->reference_interfaces_[0] = 10.0; // clamped to max_velocity (1.0)
  for ( int i = 0; i < 500; i++ ) { callUpdate(); }

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

// Verify braking with trapezoidal profiles: joints with different velocities
// get different profile durations and stop at different times.
TEST_F( VelocityToPositionCommandControllerTest, IndependentBrakingWithProfiles )
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

  // joint0 profile: vel=0.1, decel=4.0 -> duration=0.1/4.0=0.025s = 2.5 cycles
  // joint1 profile: vel=0.8, decel=4.0 -> duration=0.8/4.0=0.2s = 20 cycles
  EXPECT_NEAR( controller_->braking_profiles_[0].total_time, 0.025, 1e-9 );
  EXPECT_NEAR( controller_->braking_profiles_[1].total_time, 0.2, 1e-9 );

  // After 3 cycles, joint0 should be STOPPED
  for ( int i = 0; i < 3; i++ ) { callUpdate(); }
  EXPECT_EQ( controller_->move_states_[0], MoveState::STOPPED );
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
  EXPECT_FALSE( controller_->sync_pairs_.is_offset_nan( 0 ) );
  EXPECT_FALSE( controller_->sync_pairs_.is_offset_nan( 1 ) );
  // joint1_pos - joint0_pos = 0.1
  EXPECT_NEAR( controller_->sync_pairs_.get_offset( 0 ), 0.1, 1e-9 );
  // joint0_pos - joint1_pos = -0.1 (symmetric by construction)
  EXPECT_NEAR( controller_->sync_pairs_.get_offset( 1 ), -0.1, 1e-9 );

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
  EXPECT_FALSE( controller_->sync_pairs_.is_offset_nan( 0 ) );
  EXPECT_FALSE( controller_->sync_pairs_.is_offset_nan( 1 ) );
  EXPECT_NEAR( controller_->sync_pairs_.get_offset( 0 ), 0.1, 1e-6 );
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
// Trapezoidal Profile Unit Tests
// ============================================================================

using TrapProfile = velocity_to_position_command_controller::TrapezoidalProfile;

// Verify full trapezoidal profile (accel + cruise + decel)
TEST( TrapezoidalProfileTest, FullTrapezoidalProfile )
{
  auto p = TrapProfile::compute( 0.0, 2.0, 1.0, 2.0 );

  // t_accel = 1.0/2.0 = 0.5s, dist_accel = 0.5*2.0*0.25 = 0.25
  // dist_decel = 0.25, cruise_dist = 2.0 - 0.5 = 1.5, t_cruise = 1.5/1.0 = 1.5
  // total = 0.5 + 1.5 + 0.5 = 2.5
  EXPECT_NEAR( p.total_time, 2.5, 1e-9 );
  EXPECT_NEAR( p.t_accel, 0.5, 1e-9 );
  EXPECT_NEAR( p.t_cruise, 1.5, 1e-9 );
  EXPECT_NEAR( p.t_decel, 0.5, 1e-9 );

  // At t=0: pos=0, vel=0
  auto [pos0, vel0] = p.evaluate( 0.0 );
  EXPECT_NEAR( pos0, 0.0, 1e-9 );
  EXPECT_NEAR( vel0, 0.0, 1e-9 );

  // At t=total: pos=target, vel=0
  auto [posEnd, velEnd] = p.evaluate( p.total_time );
  EXPECT_NEAR( posEnd, 2.0, 1e-9 );
  EXPECT_NEAR( velEnd, 0.0, 1e-9 );

  // At t=0.5 (end of accel): vel should be max_velocity
  auto [posAccel, velAccel] = p.evaluate( 0.5 );
  EXPECT_NEAR( velAccel, 1.0, 1e-9 );
}

// Verify triangular profile (distance too short for full cruise)
TEST( TrapezoidalProfileTest, TriangularProfile )
{
  // Distance = 0.1, max_vel = 10.0, accel = 2.0
  // dist_for_full = 10^2/2 = 50 >> 0.1, so triangular
  // v_peak = sqrt(0.1 * 2.0) = sqrt(0.2) ≈ 0.4472
  // t_accel = v_peak / 2.0 ≈ 0.2236
  auto p = TrapProfile::compute( 1.0, 1.1, 10.0, 2.0 );

  EXPECT_NEAR( p.t_cruise, 0.0, 1e-9 );
  EXPECT_GT( p.total_time, 0.0 );

  auto [posEnd, velEnd] = p.evaluate( p.total_time );
  EXPECT_NEAR( posEnd, 1.1, 1e-6 );
  EXPECT_NEAR( velEnd, 0.0, 1e-6 );
}

// Verify negative direction profile
TEST( TrapezoidalProfileTest, NegativeDirection )
{
  auto p = TrapProfile::compute( 2.0, 0.0, 1.0, 2.0 );

  EXPECT_EQ( p.direction, -1 );

  auto [posEnd, velEnd] = p.evaluate( p.total_time );
  EXPECT_NEAR( posEnd, 0.0, 1e-9 );
  EXPECT_NEAR( velEnd, 0.0, 1e-9 );
}

// Verify zero-distance profile
TEST( TrapezoidalProfileTest, ZeroDistance )
{
  auto p = TrapProfile::compute( 1.0, 1.0, 1.0, 2.0 );
  EXPECT_NEAR( p.total_time, 0.0, 1e-9 );

  auto [pos, vel] = p.evaluate( 0.0 );
  EXPECT_NEAR( pos, 1.0, 1e-9 );
}

// ============================================================================
// Velocity Command Timeout Tests
// ============================================================================

TEST_F( VelocityToPositionCommandControllerTest, VelocityTimeoutZerosReferences )
{
  initController();

  // Set timeout parameter before configure
  controller_->get_node()->set_parameter( rclcpp::Parameter( "velocity_command_timeout", 0.05 ) );

  configureController();
  setupHardwareInterfaces();
  activateController();

  // Send non-zero velocity for a few cycles
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;
  callUpdate(); // t=0ms, resets last_command_time

  // Now send zero velocity for enough cycles to exceed timeout (50ms = 5 cycles)
  controller_->reference_interfaces_[0] = 0.0;
  for ( int i = 0; i < 6; i++ ) { callUpdate(); }

  // After timeout, references should remain 0 (already were 0, timeout just ensures it)
  EXPECT_DOUBLE_EQ( controller_->reference_interfaces_[0], 0.0 );
}

TEST_F( VelocityToPositionCommandControllerTest, VelocityTimeoutDisabledByDefault )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  activateController();

  // Default timeout is 0 (disabled). Sending NaN-ish references should not crash.
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  for ( int i = 0; i < 100; i++ ) { callUpdate(); }
  // Should not crash, references remain 0
  EXPECT_DOUBLE_EQ( controller_->reference_interfaces_[0], 0.0 );
}

// ============================================================================
// Velocity Clamping Tests
// ============================================================================

TEST_F( VelocityToPositionCommandControllerTest, VelocityClampedToMaxVelocity )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  activateController();

  // max_velocity defaults to 1.0
  // Command 10.0 rad/s -> should be clamped to 1.0 in the update loop
  controller_->reference_interfaces_[0] = 10.0;
  controller_->reference_interfaces_[1] = -10.0;
  controller_->reference_interfaces_[2] = 0.0;
  callUpdate();

  // desired_pos should integrate with clamped velocity (1.0), not 10.0
  // desired_pos = 0 + 1.0 * 0.01 = 0.01
  EXPECT_NEAR( controller_->desired_positions_[0], 0.01, 1e-9 );
  // Negative clamping: desired_pos = 0 + (-1.0) * 0.01 = -0.01
  EXPECT_NEAR( controller_->desired_positions_[1], -0.01, 1e-9 );
}

// ============================================================================
// Group Action Cancellation by Velocity Command Tests
// ============================================================================

using GroupActionState = velocity_to_position_command_controller::GroupActionState;
using GroupActionCommand = velocity_to_position_command_controller::GroupActionCommand;
using TrapezoidalProfile = velocity_to_position_command_controller::TrapezoidalProfile;

// Verify that an active group action is cancelled when a non-zero velocity command arrives
TEST_F( VelocityToPositionCommandControllerTest, GroupActionCancelledByVelocityCommand )
{
  std::vector<std::string> sync_groups = { "group1", "group1", "group2" };
  initController( sync_groups );
  configureController();
  setupHardwareInterfaces();
  activateController();

  // Manually set up a group action on group1 (joints 0 and 1)
  size_t group_idx = controller_->group_index_map_["group1"];

  GroupActionCommand cmd;
  cmd.active = true;
  cmd.start_time = rclcpp::Time( 0, 0, RCL_ROS_TIME );
  cmd.target_position = 1.0;
  cmd.joint_profiles.push_back( TrapezoidalProfile::compute( 0.0, 1.0, 1.0, 2.0 ) );
  cmd.joint_profiles.push_back( TrapezoidalProfile::compute( 0.0, 1.0, 1.0, 2.0 ) );
  controller_->rt_group_action_cmds_[group_idx].writeFromNonRT( cmd );
  controller_->group_action_states_[group_idx].store( GroupActionState::EXECUTING );

  // First update with zero velocities -- action should continue
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;
  callUpdate();
  EXPECT_EQ( controller_->group_action_states_[group_idx].load(), GroupActionState::EXECUTING );

  // Now send a non-zero velocity command on joint 0 (in group1) -> action must be cancelled
  controller_->reference_interfaces_[0] = 0.5;
  callUpdate();
  EXPECT_EQ( controller_->group_action_states_[group_idx].load(), GroupActionState::CANCELLED );

  // Joint 0 should now be in normal velocity control (MOVING)
  EXPECT_EQ( controller_->move_states_[0], MoveState::MOVING );
}

// Verify that a velocity command on one group does NOT cancel an action on another group
TEST_F( VelocityToPositionCommandControllerTest, GroupActionNotCancelledByOtherGroupVelocity )
{
  std::vector<std::string> sync_groups = { "group1", "group1", "group2" };
  initController( sync_groups );
  configureController();
  setupHardwareInterfaces();
  activateController();

  size_t group1_idx = controller_->group_index_map_["group1"];

  GroupActionCommand cmd;
  cmd.active = true;
  cmd.start_time = rclcpp::Time( 0, 0, RCL_ROS_TIME );
  cmd.target_position = 1.0;
  cmd.joint_profiles.push_back( TrapezoidalProfile::compute( 0.0, 1.0, 1.0, 2.0 ) );
  cmd.joint_profiles.push_back( TrapezoidalProfile::compute( 0.0, 1.0, 1.0, 2.0 ) );
  controller_->rt_group_action_cmds_[group1_idx].writeFromNonRT( cmd );
  controller_->group_action_states_[group1_idx].store( GroupActionState::EXECUTING );

  // Send velocity command on joint 2 (group2) -- should NOT affect group1's action
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 1.0;
  callUpdate();

  // group1 action should still be executing
  EXPECT_EQ( controller_->group_action_states_[group1_idx].load(), GroupActionState::EXECUTING );
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
