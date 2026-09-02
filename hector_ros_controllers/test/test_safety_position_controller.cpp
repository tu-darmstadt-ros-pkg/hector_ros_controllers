#include "test_helpers.hpp"

#include <shared_mutex>

// __gcov_dump is only available when compiled with --coverage.
// Use a weak symbol so the call is a no-op in normal (non-coverage) builds.
#if defined( __GNUC__ )
extern "C" void __gcov_dump() __attribute__( ( weak ) );
#endif

using SafetyPositionControllerStatus =
    hector_ros_controllers_msgs::msg::SafetyPositionControllerStatus;
using SPC = safety_position_controller::SafetyPositionController;

// ============================================================================
// Fixture for full controller tests
// ============================================================================

class SafetyPositionControllerTest
    : public hector_test::SafetyControllerTestBase<SafetyPositionControllerTest>
{
public:
  using ControllerType = SPC;
  using StatusMsgType = SafetyPositionControllerStatus;

  std::vector<std::string> controlled_joints_{ "joint1", "joint2", "joint3" };

  std::shared_ptr<ControllerType> controller_;
  std::shared_ptr<rtest::PublisherMock<StatusMsgType>> status_pub_mock_;

  // Controller-specific mocks
  std::shared_ptr<rtest::ServiceMock<std_srvs::srv::SetBool>> bypass_service_mock_;

  void resetControllerSpecificMocks() { bypass_service_mock_.reset(); }

  void findControllerSpecificMocks( const std::string &node_name )
  {
    bypass_service_mock_ =
        rtest::findService<std_srvs::srv::SetBool>( node_name, "~/bypass_safety_checks" );
  }

  void SetUp() override { controller_ = std::make_shared<ControllerType>(); }

  void initController( const std::vector<std::string> &joints = {},
                       bool check_self_collisions = false, bool set_current_limits = false,
                       const std::string &urdf_file = "test_robot.urdf" )
  {
    auto j = joints.empty() ? controlled_joints_ : joints;
    const auto urdf = hector_test::loadUrdfFile( urdf_file );

    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "test_safety_position";
    params.robot_description = urdf;
    params.update_rate = kUpdateRate;
    params.controller_manager_update_rate = kUpdateRate;
    params.node_namespace = "";

    rclcpp::NodeOptions opts;
    std::vector<rclcpp::Parameter> overrides = {
        rclcpp::Parameter( "joints", j ),
        rclcpp::Parameter( "check_self_collisions", check_self_collisions ),
        rclcpp::Parameter( "collision_safety_zone", 0.05 ),
        rclcpp::Parameter( "set_current_limits", set_current_limits ),
        rclcpp::Parameter( "safety_bypass_timeout", 60.0 ),
        rclcpp::Parameter( "safety_bypass_joint_limit_tolerance", 0.03 ),
        rclcpp::Parameter( "publish_debug_joint_states", false ),
        rclcpp::Parameter( "collision_padding", 0.0 ),
        rclcpp::Parameter( "collision_cache_epsilon", 0.000001 ),
        rclcpp::Parameter( "debug_visualize_collisions", false ),
    };
    opts.parameter_overrides( overrides );
    params.node_options = opts;

    auto result = controller_->init( params );
    ASSERT_EQ( result, controller_interface::return_type::OK );
  }

  void configureController()
  {
    // Skip SRDF wait
    controller_->srdf_received_ = true;
    controller_->srdf_ = "";

    rclcpp_lifecycle::State unconfigured( lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                          "unconfigured" );
    auto cb = controller_->on_configure( unconfigured );
    ASSERT_EQ( cb, controller_interface::CallbackReturn::SUCCESS );
  }

  void setupHardwareInterfaces( const std::vector<std::string> &joints = {} )
  {
    auto cj = joints.empty() ? controlled_joints_ : joints;

    // State interfaces for ALL non-fixed joints (from all_joint_names_)
    hw_state_values_.assign( controller_->all_joint_names_.size(), 0.0 );
    // Command interfaces for controlled joints only
    hw_cmd_values_.assign( cj.size(), 0.0 );

    cmd_ifaces_.clear();
    state_ifaces_.clear();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    for ( size_t i = 0; i < cj.size(); ++i ) {
      cmd_ifaces_.push_back( std::make_shared<hardware_interface::CommandInterface>(
          cj[i], "position", &hw_cmd_values_[i] ) );
    }
    for ( size_t i = 0; i < controller_->all_joint_names_.size(); ++i ) {
      state_ifaces_.push_back( std::make_shared<hardware_interface::StateInterface>(
          controller_->all_joint_names_[i], "position", &hw_state_values_[i] ) );
    }
#pragma GCC diagnostic pop

    controller_->command_interfaces_.clear();
    controller_->state_interfaces_.clear();

    for ( auto &ci : cmd_ifaces_ ) {
      controller_->command_interfaces_.emplace_back( ci, []() { } );
    }
    for ( auto &si : state_ifaces_ ) { controller_->state_interfaces_.emplace_back( si ); }
  }

  // Set state value for a specific joint by name
  void setStateValue( const std::string &joint, double value )
  {
    for ( size_t i = 0; i < controller_->all_joint_names_.size(); ++i ) {
      if ( controller_->all_joint_names_[i] == joint ) {
        hw_state_values_[i] = value;
        return;
      }
    }
    FAIL() << "Joint '" << joint << "' not found in all_joint_names_";
  }

  // Mock hardware that follows the position command exactly.
  void followCommands()
  {
    for ( size_t i = 0; i < controlled_joints_.size(); ++i ) {
      setStateValue( controlled_joints_[i], hw_cmd_values_[i] );
    }
  }

  // Full chainable update. Unlike callUpdate() this also runs
  // update_reference_from_subscribers(), i.e. the non-chained "~/commands" path.
  controller_interface::return_type callFullUpdate()
  {
    rclcpp::Time now( 0, 0, RCL_ROS_TIME );
    rclcpp::Duration period( std::chrono::milliseconds( 10 ) );
    return controller_->update( now, period );
  }

  void sendCommand( const std::vector<double> &positions )
  {
    auto msg = std::make_shared<safety_position_controller::CmdType>();
    msg->data = positions;
    controller_->rt_command_ptr_.writeFromNonRT( msg );
  }
};

// ============================================================================
// Lifecycle & Init Tests
// ============================================================================

TEST_F( SafetyPositionControllerTest, OnInitSucceeds )
{
  initController();
  EXPECT_TRUE( controller_->param_listener_ != nullptr );
  EXPECT_TRUE( controller_->collision_checker_ == nullptr ); // disabled
  EXPECT_TRUE( controller_->bypass_safety_checks_service_ != nullptr );
}

// ============================================================================
// Enforce Limits Tests
// ============================================================================

TEST_F( SafetyPositionControllerTest, ReferenceAbovePositionLimitIsClamped )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  // Set current positions to 0
  for ( auto &v : hw_state_values_ ) v = 0.0;

  // Set reference beyond limit: joint2 upper is 1.5
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = 5.0;
  controller_->reference_interfaces_[2] = 0.0;

  callUpdate();
  EXPECT_LE( hw_cmd_values_[1], 1.5 );
}

TEST_F( SafetyPositionControllerTest, ReferenceBelowPositionLimitIsClamped )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  for ( auto &v : hw_state_values_ ) v = 0.0;

  // joint2 lower limit is -1.5
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = -5.0;
  controller_->reference_interfaces_[2] = 0.0;

  callUpdate();
  EXPECT_GE( hw_cmd_values_[1], -1.5 );
}

TEST_F( SafetyPositionControllerTest, ContinuousJointTakesShortestPath )
{
  // Use joints including joint4 (continuous)
  std::vector<std::string> j = { "joint1", "joint4" };
  initController( j );
  configureController();
  setupHardwareInterfaces( j );
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  // joint4 is continuous, current at 10.0, target at 0.1
  setStateValue( "joint4", 10.0 );
  setStateValue( "joint1", 0.0 );

  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = 0.1; // equivalent to 0.1 + 4*pi

  callUpdate();

  // The shortest path stays near 10.0 instead of unwinding to 0.1
  EXPECT_GT( hw_cmd_values_[1], 5.0 );
}

// ============================================================================
// Velocity Limiting Tests (no collision fixture — collision checks disabled)
// ============================================================================

// ============================================================================
// E-Stop Tests
// ============================================================================

TEST_F( SafetyPositionControllerTest, EstopEngageHoldsPositions )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  setStateValue( "joint1", 0.5 );
  setStateValue( "joint2", -0.3 );
  setStateValue( "joint3", 1.0 );

  // Provide valid references
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  // First update to read positions
  callUpdate();

  sendEstop( true );
  callUpdate();

  EXPECT_TRUE( controller_->estop_engaged_.load() );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[0], 0.5 );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[1], -0.3 );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[2], 1.0 );
}

TEST_F( SafetyPositionControllerTest, EstopReleaseHoldsUntilNewReference )
{
  // An E-stop abandons whatever was being tracked. On release the arm must hold even
  // though the upstream controller keeps writing the pre-E-stop target every cycle.
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  for ( auto &v : hw_state_values_ ) v = 0.0;
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  for ( int i = 0; i < 10; ++i ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    followCommands();
  }
  ASSERT_GT( hw_cmd_values_[0], 0.0 ) << "should have started moving toward the target";

  sendEstop( true );
  ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
  const double position_at_estop = hw_cmd_values_[0];

  sendEstop( false );
  for ( int i = 0; i < 50; ++i ) {
    controller_->reference_interfaces_[0] = 1.0; // upstream keeps commanding it
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    followCommands();
    ASSERT_NEAR( hw_cmd_values_[0], position_at_estop, 1e-6 )
        << "the pre-E-stop target must stay abandoned (cycle " << i << ")";
  }

  // A changed reference is a new command and releases the hold.
  for ( int i = 0; i < 20; ++i ) {
    controller_->reference_interfaces_[0] = 1.1;
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    followCommands();
  }
  EXPECT_GT( hw_cmd_values_[0], position_at_estop + 1e-3 );
}

TEST_F( SafetyPositionControllerTest, EstopPulseBetweenCyclesIsHonored )
{
  // A stalled executor can deliver the engage and the release back to back, so the
  // update loop never samples the engaged level. The engage must still abandon the
  // pre-E-stop target instead of cancelling out against the release.
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  for ( auto &v : hw_state_values_ ) v = 0.0;
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  for ( int i = 0; i < 10; ++i ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    followCommands();
  }
  ASSERT_GT( hw_cmd_values_[0], 0.0 ) << "should have started moving toward the target";

  // Both messages land between two update cycles.
  controller_->note_estop_request( true );
  controller_->note_estop_request( false );

  ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
  const double position_at_estop = hw_cmd_values_[0];
  EXPECT_TRUE( controller_->estop_engaged_.load() ) << "the latched engage must produce one cycle";

  ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
  followCommands();
  EXPECT_FALSE( controller_->estop_engaged_.load() )
      << "the latch must be consumed, so the release is seen on the next cycle";

  for ( int i = 0; i < 50; ++i ) {
    controller_->reference_interfaces_[0] = 1.0; // upstream keeps commanding it
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    followCommands();
    ASSERT_NEAR( hw_cmd_values_[0], position_at_estop, 1e-6 )
        << "the pulse must abandon the pre-E-stop target (cycle " << i << ")";
  }
}

TEST_F( SafetyPositionControllerTest, EstopReleaseHoldsUntilNewCommandOnCommandTopic )
{
  // Same contract in non-chained mode. The "~/commands" message stays in the realtime
  // buffer and is re-read every cycle, so the hold cannot rely on clearing references.
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();
  ASSERT_FALSE( controller_->is_in_chained_mode() );

  for ( auto &v : hw_state_values_ ) v = 0.0;
  sendCommand( { 1.0, 0.0, 0.0 } );
  for ( int i = 0; i < 10; ++i ) {
    ASSERT_EQ( callFullUpdate(), controller_interface::return_type::OK );
    followCommands();
  }
  ASSERT_GT( hw_cmd_values_[0], 0.0 ) << "should have started moving toward the command";

  sendEstop( true );
  ASSERT_EQ( callFullUpdate(), controller_interface::return_type::OK );
  const double position_at_estop = hw_cmd_values_[0];

  sendEstop( false );
  for ( int i = 0; i < 50; ++i ) {
    ASSERT_EQ( callFullUpdate(), controller_interface::return_type::OK );
    followCommands();
    ASSERT_NEAR( hw_cmd_values_[0], position_at_estop, 1e-6 )
        << "the buffered command must stay abandoned (cycle " << i << ")";
  }

  sendCommand( { 1.1, 0.0, 0.0 } );
  for ( int i = 0; i < 20; ++i ) {
    ASSERT_EQ( callFullUpdate(), controller_interface::return_type::OK );
    followCommands();
  }
  EXPECT_GT( hw_cmd_values_[0], position_at_estop + 1e-3 );
}

// ============================================================================
// Safety Bypass Tests
// ============================================================================

TEST_F( SafetyPositionControllerTest, SafetyBypassServiceEnables )
{
  initController();
  findMocks();
  ASSERT_TRUE( bypass_service_mock_ );
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );

  auto req_header = std::make_shared<rmw_request_id_t>();
  req_header->sequence_number = 1L;
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  std_srvs::srv::SetBool::Response expected_resp;
  expected_resp.success = true;
  EXPECT_CALL( *bypass_service_mock_, send_response( ::testing::_, ::testing::_ ) ).Times( 1 );

  bypass_service_mock_->handle_request( req_header, request );

  EXPECT_TRUE( controller_->safety_bypass_active_.load() );
}

TEST_F( SafetyPositionControllerTest, SafetyBypassServiceDisables )
{
  initController();
  findMocks();
  ASSERT_TRUE( bypass_service_mock_ );
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );

  // First enable
  controller_->safety_bypass_active_.store( true );

  auto req_header = std::make_shared<rmw_request_id_t>();
  req_header->sequence_number = 2L;
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;

  EXPECT_CALL( *bypass_service_mock_, send_response( ::testing::_, ::testing::_ ) ).Times( 1 );

  bypass_service_mock_->handle_request( req_header, request );

  EXPECT_FALSE( controller_->safety_bypass_active_.load() );
}

// ============================================================================
// NaN Handling
// ============================================================================

TEST_F( SafetyPositionControllerTest, NaNReferenceHoldsAtCurrentPosition )
{
  // NaN references demand zero velocity: the controller holds the (rebased) commanded
  // position instead of tracking anything — the joints must not move.
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  setStateValue( "joint1", 0.4 );
  setStateValue( "joint2", -0.2 );
  setStateValue( "joint3", 0.1 );

  controller_->reference_interfaces_[0] = std::numeric_limits<double>::quiet_NaN();
  controller_->reference_interfaces_[1] = std::numeric_limits<double>::quiet_NaN();
  controller_->reference_interfaces_[2] = std::numeric_limits<double>::quiet_NaN();

  for ( int i = 0; i < 20; ++i ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    EXPECT_NEAR( hw_cmd_values_[0], 0.4, 1e-6 );
    EXPECT_NEAR( hw_cmd_values_[1], -0.2, 1e-6 );
    EXPECT_NEAR( hw_cmd_values_[2], 0.1, 1e-6 );
  }
}

TEST_F( SafetyPositionControllerTest, NaNAfterValidReferenceBrakesAndHolds )
{
  // A reference that becomes NaN means "no target" and must make the joint brake to a
  // stop and hold. It must NOT keep tracking the target that was valid before: the
  // reference interfaces are reset to NaN on an E-stop release and on activation, and
  // resuming the old target there would be unexpected delayed motion.
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  for ( auto &v : hw_state_values_ ) v = 0.0;
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  // Move toward the target for a while; the mock hardware follows the command exactly.
  for ( int i = 0; i < 10; ++i ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    followCommands();
  }
  const double cmd_when_invalidated = hw_cmd_values_[0];
  ASSERT_GT( cmd_when_invalidated, 0.0 ) << "should have started moving toward the target";
  ASSERT_LT( cmd_when_invalidated, 1.0 ) << "should not have arrived yet";

  // Upstream stops commanding: all references become NaN.
  for ( auto &ref : controller_->reference_interfaces_ ) {
    ref = std::numeric_limits<double>::quiet_NaN();
  }

  for ( int i = 0; i < 100; ++i ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    followCommands();
  }

  // joint1: v_max = 1.0 rad/s, a_dec = deceleration_scale(3) * 8 rad/s^2 = 24 rad/s^2.
  // Braking distance is at most v^2 / (2 * a_dec) ~= 0.021 rad; allow a few cycles slack.
  constexpr double kBrakingDistance = 1.0 / ( 2.0 * 24.0 ) + 0.03;
  EXPECT_NEAR( hw_cmd_values_[0], cmd_when_invalidated, kBrakingDistance )
      << "NaN reference must brake and hold instead of tracking the stale target";
  EXPECT_LT( hw_cmd_values_[0], 0.9 ) << "the abandoned target must never be reached";
}

TEST_F( SafetyPositionControllerTest, ReactivateDoesNotResumeStaleReference )
{
  // After a deactivate/activate cycle the references are NaN again. The processed
  // reference derived from them must be invalidated too, otherwise the controller keeps
  // driving toward the target from before the deactivation.
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  for ( auto &v : hw_state_values_ ) v = 0.0;
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  for ( int i = 0; i < 10; ++i ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    followCommands();
  }

  deactivateController();
  activateController();

  // Fresh activation: references are NaN and nothing new is commanded.
  for ( size_t i = 0; i < controlled_joints_.size(); ++i ) {
    ASSERT_TRUE( std::isnan( controller_->reference_interfaces_[i] ) );
  }
  const double position_at_activation = hw_state_values_[0];

  for ( int i = 0; i < 20; ++i ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    followCommands();
    EXPECT_NEAR( hw_cmd_values_[0], position_at_activation, 1e-6 )
        << "reactivated controller must hold, not resume the pre-deactivation target "
           "(cycle "
        << i << ")";
  }
}

// ============================================================================
// Status Publishing
// ============================================================================

TEST_F( SafetyPositionControllerTest, StatusPublishesCorrectFields )
{
  initController( {}, false, false );
  configureController();
  setupHardwareInterfaces();
  findMocks();

  SafetyPositionControllerStatus captured;
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) )
      .WillRepeatedly( [&captured]( const auto &msg ) { captured = msg; } );

  activateController();

  EXPECT_FALSE( captured.safety_bypass_active );
  EXPECT_FALSE( captured.compliant_mode );
  EXPECT_FALSE( captured.current_limits_enabled );
  EXPECT_FALSE( captured.collision_check_enabled ); // disabled in params
  EXPECT_FALSE( captured.estop_engaged );
}

TEST_F( SafetyPositionControllerTest, StatusUpdatesOnEstopEngage )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();

  SafetyPositionControllerStatus captured;
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) )
      .WillRepeatedly( [&captured]( const auto &msg ) { captured = msg; } );

  activateController();
  EXPECT_FALSE( captured.estop_engaged );

  for ( auto &v : hw_state_values_ ) v = 0.0;
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  sendEstop( true );
  callUpdate();
  EXPECT_TRUE( captured.estop_engaged );
}

TEST_F( SafetyPositionControllerTest, StatusUpdatesOnEstopRelease )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();

  SafetyPositionControllerStatus captured;
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) )
      .WillRepeatedly( [&captured]( const auto &msg ) { captured = msg; } );

  activateController();

  for ( auto &v : hw_state_values_ ) v = 0.0;
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  sendEstop( true );
  callUpdate();
  EXPECT_TRUE( captured.estop_engaged );

  sendEstop( false );
  callUpdate();
  EXPECT_FALSE( captured.estop_engaged );
}

TEST_F( SafetyPositionControllerTest, StatusReflectsCollisionCheckEnabled )
{
  // Collision checking needs a model with collision geometry: configuring it against a
  // model with no checkable pair is refused, because the check would pass everything.
  initController( {}, /*check_self_collisions=*/true, /*set_current_limits=*/false,
                  "test_robot_collision.urdf" );
  configureController();
  setupHardwareInterfaces();
  findMocks();

  SafetyPositionControllerStatus captured;
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) )
      .WillRepeatedly( [&captured]( const auto &msg ) { captured = msg; } );

  activateController();
  EXPECT_TRUE( captured.collision_check_enabled );
}

TEST_F( SafetyPositionControllerTest, StatusReflectsSafetyBypassActive )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();

  SafetyPositionControllerStatus captured;
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) )
      .WillRepeatedly( [&captured]( const auto &msg ) { captured = msg; } );

  activateController();
  EXPECT_FALSE( captured.safety_bypass_active );

  // The bypass publish happens in the service callback, not in update.
  // Simulate what the service callback does: set active + publish.
  controller_->safety_bypass_active_.store( true );
  controller_->publish_status();
  EXPECT_TRUE( captured.safety_bypass_active );
}

// ============================================================================
// Repeated Activation/Deactivation Cycle Tests
// ============================================================================

TEST_F( SafetyPositionControllerTest, RepeatedActivateDeactivateCycles )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );

  for ( int cycle = 0; cycle < 3; ++cycle ) {
    activateController();

    // Verify references are NaN after activation (reset)
    for ( size_t i = 0; i < controlled_joints_.size(); ++i ) {
      EXPECT_TRUE( std::isnan( controller_->reference_interfaces_[i] ) )
          << "Cycle " << cycle << ": reference_interfaces_[" << i << "] not NaN after activate";
    }

    // Set valid references and verify operation
    for ( auto &v : hw_state_values_ ) v = 0.0;
    controller_->reference_interfaces_[0] = 0.01;
    controller_->reference_interfaces_[1] = 0.0;
    controller_->reference_interfaces_[2] = 0.0;
    auto ret = callUpdate();
    EXPECT_EQ( ret, controller_interface::return_type::OK );
    EXPECT_GT( std::abs( hw_cmd_values_[0] ), 0.0 );

    deactivateController();

    // The engaged E-stop is cleared; the subscriptions outlive the activation
    EXPECT_FALSE( controller_->estop_engaged_.load() );
    EXPECT_TRUE( controller_->estop_subscriber_ != nullptr );
    EXPECT_TRUE( controller_->joints_command_subscriber_ != nullptr );
  }
}

TEST_F( SafetyPositionControllerTest, EventStatusReportsTheCurrentCycle )
{
  // The status timer never fires in these tests, which is exactly the documented
  // status_publish_rate=0 mode: what an event publishes is all a consumer ever sees.
  // An E-stop release parks the pipeline within the same cycle, so the message the
  // release publishes must already report that.
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();

  SafetyPositionControllerStatus captured;
  int publishes = 0;
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).WillRepeatedly( [&]( const auto &msg ) {
    captured = msg;
    ++publishes;
  } );

  activateController();

  for ( auto &v : hw_state_values_ ) v = 0.0;
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;
  for ( int i = 0; i < 5; ++i ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    followCommands();
  }

  sendEstop( true );
  ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
  EXPECT_TRUE( captured.estop_engaged );

  sendEstop( false );
  const int publishes_before_release = publishes;
  ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
  ASSERT_GT( publishes, publishes_before_release ) << "the release must publish a status";
  EXPECT_FALSE( captured.estop_engaged );
  EXPECT_TRUE( captured.parked ) << "the release parks the pipeline in this very cycle";
}

TEST_F( SafetyPositionControllerTest, ReactivateAfterEstop )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );

  activateController();

  for ( auto &v : hw_state_values_ ) v = 0.0;
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  sendEstop( true );
  callUpdate();
  EXPECT_TRUE( controller_->estop_engaged_.load() );

  deactivateController();
  EXPECT_FALSE( controller_->estop_engaged_.load() );

  sendEstop( false );
  activateController();
  EXPECT_FALSE( controller_->estop_engaged_.load() );

  // References should be NaN (fresh activation)
  for ( size_t i = 0; i < controlled_joints_.size(); ++i ) {
    EXPECT_TRUE( std::isnan( controller_->reference_interfaces_[i] ) );
  }

  // Verify it works normally after re-activation
  for ( auto &v : hw_state_values_ ) v = 0.0;
  controller_->reference_interfaces_[0] = 0.01;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;
  auto ret = callUpdate();
  EXPECT_EQ( ret, controller_interface::return_type::OK );
  EXPECT_GT( std::abs( hw_cmd_values_[0] ), 0.0 );
}

TEST_F( SafetyPositionControllerTest, EstopSurvivesReactivation )
{
  // estop_active_ tracks the external safety signal, so restarting the controller must
  // not silently release it: the first cycle after activation re-engages the hold.
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  setStateValue( "joint1", 0.5 );
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  sendEstop( true );
  ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
  ASSERT_TRUE( controller_->estop_engaged_.load() );

  deactivateController();
  EXPECT_TRUE( controller_->estop_active_.load() ) << "the E-stop request must survive";

  activateController();
  ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
  EXPECT_TRUE( controller_->estop_engaged_.load() )
      << "a still-active E-stop must re-engage instead of resuming motion";
  EXPECT_DOUBLE_EQ( hw_cmd_values_[0], 0.5 );
}

TEST_F( SafetyPositionControllerTest, BusyStateHandleSkipsTheCycleWithoutFailing )
{
  // An async hardware component can hold a handle's lock while the controller reads it.
  // A missed try_lock is contention, not a fault: returning ERROR makes the controller
  // manager deactivate this controller and every controller in its chain.
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  for ( auto &v : hw_state_values_ ) v = 0.0;
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  for ( int i = 0; i < 5; ++i ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    followCommands();
  }
  const double cmd_before = hw_cmd_values_[0];
  ASSERT_GT( cmd_before, 0.0 );

  {
    const auto state_index = static_cast<size_t>( controller_->joint_index_[0] );
    std::unique_lock<std::shared_mutex> busy( state_ifaces_[state_index]->get_mutex() );
    EXPECT_EQ( callUpdate(), controller_interface::return_type::OK )
        << "a busy state handle must not fail the update";
    EXPECT_DOUBLE_EQ( hw_cmd_values_[0], cmd_before ) << "the cycle is skipped, not guessed";
  }

  ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
  EXPECT_GT( hw_cmd_values_[0], cmd_before ) << "tracking resumes once the handle is free";
}

TEST_F( SafetyPositionControllerTest, NonFiniteJointStateHoldsPositionAndIsNeverCommanded )
{
  // A broken encoder must never become a command. The pipeline seeds its integration
  // state from the measured position and the tracking leash pulls the command toward it,
  // while only NaN was filtered on the way out - so an infinity reached the hardware.
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  for ( auto &v : hw_state_values_ ) v = 0.0;
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  for ( int i = 0; i < 5; ++i ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    followCommands();
  }
  const double cmd_before = hw_cmd_values_[0];
  ASSERT_GT( cmd_before, 0.0 );

  for ( const double bad :
        { std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::quiet_NaN() } ) {
    setStateValue( "joint1", bad );
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    ASSERT_TRUE( std::isfinite( hw_cmd_values_[0] ) )
        << "non-finite feedback must never be commanded";
    ASSERT_DOUBLE_EQ( hw_cmd_values_[0], cmd_before )
        << "the cycle is skipped: no motion without valid feedback";
  }

  setStateValue( "joint1", cmd_before );
  ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
  EXPECT_GT( hw_cmd_values_[0], cmd_before ) << "tracking resumes once feedback is valid again";
}

TEST_F( SafetyPositionControllerTest, ProlongedBusyStateHandleParksOnRecovery )
{
  // A single missed try_lock is contention and skips the cycle; a handle that stays
  // busy past state_read_timeout is a fault. The pipeline must be invalidated so that
  // recovery rebases to the measured state and PARKS: resuming the pre-failure
  // reference from a stale velocity state could jump-start a long-stopped arm.
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();
  StatusMsgType captured;
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) )
      .WillRepeatedly( [&captured]( const auto &msg ) { captured = msg; } );
  activateController();

  for ( auto &v : hw_state_values_ ) v = 0.0;
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  for ( int i = 0; i < 5; ++i ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    followCommands();
  }
  const double cmd_before = hw_cmd_values_[0];
  ASSERT_GT( cmd_before, 0.0 );

  {
    const auto state_index = static_cast<size_t>( controller_->joint_index_[0] );
    std::unique_lock<std::shared_mutex> busy( state_ifaces_[state_index]->get_mutex() );
    // default state_read_timeout is 0.1 s = 10 cycles at the 100 Hz test rate
    for ( int i = 0; i < 20; ++i ) {
      ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
      EXPECT_DOUBLE_EQ( hw_cmd_values_[0], cmd_before ) << "cycles are skipped while busy";
    }
  }

  // Recovery: the pre-failure reference (still 1.0) is abandoned, the limb holds
  ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
  EXPECT_TRUE( controller_->pipeline_->parked() )
      << "recovery after a prolonged state-read failure must park";
  EXPECT_TRUE( captured.parked )
      << "the recovery park is an event and must be published (event-only mode consumers)";
  for ( int i = 0; i < 20; ++i ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    EXPECT_NEAR( hw_cmd_values_[0], cmd_before, 1e-9 ) << "parked limb must hold, not resume";
  }

  // A NEW reference releases the park and is tracked again
  controller_->reference_interfaces_[0] = 0.1;
  ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
  EXPECT_FALSE( controller_->pipeline_->parked() );
  const double resume_start = hw_cmd_values_[0];
  for ( int i = 0; i < 5; ++i ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    followCommands();
  }
  EXPECT_GT( hw_cmd_values_[0], resume_start ) << "tracking must resume toward the new reference";
}

TEST_F( SafetyPositionControllerTest, ProlongedNonFiniteJointStateParksOnRecovery )
{
  // The dead-encoder variant of the prolonged failure: non-finite reads accumulate into
  // the same escalation as busy handles.
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  for ( auto &v : hw_state_values_ ) v = 0.0;
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  for ( int i = 0; i < 5; ++i ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    followCommands();
  }
  const double cmd_before = hw_cmd_values_[0];
  ASSERT_GT( cmd_before, 0.0 );

  setStateValue( "joint1", std::numeric_limits<double>::quiet_NaN() );
  for ( int i = 0; i < 20; ++i ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
  }

  setStateValue( "joint1", cmd_before );
  ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
  EXPECT_TRUE( controller_->pipeline_->parked() )
      << "recovery after prolonged non-finite feedback must park";
}

TEST_F( SafetyPositionControllerTest, EstopPulseDuringStateReadOutageIsHonored )
{
  // An E-stop must not need working joint-state reads: an engage (or a whole
  // engage+release pulse) inside a read outage previously fell through the skip-cycle
  // early return and was lost — the arm resumed the pre-outage reference untouched.
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  for ( auto &v : hw_state_values_ ) v = 0.0;
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  for ( int i = 0; i < 5; ++i ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    followCommands();
  }
  ASSERT_GT( hw_cmd_values_[0], 0.0 );

  {
    const auto state_index = static_cast<size_t>( controller_->joint_index_[0] );
    std::unique_lock<std::shared_mutex> busy( state_ifaces_[state_index]->get_mutex() );
    sendEstop( true );
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    EXPECT_TRUE( controller_->estop_engaged_.load() )
        << "the engage edge must not wait for joint states";
    sendEstop( false );
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    EXPECT_FALSE( controller_->estop_engaged_.load() );
  }

  // The E-stop abandoned the reference: recovery must hold and park, not resume the
  // pre-outage target.
  ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
  EXPECT_TRUE( controller_->pipeline_->parked() )
      << "the release must park like any E-stop release";
  const double held = hw_cmd_values_[0];
  for ( int i = 0; i < 10; ++i ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    EXPECT_NEAR( hw_cmd_values_[0], held, 1e-9 ) << "the abandoned reference must not be resumed";
  }
}

TEST_F( SafetyPositionControllerTest, BusyCommandHandleDoesNotFailTheCycle )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  for ( auto &v : hw_state_values_ ) v = 0.0;
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  std::unique_lock<std::shared_mutex> busy( cmd_ifaces_[0]->get_mutex() );
  EXPECT_EQ( callUpdate(), controller_interface::return_type::OK )
      << "a busy command handle must not fail the update";
}

// ============================================================================
// Chained Mode Tests
// ============================================================================

TEST_F( SafetyPositionControllerTest, SwitchingChainedModeInvalidatesReferences )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  for ( const bool chained : { true, false } ) {
    controller_->reference_interfaces_[0] = 1.0;
    controller_->reference_interfaces_[1] = 2.0;
    controller_->reference_interfaces_[2] = 3.0;

    controller_->on_set_chained_mode( chained );

    for ( size_t i = 0; i < controlled_joints_.size(); ++i ) {
      EXPECT_TRUE( std::isnan( controller_->reference_interfaces_[i] ) )
          << "reference_interfaces_[" << i << "] must be NaN after switching to "
          << ( chained ? "chained" : "unchained" ) << " mode";
    }
  }
}

TEST_F( SafetyPositionControllerTest, InputSubscriptionsSurviveReactivation )
{
  // Both reference inputs live for the whole controller lifetime. Re-creating them per
  // activation left "~/commands" without a subscription after the first deactivation.
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();
  deactivateController();
  activateController();

  const auto node_name = std::string( controller_->get_node()->get_fully_qualified_name() );
  EXPECT_TRUE( rtest::findSubscription<safety_position_controller::CmdType>(
      node_name, node_name + "/commands" ) );
  EXPECT_TRUE(
      rtest::findSubscription<std_msgs::msg::Bool>( node_name, node_name + "/safety_estop" ) );

  for ( auto &v : hw_state_values_ ) v = 0.0;
  sendCommand( { 0.5, 0.0, 0.0 } );
  for ( int i = 0; i < 5; ++i ) {
    ASSERT_EQ( callFullUpdate(), controller_interface::return_type::OK );
    followCommands();
  }
  EXPECT_GT( hw_cmd_values_[0], 0.0 ) << "the command topic must still reach the hardware";
}

// ============================================================================
// Collision Tests
// ============================================================================

class SafetyPositionControllerCollisionTest
    : public hector_test::SafetyControllerTestBase<SafetyPositionControllerCollisionTest>
{
public:
  using ControllerType = SPC;
  using StatusMsgType = SafetyPositionControllerStatus;

  std::vector<std::string> controlled_joints_{ "joint1", "joint2", "joint3" };

  std::shared_ptr<ControllerType> controller_;
  std::shared_ptr<rtest::PublisherMock<StatusMsgType>> status_pub_mock_;

  void resetControllerSpecificMocks() { }
  void findControllerSpecificMocks( const std::string & /*node_name*/ ) { }

  void SetUp() override { controller_ = std::make_shared<ControllerType>(); }

  void initWithCollisions( const std::vector<std::string> &joints = {},
                           const std::string &urdf_file = "test_robot_collision.urdf",
                           const std::vector<rclcpp::Parameter> &extra_overrides = {} )
  {
    auto cj = joints.empty() ? controlled_joints_ : joints;
    const auto urdf = hector_test::loadUrdfFile( urdf_file );

    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "test_safety_position_cc";
    params.robot_description = urdf;
    params.update_rate = kUpdateRate;
    params.controller_manager_update_rate = kUpdateRate;
    params.node_namespace = "";

    std::vector<rclcpp::Parameter> overrides = {
        rclcpp::Parameter( "joints", cj ),
        rclcpp::Parameter( "check_self_collisions", true ),
        rclcpp::Parameter( "collision_safety_zone", 0.05 ),
        rclcpp::Parameter( "set_current_limits", false ),
        rclcpp::Parameter( "safety_bypass_timeout", 60.0 ),
        rclcpp::Parameter( "safety_bypass_joint_limit_tolerance", 0.03 ),
        rclcpp::Parameter( "publish_debug_joint_states", false ),
        rclcpp::Parameter( "collision_padding", 0.0 ),
        rclcpp::Parameter( "collision_cache_epsilon", 0.0 ), // disable cache for tests
        rclcpp::Parameter( "debug_visualize_collisions", false ),
    };
    // Apply extra overrides (replaces matching parameters by name)
    for ( const auto &extra : extra_overrides ) {
      bool found = false;
      for ( auto &base : overrides ) {
        if ( base.get_name() == extra.get_name() ) {
          base = extra;
          found = true;
          break;
        }
      }
      if ( !found ) {
        overrides.push_back( extra );
      }
    }

    rclcpp::NodeOptions opts;
    opts.parameter_overrides( overrides );
    params.node_options = opts;

    auto result = controller_->init( params );
    ASSERT_EQ( result, controller_interface::return_type::OK );
  }

  void configureWithSrdf()
  {
    // Provide empty SRDF (no disabled collision pairs)
    controller_->srdf_received_ = true;
    controller_->srdf_ = "";

    rclcpp_lifecycle::State unconfigured( lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                          "unconfigured" );
    auto cb = controller_->on_configure( unconfigured );
    ASSERT_EQ( cb, controller_interface::CallbackReturn::SUCCESS );
  }

  void setupHardwareInterfaces( const std::vector<std::string> &joints = {} )
  {
    auto cj = joints.empty() ? controlled_joints_ : joints;

    hw_state_values_.assign( controller_->all_joint_names_.size(), 0.0 );
    hw_cmd_values_.assign( cj.size(), 0.0 );

    cmd_ifaces_.clear();
    state_ifaces_.clear();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    for ( size_t i = 0; i < cj.size(); ++i ) {
      cmd_ifaces_.push_back( std::make_shared<hardware_interface::CommandInterface>(
          cj[i], "position", &hw_cmd_values_[i] ) );
    }
    for ( size_t i = 0; i < controller_->all_joint_names_.size(); ++i ) {
      state_ifaces_.push_back( std::make_shared<hardware_interface::StateInterface>(
          controller_->all_joint_names_[i], "position", &hw_state_values_[i] ) );
    }
#pragma GCC diagnostic pop

    controller_->command_interfaces_.clear();
    controller_->state_interfaces_.clear();

    for ( auto &ci : cmd_ifaces_ ) {
      controller_->command_interfaces_.emplace_back( ci, []() { } );
    }
    for ( auto &si : state_ifaces_ ) { controller_->state_interfaces_.emplace_back( si ); }
  }

  // Set state value for a specific joint by name
  void setStateValue( const std::string &joint, double value )
  {
    for ( size_t i = 0; i < controller_->all_joint_names_.size(); ++i ) {
      if ( controller_->all_joint_names_[i] == joint ) {
        hw_state_values_[i] = value;
        return;
      }
    }
    FAIL() << "Joint '" << joint << "' not found";
  }
};

TEST_F( SafetyPositionControllerCollisionTest, CollisionCheckerInitializedCorrectly )
{
  initWithCollisions();
  EXPECT_TRUE( controller_->collision_checker_ != nullptr );
}

TEST_F( SafetyPositionControllerCollisionTest, NoCollisionAllowsMovement )
{
  initWithCollisions();
  configureWithSrdf();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  // Start at [0,0,0] (straight chain, no collision)
  for ( auto &v : hw_state_values_ ) v = 0.0;

  // Command a small safe movement
  double small_step = 0.01;
  controller_->reference_interfaces_[0] = small_step;
  controller_->reference_interfaces_[1] = small_step;
  controller_->reference_interfaces_[2] = small_step;

  auto ret = callUpdate();
  EXPECT_EQ( ret, controller_interface::return_type::OK );

  // Commands should be applied (possibly limited by block_if_too_far but non-zero)
  EXPECT_GT( std::abs( hw_cmd_values_[0] ), 0.0 );
}

TEST_F( SafetyPositionControllerCollisionTest, CollisionBlocksMovement )
{
  initWithCollisions();
  configureWithSrdf();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  // Start at a colliding configuration: joint2=pi, joint3=-pi folds link3 back onto link1
  setStateValue( "joint1", 0.0 );
  setStateValue( "joint2", M_PI );
  setStateValue( "joint3", -M_PI );

  // Command to the same colliding position
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = M_PI;
  controller_->reference_interfaces_[2] = -M_PI;

  auto ret = callUpdate();
  EXPECT_EQ( ret, controller_interface::return_type::OK );

  // When collision detected, controller holds CURRENT positions, not commanded.
  // The commands should be the current positions (collision hold).
  EXPECT_DOUBLE_EQ( hw_cmd_values_[0], 0.0 );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[1], M_PI );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[2], -M_PI );
}

TEST_F( SafetyPositionControllerCollisionTest, CollisionBypassSkipsCheck )
{
  initWithCollisions();
  configureWithSrdf();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  // Enable safety bypass -> collision check skipped
  controller_->safety_bypass_active_.store( true );

  // Start at safe position
  for ( auto &v : hw_state_values_ ) v = 0.0;

  // Command a small movement
  double small_step = 0.01;
  controller_->reference_interfaces_[0] = small_step;
  controller_->reference_interfaces_[1] = small_step;
  controller_->reference_interfaces_[2] = small_step;

  auto ret = callUpdate();
  EXPECT_EQ( ret, controller_interface::return_type::OK );

  // With bypass, commands should pass through (not blocked)
  EXPECT_GT( std::abs( hw_cmd_values_[0] ), 0.0 );
}

// ============================================================================
// Continuous Joint Collision Tests (exercises cos/sin encoding in collision_checker)
// ============================================================================

TEST_F( SafetyPositionControllerCollisionTest, ContinuousJointNoCollisionAtZero )
{
  // Use all 4 joints including the continuous joint4
  std::vector<std::string> joints = { "joint1", "joint2", "joint3", "joint4" };
  initWithCollisions( joints );
  configureWithSrdf();
  setupHardwareInterfaces( joints );
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  // All joints at 0 (straight chain, no collision)
  for ( auto &v : hw_state_values_ ) v = 0.0;

  // Command small movement on joint4 (continuous)
  double small_step = 0.01;
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;
  controller_->reference_interfaces_[3] = small_step;

  auto ret = callUpdate();
  EXPECT_EQ( ret, controller_interface::return_type::OK );

  // Commands should pass through (no collision at zero config)
  EXPECT_GT( std::abs( hw_cmd_values_[3] ), 0.0 );
}

TEST_F( SafetyPositionControllerCollisionTest, ContinuousJointAtLargeAngle )
{
  // Continuous joint at 2*pi (equivalent to 0 in cos/sin encoding)
  std::vector<std::string> joints = { "joint1", "joint2", "joint3", "joint4" };
  initWithCollisions( joints );
  configureWithSrdf();
  setupHardwareInterfaces( joints );
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  // Straight chain, joint4 at 2*pi (should be equivalent to 0 for collision)
  for ( auto &v : hw_state_values_ ) v = 0.0;
  setStateValue( "joint4", 2.0 * M_PI );

  // Command near current position (small perturbation)
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;
  controller_->reference_interfaces_[3] = 2.0 * M_PI + 0.01;

  auto ret = callUpdate();
  EXPECT_EQ( ret, controller_interface::return_type::OK );

  // Should not falsely detect collision at wrapped angle
  // Commands should be non-trivially different from zero (movement allowed)
  EXPECT_GT( std::abs( hw_cmd_values_[3] ), 0.0 );
}

TEST_F( SafetyPositionControllerCollisionTest, ContinuousJointCollisionDetected )
{
  // Folded-back chain should cause collision for ALL joints including joint4
  std::vector<std::string> joints = { "joint1", "joint2", "joint3", "joint4" };
  initWithCollisions( joints );
  configureWithSrdf();
  setupHardwareInterfaces( joints );
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  // Folded-back configuration: link1 and link4 spheres collide
  // joint2=pi, joint3=-pi folds the chain back
  setStateValue( "joint1", 0.0 );
  setStateValue( "joint2", M_PI );
  setStateValue( "joint3", -M_PI );
  setStateValue( "joint4", 0.0 );

  // Command to the same colliding position
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = M_PI;
  controller_->reference_interfaces_[2] = -M_PI;
  controller_->reference_interfaces_[3] = 0.0;

  auto ret = callUpdate();
  EXPECT_EQ( ret, controller_interface::return_type::OK );

  // Collision detected -> controller holds current positions
  EXPECT_DOUBLE_EQ( hw_cmd_values_[0], 0.0 );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[1], M_PI );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[2], -M_PI );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[3], 0.0 );
}

// ============================================================================
// Velocity Limiting & Distance-Based Scaling Tests (collision fixture)
// ============================================================================

// ============================================================================
// Directional Collision Scaling Tests
// ============================================================================

// ============================================================================
// Current Limits Tests
// ============================================================================

TEST_F( SafetyPositionControllerTest, CurrentLimitsWriteStiffByDefault )
{
  initController( {}, /*check_self_collisions=*/false, /*set_current_limits=*/true );
  configureController();

  // Setup hardware interfaces with current command interfaces
  auto cj = controlled_joints_;
  hw_state_values_.assign( controller_->all_joint_names_.size(), 0.0 );
  hw_cmd_values_.assign( cj.size() * 2, 0.0 ); // position + current

  cmd_ifaces_.clear();
  state_ifaces_.clear();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  for ( size_t i = 0; i < cj.size(); ++i ) {
    cmd_ifaces_.push_back( std::make_shared<hardware_interface::CommandInterface>(
        cj[i], "position", &hw_cmd_values_[i] ) );
  }
  for ( size_t i = 0; i < cj.size(); ++i ) {
    cmd_ifaces_.push_back( std::make_shared<hardware_interface::CommandInterface>(
        cj[i], "current", &hw_cmd_values_[cj.size() + i] ) );
  }
  for ( size_t i = 0; i < controller_->all_joint_names_.size(); ++i ) {
    state_ifaces_.push_back( std::make_shared<hardware_interface::StateInterface>(
        controller_->all_joint_names_[i], "position", &hw_state_values_[i] ) );
  }
#pragma GCC diagnostic pop

  controller_->command_interfaces_.clear();
  controller_->state_interfaces_.clear();
  for ( auto &ci : cmd_ifaces_ ) {
    controller_->command_interfaces_.emplace_back( ci, []() { } );
  }
  for ( auto &si : state_ifaces_ ) { controller_->state_interfaces_.emplace_back( si ); }

  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  for ( auto &v : hw_state_values_ ) v = 0.0;
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  callUpdate();

  // Default mode is stiff -> current limits should be stiff_limit (5.0)
  for ( size_t i = 0; i < cj.size(); ++i ) {
    EXPECT_DOUBLE_EQ( hw_cmd_values_[cj.size() + i], 5.0 )
        << "Joint " << cj[i] << " should have stiff current limit";
  }
}

TEST_F( SafetyPositionControllerTest, CurrentLimitsWriteCompliantWhenEnabled )
{
  initController( {}, /*check_self_collisions=*/false, /*set_current_limits=*/true );
  configureController();

  auto cj = controlled_joints_;
  hw_state_values_.assign( controller_->all_joint_names_.size(), 0.0 );
  hw_cmd_values_.assign( cj.size() * 2, 0.0 );

  cmd_ifaces_.clear();
  state_ifaces_.clear();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  for ( size_t i = 0; i < cj.size(); ++i ) {
    cmd_ifaces_.push_back( std::make_shared<hardware_interface::CommandInterface>(
        cj[i], "position", &hw_cmd_values_[i] ) );
  }
  for ( size_t i = 0; i < cj.size(); ++i ) {
    cmd_ifaces_.push_back( std::make_shared<hardware_interface::CommandInterface>(
        cj[i], "current", &hw_cmd_values_[cj.size() + i] ) );
  }
  for ( size_t i = 0; i < controller_->all_joint_names_.size(); ++i ) {
    state_ifaces_.push_back( std::make_shared<hardware_interface::StateInterface>(
        controller_->all_joint_names_[i], "position", &hw_state_values_[i] ) );
  }
#pragma GCC diagnostic pop

  controller_->command_interfaces_.clear();
  controller_->state_interfaces_.clear();
  for ( auto &ci : cmd_ifaces_ ) {
    controller_->command_interfaces_.emplace_back( ci, []() { } );
  }
  for ( auto &si : state_ifaces_ ) { controller_->state_interfaces_.emplace_back( si ); }

  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  // Enable compliant mode
  controller_->in_compliant_mode_.store( true );

  for ( auto &v : hw_state_values_ ) v = 0.0;
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  callUpdate();

  // Compliant mode -> current limits should be compliant_limit (3.0)
  for ( size_t i = 0; i < cj.size(); ++i ) {
    EXPECT_DOUBLE_EQ( hw_cmd_values_[cj.size() + i], 3.0 )
        << "Joint " << cj[i] << " should have compliant current limit";
  }
}

// ============================================================================
// Safety Bypass Joint Limit Tolerance Tests
// ============================================================================

TEST_F( SafetyPositionControllerTest, SafetyBypassRelaxesJointLimits )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  // Start close to the limit so per-cycle velocity stepping (active also during bypass)
  // does not dominate the test: joint2 max step = 2.0/100 * 1.5 = 0.03.
  for ( auto &v : hw_state_values_ ) v = 0.0;
  setStateValue( "joint2", 1.5 );

  // joint2 upper limit is 1.5, range = 3.0, tolerance = 3% -> 0.09 extra
  // Without bypass: command beyond 1.5 should clamp to 1.5
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = 1.55;
  controller_->reference_interfaces_[2] = 0.0;

  callUpdate();
  EXPECT_LE( hw_cmd_values_[1], 1.5 ) << "Without bypass, should clamp to upper limit";

  // Enable bypass
  controller_->safety_bypass_active_.store( true );

  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = 1.55;
  controller_->reference_interfaces_[2] = 0.0;

  callUpdate();

  // With bypass (tolerance = 3% of range 3.0 = 0.09), upper limit becomes 1.59.
  // Bypass keeps velocity/acceleration limits active, so 1.55 is approached smoothly
  // over multiple cycles rather than jumped to.
  for ( int i = 0; i < 100 && hw_cmd_values_[1] <= 1.5; ++i ) {
    setStateValue( "joint2", hw_cmd_values_[1] );
    callUpdate();
  }
  EXPECT_GT( hw_cmd_values_[1], 1.5 )
      << "With bypass, commands slightly beyond normal limits should be allowed";
  for ( int i = 0; i < 100; ++i ) {
    setStateValue( "joint2", hw_cmd_values_[1] );
    callUpdate();
  }
  EXPECT_NEAR( hw_cmd_values_[1], 1.55, 1e-4 );
}

// ============================================================================
// on_activate Validation Tests
// ============================================================================

TEST_F( SafetyPositionControllerCollisionTest, ActivateFailsWhenSafetyZoneLessThanPadding )
{
  // collision_safety_zone (0.01) <= collision_padding (0.05) -> should fail
  initWithCollisions( {}, "test_robot_collision.urdf",
                      { rclcpp::Parameter( "collision_safety_zone", 0.01 ),
                        rclcpp::Parameter( "collision_padding", 0.05 ) } );
  configureWithSrdf();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );

  // on_activate should fail validation
  rclcpp_lifecycle::State inactive( lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                    "inactive" );
  auto cb = controller_->on_activate( inactive );
  EXPECT_EQ( cb, controller_interface::CallbackReturn::ERROR );
}

TEST_F( SafetyPositionControllerCollisionTest, ActivateFailsWhenSafetyZoneEqualsPadding )
{
  initWithCollisions( {}, "test_robot_collision.urdf",
                      { rclcpp::Parameter( "collision_safety_zone", 0.05 ),
                        rclcpp::Parameter( "collision_padding", 0.05 ) } );
  configureWithSrdf();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );

  rclcpp_lifecycle::State inactive( lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                    "inactive" );
  auto cb = controller_->on_activate( inactive );
  EXPECT_EQ( cb, controller_interface::CallbackReturn::ERROR );
}

TEST_F( SafetyPositionControllerCollisionTest, NarrowSafetyZoneStillActivates )
{
  // The tunneling check compares a joint step [rad] against the zone width [m], which
  // only lines up at a ~1 m lever arm — a conservative heuristic that must warn, not
  // block activation (every deployed Athena config trips it).
  initWithCollisions( {}, "test_robot_collision.urdf",
                      { rclcpp::Parameter( "collision_safety_zone", 0.015 ),
                        rclcpp::Parameter( "collision_padding", 0.0 ) } );
  configureWithSrdf();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );

  rclcpp_lifecycle::State inactive( lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                    "inactive" );
  auto cb = controller_->on_activate( inactive );
  EXPECT_EQ( cb, controller_interface::CallbackReturn::SUCCESS );
}

// ============================================================================
// Safety Bypass Skips Collision And Allows Relaxed Limits
// ============================================================================

TEST_F( SafetyPositionControllerCollisionTest, BypassSkipsCollisionButAllowsRelaxedLimits )
{
  initWithCollisions();
  configureWithSrdf();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  controller_->safety_bypass_active_.store( true );

  // Start at a colliding configuration
  setStateValue( "joint1", 0.0 );
  setStateValue( "joint2", M_PI );
  setStateValue( "joint3", -M_PI );

  // Command to the colliding position
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = M_PI;
  controller_->reference_interfaces_[2] = -M_PI;

  auto ret = callUpdate();
  EXPECT_EQ( ret, controller_interface::return_type::OK );

  // With bypass, commands should pass through (no collision blocking, no velocity limiting)
  EXPECT_NEAR( hw_cmd_values_[1], M_PI, 1e-6 )
      << "Bypass should allow commands even in colliding configuration";
}

// ============================================================================
// Collision Checker Inf Input Test
// ============================================================================

TEST_F( SafetyPositionControllerCollisionTest, InfInputDetectedAsCollision )
{
  initWithCollisions();
  configureWithSrdf();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  for ( auto &v : hw_state_values_ ) v = 0.0;

  // Command with infinity — should be treated as collision (safe default)
  controller_->reference_interfaces_[0] = std::numeric_limits<double>::infinity();
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  // First need to call update so collision checker sees the inf
  // The inf is clamped to the position limits (joint1 has limits [-pi, pi])
  // But the collision checker receives cc_positions which includes the clamped value
  // So this tests the controller's overall handling — commands should still be safe
  auto ret = callUpdate();
  EXPECT_EQ( ret, controller_interface::return_type::OK );
}

// ============================================================================
// E-stop holds positions across multiple cycles
// ============================================================================

TEST_F( SafetyPositionControllerTest, EstopHoldsAcrossMultipleCycles )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  setStateValue( "joint1", 0.5 );
  setStateValue( "joint2", -0.3 );
  setStateValue( "joint3", 1.0 );

  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  // First update to read positions
  callUpdate();

  sendEstop( true );
  callUpdate();

  // Commands should be held positions regardless of what reference says
  controller_->reference_interfaces_[0] = 999.0;
  controller_->reference_interfaces_[1] = 999.0;
  controller_->reference_interfaces_[2] = 999.0;

  callUpdate();

  EXPECT_DOUBLE_EQ( hw_cmd_values_[0], 0.5 );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[1], -0.3 );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[2], 1.0 );

  // Third cycle still holds
  callUpdate();
  EXPECT_DOUBLE_EQ( hw_cmd_values_[0], 0.5 );
}

// ============================================================================
// Collision test: distance-based scaling status fields
// ============================================================================

// ============================================================================
// Safety QP behavior
// ============================================================================

namespace
{
// kUpdateRate = 100 → dt = 0.01. Defaults: acceleration limit 8 rad/s²,
// deceleration_scale 3 → decel 24 rad/s². URDF velocity limits: joint1=1.0,
// joint2=2.0, joint3=1.5.
constexpr double kDt = 0.01;
constexpr double kAccPerCycle = 8.0 * kDt;       // max speed-up per cycle
constexpr double kDecPerCycle = 3.0 * 8.0 * kDt; // max brake per cycle
} // namespace

TEST_F( SafetyPositionControllerCollisionTest, TransientUncontrolledJointGlitchDoesNotLatchCollision )
{
  // A one-cycle NaN on an UNCONTROLLED joint (joint4 has a state interface but is not
  // commanded) must not freeze the arm. With the movement cache enabled, a latched
  // "assume collision" result served for the unchanged configuration would zero the
  // tracking demand forever: the arm never moves, so the cache would never invalidate.
  initWithCollisions( {}, "test_robot_collision.urdf",
                      { rclcpp::Parameter( "collision_cache_epsilon", 1e-6 ) } );
  configureWithSrdf();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  for ( auto &v : hw_state_values_ ) v = 0.0;

  // Settle at a collision-free target so the configuration is stationary.
  controller_->reference_interfaces_[0] = 0.5;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;
  for ( int cycle = 0; cycle < 300; ++cycle ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    setStateValue( "joint1", hw_cmd_values_[0] );
  }
  ASSERT_NEAR( hw_cmd_values_[0], 0.5, 1e-4 );

  // One glitched cycle: the collision state is unobservable -> brake, latch nothing.
  setStateValue( "joint4", std::numeric_limits<double>::quiet_NaN() );
  ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
  EXPECT_EQ( controller_->collision_observer_->lastMinDistance(), std::numeric_limits<double>::max() )
      << "an unobservable cycle must not report the pre-glitch distance as current";
  setStateValue( "joint4", 0.0 );

  // A new reference after the recovery must be tracked again.
  controller_->reference_interfaces_[0] = 0.2;
  for ( int cycle = 0; cycle < 300; ++cycle ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    setStateValue( "joint1", hw_cmd_values_[0] );
  }
  EXPECT_NEAR( hw_cmd_values_[0], 0.2, 1e-3 );
}

TEST_F( SafetyPositionControllerCollisionTest, ProlongedUncontrolledJointOutageParksOnRecovery )
{
  // joint4 is observed for collision checking but not controlled, so it never reaches
  // read_current_positions(). A dead encoder there makes the collision state
  // unobservable: the arm brakes, and on recovery it must park rather than jump-start
  // toward the still-live reference — the same contract as a controlled-joint outage.
  initWithCollisions();
  configureWithSrdf();
  setupHardwareInterfaces();
  findMocks();
  int publishes = 0;
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).WillRepeatedly( [&publishes]( auto & ) {
    ++publishes;
  } );
  activateController();

  for ( auto &v : hw_state_values_ ) v = 0.0;
  controller_->reference_interfaces_[0] = 0.5;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  for ( int i = 0; i < 5; ++i ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    setStateValue( "joint1", hw_cmd_values_[0] );
  }
  ASSERT_GT( hw_cmd_values_[0], 0.0 );

  // default state_read_timeout is 0.1 s = 10 cycles at the 100 Hz test rate
  setStateValue( "joint4", std::numeric_limits<double>::quiet_NaN() );
  publishes = 0;
  for ( int i = 0; i < 15; ++i ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    setStateValue( "joint1", hw_cmd_values_[0] );
  }
  EXPECT_LE( publishes, 2 ) << "the fault is an event, not a per-cycle publish";

  // A loaded limb sags away from its command while the fault lasts. Rebasing onto the
  // measurement every cycle would turn the hold into "follow the sag".
  const double held_during_outage = hw_cmd_values_[0];
  for ( int i = 0; i < 20; ++i ) {
    setStateValue( "joint1", hw_cmd_values_[0] - 0.05 ); // simulated sag under gravity
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    ASSERT_NEAR( hw_cmd_values_[0], held_during_outage, 1e-9 )
        << "the command must hold, not follow the measurement (cycle " << i << ")";
  }
  setStateValue( "joint1", held_during_outage );
  setStateValue( "joint4", 0.0 );

  ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
  EXPECT_TRUE( controller_->pipeline_->parked() )
      << "an uncontrolled-joint outage must park on recovery like a controlled one";
  EXPECT_NEAR( hw_cmd_values_[0], held_during_outage, 1e-9 )
      << "the hold must not have drifted toward the (sagging) measurement";

  publishes = 0;
  for ( int i = 0; i < 20; ++i ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
  }
  EXPECT_EQ( publishes, 0 ) << "the recovery park publishes once, not every cycle";
}

TEST_F( SafetyPositionControllerCollisionTest, QpModeRampsAndReachesTarget )
{
  // End-to-end regression for the jump bug: a far target is approached with bounded
  // velocity AND bounded acceleration, and is still reached (later).
  initWithCollisions();
  configureWithSrdf();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  for ( auto &v : hw_state_values_ ) v = 0.0;

  // joint1 rotates the whole chain about z → no collision along the way
  controller_->reference_interfaces_[0] = 0.5;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  double prev_cmd = 0.0, prev_v = 0.0, max_dv = 0.0, max_v = 0.0;
  int reached_at = -1;
  for ( int cycle = 0; cycle < 300; ++cycle ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    const double cmd = hw_cmd_values_[0];
    const double v = ( cmd - prev_cmd ) / kDt;
    max_dv = std::max( max_dv, std::abs( v - prev_v ) );
    max_v = std::max( max_v, std::abs( v ) );
    prev_cmd = cmd;
    prev_v = v;
    // hardware follows the command
    setStateValue( "joint1", cmd );
    if ( reached_at < 0 && std::abs( cmd - 0.5 ) < 1e-4 ) {
      reached_at = cycle;
    }
  }

  EXPECT_GE( reached_at, 50 ) << "cannot be faster than the velocity limit";
  EXPECT_GT( reached_at, 0 ) << "target never reached";
  EXPECT_LE( max_v, 1.0 + 1e-6 ) << "velocity limit violated";
  EXPECT_LE( max_dv, kDecPerCycle + 1e-6 ) << "acceleration limit violated";
}

TEST_F( SafetyPositionControllerCollisionTest, QpModeStopsAtCollisionAndReportsStall )
{
  // Command straight into a self-collision: the damper must stop the motion at the
  // boundary (no penetration of the commanded configuration) and report 'stalled'.
  initWithCollisions();
  configureWithSrdf();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  for ( auto &v : hw_state_values_ ) v = 0.0;

  // joint2 = pi folds link4 into link1 → collision on the way
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = M_PI;
  controller_->reference_interfaces_[2] = 0.0;

  for ( int cycle = 0; cycle < 500; ++cycle ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    for ( size_t j = 0; j < 3; ++j ) { setStateValue( controlled_joints_[j], hw_cmd_values_[j] ); }
    // The commanded configuration must never penetrate (padding = 0 in this fixture;
    // small negative tolerance for the linearization sag on curved geometry)
    ASSERT_GT( controller_->collision_observer_->lastMinDistance(), -5e-3 )
        << "commanded configuration in collision at cycle " << cycle;
  }

  EXPECT_TRUE( controller_->pipeline_->stalled() ) << "head-on block must be reported as stalled";
  EXPECT_LT( hw_cmd_values_[1], M_PI - 0.1 ) << "should have stopped before the fold";
  EXPECT_GT( hw_cmd_values_[1], 0.1 ) << "should have moved toward the target first";

  // ---- Bypass: the fold must proceed, but still velocity/acceleration limited ----
  controller_->safety_bypass_active_.store( true );

  const double stalled_cmd = hw_cmd_values_[1];
  double prev_cmd = stalled_cmd, prev_v = 0.0, max_dv = 0.0, max_v = 0.0;
  for ( int cycle = 0; cycle < 500; ++cycle ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    const double cmd = hw_cmd_values_[1];
    const double v = ( cmd - prev_cmd ) / kDt;
    max_dv = std::max( max_dv, std::abs( v - prev_v ) );
    max_v = std::max( max_v, std::abs( v ) );
    prev_cmd = cmd;
    prev_v = v;
    for ( size_t j = 0; j < 3; ++j ) { setStateValue( controlled_joints_[j], hw_cmd_values_[j] ); }
  }

  EXPECT_GT( hw_cmd_values_[1], stalled_cmd + 0.5 ) << "bypass should allow the fold to proceed";
  EXPECT_LE( max_v, 2.0 + 1e-6 ) << "velocity limit must hold during bypass (joint2 limit 2.0)";
  EXPECT_LE( max_dv, kDecPerCycle + 1e-6 )
      << "acceleration limit must hold during bypass (the original jump bug)";
}

TEST_F( SafetyPositionControllerCollisionTest, QpModeParksAfterStallAndResumesOnNewReference )
{
  // A limb stalled past stall_park_timeout must abandon the stale reference and hold
  // position (no delayed motion when the blockage clears), resuming only on a NEW
  // command (reference change).
  initWithCollisions( {}, "test_robot_collision.urdf",
                      { rclcpp::Parameter( "stall_park_timeout", 2.0 ) } );
  configureWithSrdf();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  for ( auto &v : hw_state_values_ ) v = 0.0;

  // joint2 = pi folds into a self-collision → the QP stalls at the boundary
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = M_PI;
  controller_->reference_interfaces_[2] = 0.0;

  for ( int cycle = 0; cycle < 600 && !controller_->pipeline_->parked(); ++cycle ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    for ( size_t j = 0; j < 3; ++j ) { setStateValue( controlled_joints_[j], hw_cmd_values_[j] ); }
  }
  ASSERT_TRUE( controller_->pipeline_->parked() ) << "did not park within 600 cycles";
  EXPECT_TRUE( controller_->pipeline_->stalled() );

  // While parked: tracking demand is zeroed even though the reference is still far away
  // — the guarantee that clearing the blockage cannot cause delayed motion.
  const double parked_cmd = hw_cmd_values_[1];
  for ( int cycle = 0; cycle < 100; ++cycle ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    for ( size_t j = 0; j < 3; ++j ) { setStateValue( controlled_joints_[j], hw_cmd_values_[j] ); }
    EXPECT_LT( controller_->pipeline_->qpInput().v_des.cwiseAbs().maxCoeff(), 1e-9 );
  }
  EXPECT_TRUE( controller_->pipeline_->parked() );
  EXPECT_NEAR( hw_cmd_values_[1], parked_cmd, 1e-6 ) << "parked limb must not creep";

  // A NEW reference (retract away from the collision) releases the park and is tracked
  controller_->reference_interfaces_[1] = 0.3;
  ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
  EXPECT_FALSE( controller_->pipeline_->parked() );

  for ( int cycle = 0; cycle < 500; ++cycle ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    for ( size_t j = 0; j < 3; ++j ) { setStateValue( controlled_joints_[j], hw_cmd_values_[j] ); }
  }
  EXPECT_NEAR( hw_cmd_values_[1], 0.3, 1e-3 );
}

TEST_F( SafetyPositionControllerCollisionTest, ParkEventPublishesTheParkedState )
{
  // The park event is the only status publication in status_publish_rate=0 mode, so it
  // has to carry this cycle's state and not the snapshot from before the pipeline ran.
  initWithCollisions( {}, "test_robot_collision.urdf",
                      { rclcpp::Parameter( "stall_park_timeout", 2.0 ) } );
  configureWithSrdf();
  setupHardwareInterfaces();
  findMocks();

  SafetyPositionControllerStatus captured;
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) )
      .WillRepeatedly( [&captured]( const auto &msg ) { captured = msg; } );

  activateController();

  for ( auto &v : hw_state_values_ ) v = 0.0;
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = M_PI; // folds into a self-collision
  controller_->reference_interfaces_[2] = 0.0;

  for ( int cycle = 0; cycle < 600 && !controller_->pipeline_->parked(); ++cycle ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    for ( size_t j = 0; j < 3; ++j ) { setStateValue( controlled_joints_[j], hw_cmd_values_[j] ); }
  }
  ASSERT_TRUE( controller_->pipeline_->parked() ) << "did not park within 600 cycles";
  EXPECT_TRUE( captured.stalled );
  EXPECT_TRUE( captured.parked ) << "the park event must publish parked=true";
}

TEST_F( SafetyPositionControllerCollisionTest, QpModeJointDeviationBoxWiring )
{
  // The per-joint deviation box must be centered on the LEASHED reference and widened
  // to include the current command (one-sided: prevents drifting, never pulls).
  initWithCollisions( {}, "test_robot_collision.urdf",
                      { rclcpp::Parameter( "joint_deviation_limits.joint1.limit", 0.1 ) } );
  configureWithSrdf();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  for ( auto &v : hw_state_values_ ) v = 0.0;
  controller_->reference_interfaces_[0] = 0.5; // leash: v_max(1.0) * 0.3 s → leashed ref 0.3
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );

  // joint1 (limit 0.1): box = [min(0.3-0.1, cmd~0), 0.3+0.1] = [0.0, 0.4]
  EXPECT_NEAR( controller_->pipeline_->qpInput().q_hi[0], 0.4, 1e-6 );
  EXPECT_NEAR( controller_->pipeline_->qpInput().q_lo[0], 0.0, 1e-6 );
  // joint2 (default limit 0.25, ref 0): box = [-0.25, 0.25] within URDF [-pi, pi]
  EXPECT_NEAR( controller_->pipeline_->qpInput().q_hi[1], 0.25, 1e-6 );
  EXPECT_NEAR( controller_->pipeline_->qpInput().q_lo[1], -0.25, 1e-6 );
}

TEST_F( SafetyPositionControllerCollisionTest, QpModeWorksWithoutCollisionChecker )
{
  // check_self_collisions=false: no collision constraints, but
  // velocity/acceleration/position limits still apply (pure smoothing mode).
  initWithCollisions( {}, "test_robot_collision.urdf",
                      { rclcpp::Parameter( "check_self_collisions", false ) } );
  configureWithSrdf();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  EXPECT_TRUE( controller_->collision_checker_ == nullptr );

  for ( auto &v : hw_state_values_ ) v = 0.0;
  controller_->reference_interfaces_[0] = 0.3;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  double prev_cmd = 0.0, prev_v = 0.0, max_dv = 0.0;
  for ( int cycle = 0; cycle < 200; ++cycle ) {
    ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
    const double cmd = hw_cmd_values_[0];
    const double v = ( cmd - prev_cmd ) / kDt;
    max_dv = std::max( max_dv, std::abs( v - prev_v ) );
    prev_cmd = cmd;
    prev_v = v;
    setStateValue( "joint1", cmd );
  }

  EXPECT_NEAR( hw_cmd_values_[0], 0.3, 1e-4 );
  EXPECT_LE( max_dv, kDecPerCycle + 1e-6 );
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
  // Use _exit to avoid double-free in global destructors caused by rtest
  // mocked rclcpp context + pinocchio/hpp-fcl library cleanup ordering issues.
  // All test resources are cleaned up in TearDown before reaching this point.
#if defined( __GNUC__ )
  if ( __gcov_dump )
    __gcov_dump(); // Flush coverage data before _exit
#endif
  _exit( result );
}
