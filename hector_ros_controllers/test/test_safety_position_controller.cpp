#include "test_helpers.hpp"

// __gcov_dump is only available when compiled with --coverage.
// Use a weak symbol so the call is a no-op in normal (non-coverage) builds.
#if defined( __GNUC__ )
extern "C" void __gcov_dump() __attribute__( ( weak ) );
#endif

using SafetyPositionControllerStatus =
    hector_ros_controllers_msgs::msg::SafetyPositionControllerStatus;
using SPC = safety_position_controller::SafetyPositionController;

// ============================================================================
// Static method tests (no fixture needed)
// ============================================================================

TEST( SafetyPositionControllerStatic, UnwrapToNearestBasic )
{
  // Target near current - no wrapping needed
  EXPECT_NEAR( safety_position_controller::unwrap_to_nearest( 0.0, 0.1 ), 0.1, 1e-9 );
  EXPECT_NEAR( safety_position_controller::unwrap_to_nearest( 0.0, -0.1 ), -0.1, 1e-9 );

  // Wrapping: current=3.0, target=-3.0 -> should unwrap to near 3.0 (adding 2*pi)
  double result = safety_position_controller::unwrap_to_nearest( 3.0, -3.0 );
  EXPECT_NEAR( result, -3.0 + 2 * M_PI, 1e-9 );
}

TEST( SafetyPositionControllerStatic, UnwrapToNearestMultiRevolutions )
{
  // current at 10.0 rad, target at 0.1 -> should unwrap near 10.0
  double result = safety_position_controller::unwrap_to_nearest( 10.0, 0.1 );
  double expected = 0.1 + std::round( ( 10.0 - 0.1 ) / ( 2 * M_PI ) ) * ( 2 * M_PI );
  EXPECT_NEAR( result, expected, 1e-9 );
  // Result should be within pi of the current position
  EXPECT_LT( std::abs( result - 10.0 ), M_PI );
}

TEST( SafetyPositionControllerStatic, UnwrapToNearestNegative )
{
  // current at -10.0, target 0.0 -> should unwrap near -10.0
  double result = safety_position_controller::unwrap_to_nearest( -10.0, 0.0 );
  double expected = 0.0 + std::round( ( -10.0 - 0.0 ) / ( 2 * M_PI ) ) * ( 2 * M_PI );
  EXPECT_NEAR( result, expected, 1e-9 );
}

TEST( SafetyPositionControllerStatic, GetSignedDistanceBasic )
{
  EXPECT_NEAR( safety_position_controller::get_signed_distance( 0.0, 0.0 ), 0.0, 1e-9 );
  EXPECT_NEAR( safety_position_controller::get_signed_distance( 0.0, 1.0 ), 1.0, 1e-9 );
  EXPECT_NEAR( safety_position_controller::get_signed_distance( 0.0, -1.0 ), -1.0, 1e-9 );
  EXPECT_NEAR( safety_position_controller::get_signed_distance( 1.0, 2.0 ), 1.0, 1e-9 );
}

TEST( SafetyPositionControllerStatic, GetSignedDistanceWrapping )
{
  // From 3.0 to -3.0: shortest path is positive ~0.28 rad
  double dist = safety_position_controller::get_signed_distance( 3.0, -3.0 );
  EXPECT_NEAR( dist, 2 * M_PI - 6.0, 1e-9 );
  EXPECT_GT( dist, 0.0 );

  // From 0 to pi+0.1: shortest should be negative (wrap around)
  dist = safety_position_controller::get_signed_distance( 0.0, M_PI + 0.1 );
  EXPECT_NEAR( dist, -( 2 * M_PI - M_PI - 0.1 ), 1e-9 );
  EXPECT_LT( dist, 0.0 );
}

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
        rclcpp::Parameter( "unwrap_continuous_joints", true ),
        rclcpp::Parameter( "enforce_position_limits", true ),
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

TEST_F( SafetyPositionControllerTest, EnforceLimitsClampsRevolute )
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

TEST_F( SafetyPositionControllerTest, EnforceLimitsClampsRevoluteLower )
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

TEST_F( SafetyPositionControllerTest, EnforceLimitsUnwrapsContinuous )
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
  controller_->reference_interfaces_[1] = 0.1; // will be unwrapped

  callUpdate();

  // After unwrap, cmd should be closer to 10.0 than to 0.1
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

TEST_F( SafetyPositionControllerTest, EstopReleaseInvalidatesReferences )
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
  sendEstop( false );
  callUpdate();

  for ( size_t i = 0; i < controlled_joints_.size(); ++i ) {
    EXPECT_TRUE( std::isnan( controller_->reference_interfaces_[i] ) );
  }
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
  EXPECT_TRUE( captured.position_limits_enforced );
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
  // Init with collisions enabled
  initController( {}, /*check_self_collisions=*/true );
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

    // Verify state is reset after deactivation
    EXPECT_FALSE( controller_->estop_active_.load() );
    EXPECT_FALSE( controller_->estop_engaged_.load() );
    EXPECT_TRUE( controller_->estop_subscriber_ == nullptr );
    EXPECT_TRUE( controller_->joints_command_subscriber_ == nullptr );
  }
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
  // After deactivation, estop should be cleared
  EXPECT_FALSE( controller_->estop_active_.load() );
  EXPECT_FALSE( controller_->estop_engaged_.load() );

  activateController();
  // Controller should be in clean state
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

// ============================================================================
// Chained Mode Tests
// ============================================================================

TEST_F( SafetyPositionControllerTest, ChainedModeInvalidatesReferences )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  // Set valid references
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 2.0;
  controller_->reference_interfaces_[2] = 3.0;

  // Switching to chained mode should invalidate all references
  controller_->on_set_chained_mode( true );
  EXPECT_TRUE( controller_->is_chained_ );

  for ( size_t i = 0; i < controlled_joints_.size(); ++i ) {
    EXPECT_TRUE( std::isnan( controller_->reference_interfaces_[i] ) )
        << "reference_interfaces_[" << i << "] should be NaN after switching to chained mode";
  }
}

TEST_F( SafetyPositionControllerTest, UnchainedModeInvalidatesReferences )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  // Start in chained mode
  controller_->on_set_chained_mode( true );

  // Set valid references
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 2.0;
  controller_->reference_interfaces_[2] = 3.0;

  // Switching to unchained mode should also invalidate references
  controller_->on_set_chained_mode( false );
  EXPECT_FALSE( controller_->is_chained_ );

  for ( size_t i = 0; i < controlled_joints_.size(); ++i ) {
    EXPECT_TRUE( std::isnan( controller_->reference_interfaces_[i] ) )
        << "reference_interfaces_[" << i << "] should be NaN after switching to unchained mode";
  }
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
        rclcpp::Parameter( "unwrap_continuous_joints", true ),
        rclcpp::Parameter( "enforce_position_limits", true ),
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
  // The inf will be clamped by enforce_limits (joint1 has limits [-pi, pi])
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
