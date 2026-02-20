#include "test_helpers.hpp"

using SafetyForwardControllerStatus = hector_ros_controllers_msgs::msg::SafetyForwardControllerStatus;

class SafetyForwardControllerTest
    : public hector_test::SafetyControllerTestBase<SafetyForwardControllerTest>
{
public:
  using ControllerType = safety_forward_controller::SafetyForwardController;
  using StatusMsgType = SafetyForwardControllerStatus;

  std::vector<std::string> joints_{ "joint1", "joint2", "joint3" };

  std::shared_ptr<ControllerType> controller_;
  std::shared_ptr<rtest::PublisherMock<StatusMsgType>> status_pub_mock_;

  void resetControllerSpecificMocks() { }
  void findControllerSpecificMocks( const std::string & /*node_name*/ ) { }

  void SetUp() override { controller_ = std::make_shared<ControllerType>(); }

  void initController( const std::string &interface_type, const std::string &passthrough = "" )
  {
    const auto urdf = hector_test::loadUrdfFile( "test_robot.urdf" );

    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "test_safety_forward";
    params.robot_description = urdf;
    params.update_rate = kUpdateRate;
    params.controller_manager_update_rate = kUpdateRate;
    params.node_namespace = "";

    rclcpp::NodeOptions opts;
    opts.parameter_overrides( {
        rclcpp::Parameter( "joints", joints_ ),
        rclcpp::Parameter( "interface_type", interface_type ),
        rclcpp::Parameter( "safety_timer_duration", 500 ),
        rclcpp::Parameter( "passthrough_controller", passthrough ),
    } );
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
    controller_->reference_interfaces_.assign( joints_.size(), 0.0 );
  }

  void setupHardwareInterfaces( const std::string &iface_type )
  {
    hw_cmd_values_.assign( joints_.size(), 0.0 );
    hw_state_values_.assign( joints_.size(), 0.0 );

    cmd_ifaces_.clear();
    state_ifaces_.clear();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    for ( size_t i = 0; i < joints_.size(); ++i ) {
      cmd_ifaces_.push_back( std::make_shared<hardware_interface::CommandInterface>(
          joints_[i], iface_type, &hw_cmd_values_[i] ) );
      state_ifaces_.push_back( std::make_shared<hardware_interface::StateInterface>(
          joints_[i], iface_type, &hw_state_values_[i] ) );
    }
#pragma GCC diagnostic pop

    controller_->command_interfaces_.clear();
    controller_->state_interfaces_.clear();

    for ( auto &ci : cmd_ifaces_ ) {
      controller_->command_interfaces_.emplace_back( ci, []() { } );
    }
    for ( auto &si : state_ifaces_ ) { controller_->state_interfaces_.emplace_back( si ); }
  }
};

// ============================================================================
// Lifecycle & Init Tests
// ============================================================================

TEST_F( SafetyForwardControllerTest, OnInitSucceeds )
{
  initController( "velocity" );
  EXPECT_TRUE( controller_->param_listener_ != nullptr );
}

TEST_F( SafetyForwardControllerTest, OnConfigureVelocitySucceeds )
{
  initController( "velocity" );
  configureController();
  EXPECT_EQ( controller_->interface_type_, "velocity" );
  EXPECT_EQ( controller_->joints_.size(), 3u );
  EXPECT_TRUE( controller_->safety_timer_ != nullptr );
}

TEST_F( SafetyForwardControllerTest, OnConfigureFailsEmptyJoints )
{
  // Override joints to empty
  joints_ = {};
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "test_ctrl";
  params.robot_description = hector_test::loadUrdfFile( "test_robot.urdf" );
  params.update_rate = kUpdateRate;
  params.controller_manager_update_rate = kUpdateRate;
  rclcpp::NodeOptions opts;
  opts.parameter_overrides( {
      rclcpp::Parameter( "joints", std::vector<std::string>{} ),
      rclcpp::Parameter( "interface_type", std::string( "velocity" ) ),
      rclcpp::Parameter( "safety_timer_duration", 500 ),
      rclcpp::Parameter( "passthrough_controller", std::string( "" ) ),
  } );
  params.node_options = opts;
  controller_->init( params );

  rclcpp_lifecycle::State unconfigured( lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                        "unconfigured" );
  auto cb = controller_->on_configure( unconfigured );
  EXPECT_EQ( cb, controller_interface::CallbackReturn::ERROR );
}

TEST_F( SafetyForwardControllerTest, OnConfigureFailsInvalidInterface )
{
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "test_ctrl";
  params.robot_description = hector_test::loadUrdfFile( "test_robot.urdf" );
  params.update_rate = kUpdateRate;
  params.controller_manager_update_rate = kUpdateRate;
  rclcpp::NodeOptions opts;
  opts.parameter_overrides( {
      rclcpp::Parameter( "joints", joints_ ),
      rclcpp::Parameter( "interface_type", std::string( "invalid" ) ),
      rclcpp::Parameter( "safety_timer_duration", 500 ),
      rclcpp::Parameter( "passthrough_controller", std::string( "" ) ),
  } );
  params.node_options = opts;
  controller_->init( params );

  rclcpp_lifecycle::State unconfigured( lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                        "unconfigured" );
  auto cb = controller_->on_configure( unconfigured );
  EXPECT_EQ( cb, controller_interface::CallbackReturn::ERROR );
}

// ============================================================================
// Normal Operation Tests
// ============================================================================

TEST_F( SafetyForwardControllerTest, NormalForwardingVelocity )
{
  initController( "velocity" );
  configureController();
  setupHardwareInterfaces( "velocity" );
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  controller_->reference_interfaces_[0] = 0.5;
  controller_->reference_interfaces_[1] = -0.3;
  controller_->reference_interfaces_[2] = 0.0;

  auto ret = callUpdate();
  EXPECT_EQ( ret, controller_interface::return_type::OK );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[0], 0.5 );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[1], -0.3 );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[2], 0.0 );
}

TEST_F( SafetyForwardControllerTest, VelocityClamped )
{
  initController( "velocity" );
  configureController();
  setupHardwareInterfaces( "velocity" );
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  // joint1 velocity limit = 1.0, set reference beyond
  controller_->reference_interfaces_[0] = 5.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  callUpdate();
  EXPECT_DOUBLE_EQ( hw_cmd_values_[0], 1.0 ); // clamped to velocity limit
}

TEST_F( SafetyForwardControllerTest, VelocityClampedNegative )
{
  initController( "velocity" );
  configureController();
  setupHardwareInterfaces( "velocity" );
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  // joint2 velocity limit = 2.0, set negative beyond
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = -10.0;
  controller_->reference_interfaces_[2] = 0.0;

  callUpdate();
  EXPECT_DOUBLE_EQ( hw_cmd_values_[1], -2.0 ); // clamped to -velocity limit
}

TEST_F( SafetyForwardControllerTest, EffortClamped )
{
  initController( "effort" );
  configureController();
  setupHardwareInterfaces( "effort" );
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  // joint1 effort limit = 10.0
  controller_->reference_interfaces_[0] = 20.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  callUpdate();
  EXPECT_DOUBLE_EQ( hw_cmd_values_[0], 10.0 ); // clamped to effort limit
}

TEST_F( SafetyForwardControllerTest, PositionClamped )
{
  initController( "position" );
  configureController();
  setupHardwareInterfaces( "position" );
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  // joint2 position limits [-1.5, 1.5]
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = 5.0; // exceeds upper
  controller_->reference_interfaces_[2] = 0.0;

  callUpdate();
  EXPECT_DOUBLE_EQ( hw_cmd_values_[1], 1.5 ); // clamped to upper limit
}

// ============================================================================
// E-Stop Tests
// ============================================================================

TEST_F( SafetyForwardControllerTest, EstopEngagePosition )
{
  initController( "position" );
  configureController();
  setupHardwareInterfaces( "position" );
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  // Set current state positions (for hold)
  hw_state_values_[0] = 0.5;
  hw_state_values_[1] = -0.3;
  hw_state_values_[2] = 1.0;

  sendEstop( true );
  auto ret = callUpdate();
  EXPECT_EQ( ret, controller_interface::return_type::OK );
  EXPECT_TRUE( controller_->estop_engaged_.load() );

  // Commands should be hold positions
  EXPECT_DOUBLE_EQ( hw_cmd_values_[0], 0.5 );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[1], -0.3 );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[2], 1.0 );
}

TEST_F( SafetyForwardControllerTest, EstopEngageVelocity )
{
  initController( "velocity" );
  configureController();
  setupHardwareInterfaces( "velocity" );
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  // Set some non-zero commands first
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 1.0;
  controller_->reference_interfaces_[2] = 1.0;

  sendEstop( true );
  callUpdate();

  // Velocity e-stop zeros commands
  EXPECT_DOUBLE_EQ( hw_cmd_values_[0], 0.0 );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[1], 0.0 );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[2], 0.0 );
}

TEST_F( SafetyForwardControllerTest, EstopRelease )
{
  initController( "velocity" );
  configureController();
  setupHardwareInterfaces( "velocity" );
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  sendEstop( true );
  callUpdate(); // engage
  sendEstop( false );
  callUpdate(); // release

  // reference_interfaces_ should be NaN after release
  for ( size_t i = 0; i < joints_.size(); ++i ) {
    EXPECT_TRUE( std::isnan( controller_->reference_interfaces_[i] ) );
  }
}

// ============================================================================
// Safety Timer Tests
// ============================================================================

TEST_F( SafetyForwardControllerTest, SafetyTimerEngages )
{
  initController( "velocity" );
  configureController();
  setupHardwareInterfaces( "velocity" );
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  controller_->reference_interfaces_[0] = 0.5;
  controller_->reference_interfaces_[1] = 0.5;
  controller_->reference_interfaces_[2] = 0.5;

  // Manually trigger safety engaged
  controller_->safety_engaged_.store( true );

  callUpdate();

  // Safety timer zeros all commands
  EXPECT_DOUBLE_EQ( hw_cmd_values_[0], 0.0 );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[1], 0.0 );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[2], 0.0 );
}

TEST_F( SafetyForwardControllerTest, NoSafetyTimerForPosition )
{
  initController( "position" );
  configureController();
  EXPECT_TRUE( controller_->safety_timer_ == nullptr );
}

// ============================================================================
// Chained Mode Tests
// ============================================================================

TEST_F( SafetyForwardControllerTest, ChainedModeCleanup )
{
  initController( "velocity" );
  configureController();

  EXPECT_TRUE( controller_->joints_command_subscriber_ != nullptr );
  EXPECT_TRUE( controller_->safety_timer_ != nullptr );

  controller_->on_set_chained_mode( true );

  EXPECT_TRUE( controller_->joints_command_subscriber_ == nullptr );
  EXPECT_TRUE( controller_->safety_timer_ == nullptr );
}

TEST_F( SafetyForwardControllerTest, ChainedToUnchainedTransition )
{
  initController( "velocity" );
  configureController();

  // Switch to chained mode: subscriber and timer are destroyed
  controller_->on_set_chained_mode( true );
  EXPECT_TRUE( controller_->joints_command_subscriber_ == nullptr );
  EXPECT_TRUE( controller_->safety_timer_ == nullptr );

  // Switch back to unchained: rt buffer is cleared
  controller_->on_set_chained_mode( false );
  auto cmd = controller_->rt_command_ptr_.readFromRT();
  EXPECT_TRUE( !cmd || !( *cmd ) );
}

TEST_F( SafetyForwardControllerTest, ChainedModeUpdateForwardsReferences )
{
  initController( "velocity" );
  configureController();
  setupHardwareInterfaces( "velocity" );
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  // Enter chained mode
  controller_->on_set_chained_mode( true );

  // Set reference_interfaces directly (as a chaining controller would)
  controller_->reference_interfaces_[0] = 0.7;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  auto ret = callUpdate();
  EXPECT_EQ( ret, controller_interface::return_type::OK );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[0], 0.7 );
}

// ============================================================================
// Status Publishing
// ============================================================================

TEST_F( SafetyForwardControllerTest, StatusPublishesCorrectly )
{
  initController( "velocity" );
  configureController();
  setupHardwareInterfaces( "velocity" );
  findMocks();

  SafetyForwardControllerStatus captured;
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) )
      .WillRepeatedly( [&captured]( const auto &msg ) { captured = msg; } );

  activateController();

  EXPECT_FALSE( captured.estop_engaged );
  EXPECT_FALSE( captured.safety_timer_engaged );
}

TEST_F( SafetyForwardControllerTest, StatusUpdatesOnEstopEngage )
{
  initController( "velocity" );
  configureController();
  setupHardwareInterfaces( "velocity" );
  findMocks();

  SafetyForwardControllerStatus captured;
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) )
      .WillRepeatedly( [&captured]( const auto &msg ) { captured = msg; } );

  activateController();
  EXPECT_FALSE( captured.estop_engaged );

  sendEstop( true );
  callUpdate();
  EXPECT_TRUE( captured.estop_engaged );
  EXPECT_FALSE( captured.safety_timer_engaged );
}

TEST_F( SafetyForwardControllerTest, StatusUpdatesOnEstopRelease )
{
  initController( "velocity" );
  configureController();
  setupHardwareInterfaces( "velocity" );
  findMocks();

  SafetyForwardControllerStatus captured;
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) )
      .WillRepeatedly( [&captured]( const auto &msg ) { captured = msg; } );

  activateController();

  sendEstop( true );
  callUpdate();
  EXPECT_TRUE( captured.estop_engaged );

  sendEstop( false );
  callUpdate();
  EXPECT_FALSE( captured.estop_engaged );
}

TEST_F( SafetyForwardControllerTest, StatusUpdatesOnSafetyTimerEngage )
{
  initController( "velocity" );
  configureController();
  setupHardwareInterfaces( "velocity" );
  findMocks();

  SafetyForwardControllerStatus captured;
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) )
      .WillRepeatedly( [&captured]( const auto &msg ) { captured = msg; } );

  activateController();
  EXPECT_FALSE( captured.safety_timer_engaged );

  // The safety timer publish happens in the wall timer callback, not in update.
  // Simulate what the timer callback does: set engaged + publish.
  controller_->safety_engaged_.store( true );
  controller_->publish_status();
  EXPECT_TRUE( captured.safety_timer_engaged );
  EXPECT_FALSE( captured.estop_engaged );
}

// ============================================================================
// Repeated Activation/Deactivation Cycle Tests
// ============================================================================

TEST_F( SafetyForwardControllerTest, RepeatedActivateDeactivateCycles )
{
  initController( "velocity" );
  configureController();
  setupHardwareInterfaces( "velocity" );
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );

  for ( int cycle = 0; cycle < 3; ++cycle ) {
    activateController();

    // Verify functional after activation
    controller_->reference_interfaces_[0] = 0.5;
    controller_->reference_interfaces_[1] = 0.0;
    controller_->reference_interfaces_[2] = 0.0;
    auto ret = callUpdate();
    EXPECT_EQ( ret, controller_interface::return_type::OK );
    EXPECT_DOUBLE_EQ( hw_cmd_values_[0], 0.5 );

    deactivateController();

    // Verify state is reset after deactivation
    EXPECT_FALSE( controller_->estop_active_.load() );
    EXPECT_FALSE( controller_->estop_engaged_.load() );
    EXPECT_FALSE( controller_->safety_engaged_.load() );
    EXPECT_TRUE( controller_->joints_command_subscriber_ == nullptr );
    EXPECT_TRUE( controller_->safety_estop_subscriber_ == nullptr );
  }
}

TEST_F( SafetyForwardControllerTest, ReactivateAfterEstop )
{
  initController( "velocity" );
  configureController();
  setupHardwareInterfaces( "velocity" );
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );

  activateController();
  sendEstop( true );
  callUpdate();
  EXPECT_TRUE( controller_->estop_engaged_.load() );

  deactivateController();
  // After deactivation, estop should be reset
  EXPECT_FALSE( controller_->estop_active_.load() );
  EXPECT_FALSE( controller_->estop_engaged_.load() );

  activateController();
  // Controller should be in clean state
  EXPECT_FALSE( controller_->estop_engaged_.load() );
  EXPECT_FALSE( controller_->safety_engaged_.load() );

  // Verify it works normally after re-activation
  controller_->reference_interfaces_[0] = 0.3;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;
  auto ret = callUpdate();
  EXPECT_EQ( ret, controller_interface::return_type::OK );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[0], 0.3 );
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
