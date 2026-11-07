#include "test_helpers.hpp"

#include <std_msgs/msg/bool.hpp>

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

    // The controller manager exports the references after configure
    ASSERT_EQ( controller_->export_reference_interfaces().size(), joints_.size() );
  }

  void setupHardwareInterfaces( const std::string &iface_type )
  {
    hw_cmd_values_.assign( joints_.size(), 0.0 );
    hw_state_values_.assign( joints_.size(), 0.0 );

    cmd_ifaces_.clear();
    state_ifaces_.clear();

    for ( size_t i = 0; i < joints_.size(); ++i ) {
      cmd_ifaces_.push_back( hw_cmd_values_.make_command( i, joints_[i], iface_type ) );
      state_ifaces_.push_back( hw_state_values_.make_state( i, joints_[i], iface_type ) );
    }

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

  hector_test::reference( *controller_, 0 ) = 0.5;
  hector_test::reference( *controller_, 1 ) = -0.3;
  hector_test::reference( *controller_, 2 ) = 0.0;

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
  hector_test::reference( *controller_, 0 ) = 5.0;
  hector_test::reference( *controller_, 1 ) = 0.0;
  hector_test::reference( *controller_, 2 ) = 0.0;

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
  hector_test::reference( *controller_, 0 ) = 0.0;
  hector_test::reference( *controller_, 1 ) = -10.0;
  hector_test::reference( *controller_, 2 ) = 0.0;

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
  hector_test::reference( *controller_, 0 ) = 20.0;
  hector_test::reference( *controller_, 1 ) = 0.0;
  hector_test::reference( *controller_, 2 ) = 0.0;

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
  hector_test::reference( *controller_, 0 ) = 0.0;
  hector_test::reference( *controller_, 1 ) = 5.0; // exceeds upper
  hector_test::reference( *controller_, 2 ) = 0.0;

  callUpdate();
  EXPECT_DOUBLE_EQ( hw_cmd_values_[1], 1.5 ); // clamped to upper limit
}

// The controller manager owns the e-stop and stops this controller on it
TEST_F( SafetyForwardControllerTest, SubscribesToNoEstop )
{
  initController( "position" );
  configureController();
  setupHardwareInterfaces( "position" );
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  const auto node_name = std::string( controller_->get_node()->get_fully_qualified_name() );
  EXPECT_FALSE( rtest::findSubscription<std_msgs::msg::Bool>( controller_->get_node(),
                                                              node_name + "/safety_estop" ) );
  EXPECT_TRUE( controller_->state_interface_configuration().names.empty() );
}

// No reference is a target: the hardware holds until one arrives, on activation and after a switch
TEST_F( SafetyForwardControllerTest, ReferencesStartInvalidOnActivation )
{
  initController( "velocity" );
  configureController();
  setupHardwareInterfaces( "velocity" );
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  for ( std::size_t i = 0; i < joints_.size(); ++i ) {
    EXPECT_TRUE( std::isnan( hector_test::reference( *controller_, i ) ) ) << "exported as 0.0";
  }
  activateController();
  hector_test::reference( *controller_, 0 ) = 0.5;
  ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
  ASSERT_DOUBLE_EQ( hw_cmd_values_[0], 0.5 );

  deactivateController();
  activateController();
  ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
  EXPECT_TRUE( std::isnan( hw_cmd_values_[0] ) ) << "a re-activation resumed the old reference";
}

TEST_F( SafetyForwardControllerTest, ChainedModeSwitchInvalidatesReferences )
{
  initController( "velocity" );
  configureController();
  for ( const bool chained : { true, false } ) {
    for ( std::size_t i = 0; i < joints_.size(); ++i ) {
      hector_test::reference( *controller_, i ) = 0.5;
    }
    ASSERT_TRUE( controller_->on_set_chained_mode( chained ) );
    for ( std::size_t i = 0; i < joints_.size(); ++i ) {
      EXPECT_TRUE( std::isnan( hector_test::reference( *controller_, i ) ) )
          << ( chained ? "chained" : "unchained" ) << " mode kept reference " << i;
    }
  }
}

// A malformed command is no command: no target, and no sign of life that restarts the dead man timer
TEST_F( SafetyForwardControllerTest, AWrongSizeCommandHoldsAndKeepsTheTimeout )
{
  initController( "velocity" );
  configureController();
  setupHardwareInterfaces( "velocity" );
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();
  hector_test::reference( *controller_, 0 ) = 0.5;
  controller_->safety_engaged_.store( true );

  auto sub = controller_->joints_command_subscriber_;
  ASSERT_TRUE( sub );
  auto msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
  msg->data = { 1.0 };
  sub->handle_message( msg );
  EXPECT_TRUE( controller_->safety_engaged_.load() ) << "a malformed message reset the timeout";

  EXPECT_EQ( controller_->update_reference_from_subscribers( rclcpp::Time( 0, 0, RCL_ROS_TIME ),
                                                             rclcpp::Duration( 0, 10000000 ) ),
             controller_interface::return_type::OK );
  for ( std::size_t i = 0; i < joints_.size(); ++i ) {
    EXPECT_TRUE( std::isnan( hector_test::reference( *controller_, i ) ) ) << "kept a command";
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

  hector_test::reference( *controller_, 0 ) = 0.5;
  hector_test::reference( *controller_, 1 ) = 0.5;
  hector_test::reference( *controller_, 2 ) = 0.5;

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

  // Subscribed only while active and unchained, so a command cannot arrive before activation
  EXPECT_TRUE( controller_->joints_command_subscriber_ == nullptr );
  EXPECT_TRUE( controller_->safety_timer_ != nullptr );

  controller_->on_set_chained_mode( true );

  EXPECT_TRUE( controller_->joints_command_subscriber_ == nullptr );
  ASSERT_TRUE( controller_->safety_timer_ != nullptr );
  EXPECT_TRUE( controller_->safety_timer_->is_canceled() );
}

// The dead man timer must survive a chained period, or later unchained activations run without it
TEST_F( SafetyForwardControllerTest, ChainedToUnchainedTransition )
{
  initController( "velocity" );
  configureController();
  setupHardwareInterfaces( "velocity" );
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );

  ASSERT_TRUE( controller_->set_chained_mode( true ) );
  activateController();
  EXPECT_TRUE( controller_->joints_command_subscriber_ == nullptr )
      << "chained, commands come from upstream";
  ASSERT_TRUE( controller_->safety_timer_ != nullptr ) << "the dead man timer was destroyed";
  EXPECT_TRUE( controller_->safety_timer_->is_canceled() );
  deactivateController();

  ASSERT_TRUE( controller_->set_chained_mode( false ) );
  auto cmd = controller_->rt_command_ptr_.readFromRT();
  EXPECT_TRUE( !cmd || !( *cmd ) );
  activateController();
  EXPECT_TRUE( controller_->joints_command_subscriber_ != nullptr );
  ASSERT_TRUE( controller_->safety_timer_ != nullptr ) << "the dead man timer was destroyed";
  EXPECT_FALSE( controller_->safety_timer_->is_canceled() );
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
  hector_test::reference( *controller_, 0 ) = 0.7;
  hector_test::reference( *controller_, 1 ) = 0.0;
  hector_test::reference( *controller_, 2 ) = 0.0;

  auto ret = callUpdate();
  EXPECT_EQ( ret, controller_interface::return_type::OK );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[0], 0.7 );
}

TEST_F( SafetyForwardControllerTest, UnchainedCommandBecomesTheReference )
{
  initController( "velocity" );
  configureController();
  setupHardwareInterfaces( "velocity" );
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  auto msg = std::make_shared<safety_forward_controller::CmdType>();
  msg->data = { 0.4, -0.2, 0.0 };
  controller_->rt_command_ptr_.writeFromNonRT( msg );

  rclcpp::Time now( 0, 0, RCL_ROS_TIME );
  rclcpp::Duration period( std::chrono::milliseconds( 10 ) );
  ASSERT_EQ( controller_->update_reference_from_subscribers( now, period ),
             controller_interface::return_type::OK );
  EXPECT_DOUBLE_EQ( hector_test::reference( *controller_, 0 ), 0.4 );
  EXPECT_DOUBLE_EQ( hector_test::reference( *controller_, 1 ), -0.2 );

  ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[0], 0.4 );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[1], -0.2 );
}

TEST_F( SafetyForwardControllerTest, BusyReferenceHandleKeepsLastCyclesValue )
{
  initController( "velocity" );
  configureController();
  setupHardwareInterfaces( "velocity" );
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();
  controller_->on_set_chained_mode( true );

  hector_test::reference( *controller_, 0 ) = 0.7;
  ASSERT_EQ( callUpdate(), controller_interface::return_type::OK );
  hw_cmd_values_[0] = 99.0;

  std::unique_lock<std::shared_mutex> busy(
      controller_->ordered_exported_reference_interfaces_[0]->get_mutex() );
  EXPECT_EQ( callUpdate(), controller_interface::return_type::OK )
      << "a busy reference handle must not fail the update";
  EXPECT_DOUBLE_EQ( hw_cmd_values_[0], 0.7 );
}

TEST_F( SafetyForwardControllerTest, BusyReferenceHandleTakesTheCommandOnceFree )
{
  initController( "velocity" );
  configureController();
  setupHardwareInterfaces( "velocity" );
  findMocks();
  EXPECT_CALL( *status_pub_mock_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  activateController();

  auto msg = std::make_shared<safety_forward_controller::CmdType>();
  msg->data = { 0.4, -0.2, 0.0 };
  controller_->rt_command_ptr_.writeFromNonRT( msg );
  rclcpp::Time now( 0, 0, RCL_ROS_TIME );
  rclcpp::Duration period( std::chrono::milliseconds( 10 ) );
  {
    std::unique_lock<std::shared_mutex> busy(
        controller_->ordered_exported_reference_interfaces_[0]->get_mutex() );
    EXPECT_EQ( controller_->update_reference_from_subscribers( now, period ),
               controller_interface::return_type::OK )
        << "a busy reference handle must not fail the update";
  }
  EXPECT_DOUBLE_EQ( hector_test::reference( *controller_, 1 ), -0.2 );

  ASSERT_EQ( controller_->update_reference_from_subscribers( now, period ),
             controller_interface::return_type::OK );
  EXPECT_DOUBLE_EQ( hector_test::reference( *controller_, 0 ), 0.4 );
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

  EXPECT_FALSE( captured.safety_timer_engaged );
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
    hector_test::reference( *controller_, 0 ) = 0.5;
    hector_test::reference( *controller_, 1 ) = 0.0;
    hector_test::reference( *controller_, 2 ) = 0.0;
    auto ret = callUpdate();
    EXPECT_EQ( ret, controller_interface::return_type::OK );
    EXPECT_DOUBLE_EQ( hw_cmd_values_[0], 0.5 );

    deactivateController();

    // Verify state is reset after deactivation
    EXPECT_FALSE( controller_->safety_engaged_.load() );
    EXPECT_TRUE( controller_->joints_command_subscriber_ == nullptr );
  }
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
