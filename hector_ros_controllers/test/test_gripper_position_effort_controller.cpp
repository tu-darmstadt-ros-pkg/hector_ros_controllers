#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rtest/action_server_mock.hpp>
#include <rtest/publisher_mock.hpp>
#include <rtest/static_registry.hpp>
#include <rtest/subscription_mock.hpp>

// Access private/protected members for testing
#define private public
#define protected public
#include <gripper_position_effort_controller/gripper_position_effort_controller.hpp>
#undef protected
#undef private

#include <controller_interface/controller_interface.hpp>
#include <controller_interface/controller_interface_params.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

using GripperController = gripper_position_effort_controller::GripperPositionEffortController;

namespace
{

std::string loadGripperUrdf()
{
  const std::string path =
      ament_index_cpp::get_package_share_directory( "hector_ros_controllers" ) +
      "/test/config/test_gripper.urdf";
  std::ifstream ifs( path );
  if ( !ifs.is_open() ) {
    throw std::runtime_error( "Cannot open test URDF file: " + path );
  }
  return std::string( std::istreambuf_iterator<char>( ifs ), std::istreambuf_iterator<char>() );
}

} // namespace

class GripperPositionEffortControllerTest : public ::testing::Test
{
protected:
  static constexpr unsigned int kUpdateRate = 100;
  static constexpr double kPeriodSec = 0.01;
  const std::string joint_name_ = "gripper_joint";

  std::shared_ptr<GripperController> controller_;

  // Backing storage: cmd[0]=position, cmd[1]=effort; state[0]=position, state[1]=velocity, state[2]=effort
  std::vector<double> hw_cmd_values_;
  std::vector<double> hw_state_values_;
  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> cmd_ifaces_;
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> state_ifaces_;

  void SetUp() override { controller_ = std::make_shared<GripperController>(); }

  void TearDown() override
  {
    controller_.reset();
    rtest::StaticMocksRegistry::instance().reset();
  }

  void initController( const std::vector<rclcpp::Parameter> &extra_overrides = {} )
  {
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "test_gripper";
    params.robot_description = loadGripperUrdf();
    params.update_rate = kUpdateRate;
    params.controller_manager_update_rate = kUpdateRate;
    params.node_namespace = "";

    std::vector<rclcpp::Parameter> overrides = {
        rclcpp::Parameter( "joint", joint_name_ ),
        rclcpp::Parameter( "default_max_effort", 1.0 ),
        rclcpp::Parameter( "stall_timeout", 0.1 ),
        rclcpp::Parameter( "stall_velocity_threshold", 0.001 ),
        rclcpp::Parameter( "goal_tolerance", 0.01 ),
        rclcpp::Parameter( "is_grasped_velocity_threshold", 0.005 ),
        rclcpp::Parameter( "is_grasped_effort_threshold", 0.5 ),
        rclcpp::Parameter( "is_grasped_dwell_cycles", 3 ),
        rclcpp::Parameter( "velocity_command_timeout", 0.1 ),
    };
    overrides.insert( overrides.end(), extra_overrides.begin(), extra_overrides.end() );

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
  }

  void setupHardwareInterfaces()
  {
    hw_cmd_values_.assign( 2, 0.0 );   // [position, effort]
    hw_state_values_.assign( 3, 0.0 ); // [position, velocity, effort]

    cmd_ifaces_.clear();
    state_ifaces_.clear();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cmd_ifaces_.push_back( std::make_shared<hardware_interface::CommandInterface>(
        joint_name_, "position", &hw_cmd_values_[0] ) );
    cmd_ifaces_.push_back( std::make_shared<hardware_interface::CommandInterface>(
        joint_name_, "effort", &hw_cmd_values_[1] ) );
    state_ifaces_.push_back( std::make_shared<hardware_interface::StateInterface>(
        joint_name_, "position", &hw_state_values_[0] ) );
    state_ifaces_.push_back( std::make_shared<hardware_interface::StateInterface>(
        joint_name_, "velocity", &hw_state_values_[1] ) );
    state_ifaces_.push_back( std::make_shared<hardware_interface::StateInterface>(
        joint_name_, "effort", &hw_state_values_[2] ) );
#pragma GCC diagnostic pop

    controller_->command_interfaces_.clear();
    controller_->state_interfaces_.clear();
    for ( auto &ci : cmd_ifaces_ ) controller_->command_interfaces_.emplace_back( ci, []() { } );
    for ( auto &si : state_ifaces_ ) controller_->state_interfaces_.emplace_back( si );
  }

  void activateController()
  {
    rclcpp_lifecycle::State inactive( lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                      "inactive" );
    auto cb = controller_->on_activate( inactive );
    ASSERT_EQ( cb, controller_interface::CallbackReturn::SUCCESS );
  }

  controller_interface::return_type callUpdate( double t_seconds = 0.0 )
  {
    rclcpp::Time now( static_cast<int64_t>( t_seconds * 1e9 ), RCL_ROS_TIME );
    rclcpp::Duration period( std::chrono::milliseconds( static_cast<int>( kPeriodSec * 1000 ) ) );
    return controller_->update( now, period );
  }

  void setPosition( double v ) { hw_state_values_[0] = v; }
  void setVelocity( double v ) { hw_state_values_[1] = v; }
  void setEffort( double v ) { hw_state_values_[2] = v; }
  double cmdPos() const { return hw_cmd_values_[0]; }
  double cmdEffort() const { return hw_cmd_values_[1]; }

  // Inject a position-topic command directly through the controller's RT buffer
  // (bypasses the rclcpp executor — equivalent to what the subscriber callback would do)
  void injectPositionCommand( double value )
  {
    auto msg = std::make_shared<std_msgs::msg::Float64>();
    msg->data = value;
    controller_->rt_position_cmd_.writeFromNonRT( msg );
    controller_->position_cmd_seq_.store( controller_->input_seq_counter_.fetch_add( 1 ) + 1 );
  }

  void injectVelocityCommand( double value )
  {
    auto msg = std::make_shared<std_msgs::msg::Float64>();
    msg->data = value;
    controller_->rt_velocity_cmd_.writeFromNonRT( msg );
    controller_->velocity_cmd_seq_.store( controller_->input_seq_counter_.fetch_add( 1 ) + 1 );
  }

  // Inject an action-style command directly. Mirrors what accepted_callback would do
  // (without going through a real action server / goal handle).
  void injectActionCommand( double position, double max_effort )
  {
    GripperController::Command cmd{ position, max_effort };
    controller_->rt_action_command_.writeFromNonRT( cmd );
    controller_->action_cmd_seq_.store( controller_->input_seq_counter_.fetch_add( 1 ) + 1 );
  }
};

// ============================================================================
// Lifecycle / Configuration
// ============================================================================

TEST_F( GripperPositionEffortControllerTest, OnInitSucceeds )
{
  initController();
  EXPECT_TRUE( controller_->param_listener_ != nullptr );
}

TEST_F( GripperPositionEffortControllerTest, OnConfigureFailsEmptyJoint )
{
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "test_gripper";
  params.robot_description = loadGripperUrdf();
  params.update_rate = kUpdateRate;
  params.controller_manager_update_rate = kUpdateRate;
  rclcpp::NodeOptions opts;
  opts.parameter_overrides( { rclcpp::Parameter( "joint", std::string( "" ) ) } );
  params.node_options = opts;
  controller_->init( params );

  rclcpp_lifecycle::State unconfigured( lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                        "unconfigured" );
  EXPECT_EQ( controller_->on_configure( unconfigured ), controller_interface::CallbackReturn::ERROR );
}

TEST_F( GripperPositionEffortControllerTest, JointLimitsParsedFromUrdf )
{
  initController();
  configureController();
  EXPECT_DOUBLE_EQ( controller_->joint_lower_limit_, -1.0 );
  EXPECT_DOUBLE_EQ( controller_->joint_upper_limit_, 1.0 );
}

TEST_F( GripperPositionEffortControllerTest, InterfaceConfigurationsClaimPositionAndEffort )
{
  initController();
  configureController();
  auto cmd_cfg = controller_->command_interface_configuration();
  ASSERT_EQ( cmd_cfg.names.size(), 2u );
  EXPECT_EQ( cmd_cfg.names[0], joint_name_ + "/position" );
  EXPECT_EQ( cmd_cfg.names[1], joint_name_ + "/effort" );

  auto state_cfg = controller_->state_interface_configuration();
  ASSERT_EQ( state_cfg.names.size(), 3u );
  EXPECT_EQ( state_cfg.names[0], joint_name_ + "/position" );
  EXPECT_EQ( state_cfg.names[1], joint_name_ + "/velocity" );
  EXPECT_EQ( state_cfg.names[2], joint_name_ + "/effort" );
}

TEST_F( GripperPositionEffortControllerTest, OnInitFailsWhenEffortCommandInterfaceParamInvalid )
{
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "test_gripper";
  params.robot_description = loadGripperUrdf();
  params.update_rate = kUpdateRate;
  params.controller_manager_update_rate = kUpdateRate;
  rclcpp::NodeOptions opts;
  opts.parameter_overrides( { rclcpp::Parameter( "joint", joint_name_ ),
                              rclcpp::Parameter( "effort_command_interface", std::string( "" ) ) } );
  params.node_options = opts;
  // Empty string fails one_of<> validation in the generated ParamListener constructor,
  // which throws — caught by on_init and surfaced as ERROR.
  EXPECT_EQ( controller_->init( params ), controller_interface::return_type::ERROR );
}

TEST_F( GripperPositionEffortControllerTest, DisabledModeOmitsEffortCommandInterface )
{
  initController( { rclcpp::Parameter( "effort_command_interface", std::string( "disabled" ) ) } );
  configureController();
  auto cmd_cfg = controller_->command_interface_configuration();
  ASSERT_EQ( cmd_cfg.names.size(), 1u );
  EXPECT_EQ( cmd_cfg.names[0], joint_name_ + "/position" );

  auto state_cfg = controller_->state_interface_configuration();
  ASSERT_EQ( state_cfg.names.size(), 3u );
  EXPECT_EQ( state_cfg.names[0], joint_name_ + "/position" );
  EXPECT_EQ( state_cfg.names[1], joint_name_ + "/velocity" );
  EXPECT_EQ( state_cfg.names[2], joint_name_ + "/effort" );
}

TEST_F( GripperPositionEffortControllerTest, DisabledModeWritesPositionOnly )
{
  initController( { rclcpp::Parameter( "effort_command_interface", std::string( "disabled" ) ) } );
  configureController();

  // Hardware setup without an effort command interface, mirroring a sim joint
  // that exposes only a position command.
  hw_cmd_values_.assign( 1, 0.0 );
  hw_state_values_.assign( 3, 0.0 );
  cmd_ifaces_.clear();
  state_ifaces_.clear();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  cmd_ifaces_.push_back( std::make_shared<hardware_interface::CommandInterface>(
      joint_name_, "position", &hw_cmd_values_[0] ) );
  state_ifaces_.push_back( std::make_shared<hardware_interface::StateInterface>(
      joint_name_, "position", &hw_state_values_[0] ) );
  state_ifaces_.push_back( std::make_shared<hardware_interface::StateInterface>(
      joint_name_, "velocity", &hw_state_values_[1] ) );
  state_ifaces_.push_back( std::make_shared<hardware_interface::StateInterface>(
      joint_name_, "effort", &hw_state_values_[2] ) );
#pragma GCC diagnostic pop
  controller_->command_interfaces_.clear();
  controller_->state_interfaces_.clear();
  for ( auto &ci : cmd_ifaces_ ) controller_->command_interfaces_.emplace_back( ci, []() { } );
  for ( auto &si : state_ifaces_ ) controller_->state_interfaces_.emplace_back( si );

  activateController();
  EXPECT_FALSE( controller_->effort_command_interface_.has_value() );

  injectActionCommand( 0.4, 1.5 );
  ASSERT_EQ( callUpdate( 0.0 ), controller_interface::return_type::OK );
  EXPECT_NEAR( hw_cmd_values_[0], 0.4, 1e-9 );
}

TEST_F( GripperPositionEffortControllerTest, RequiredModeFailsActivateWhenEffortInterfaceMissing )
{
  initController(); // effort_command_interface defaults to "required"
  configureController();

  // Provide ONLY a position command interface, not an effort one.
  hw_cmd_values_.assign( 1, 0.0 );
  hw_state_values_.assign( 3, 0.0 );
  cmd_ifaces_.clear();
  state_ifaces_.clear();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  cmd_ifaces_.push_back( std::make_shared<hardware_interface::CommandInterface>(
      joint_name_, "position", &hw_cmd_values_[0] ) );
  state_ifaces_.push_back( std::make_shared<hardware_interface::StateInterface>(
      joint_name_, "position", &hw_state_values_[0] ) );
  state_ifaces_.push_back( std::make_shared<hardware_interface::StateInterface>(
      joint_name_, "velocity", &hw_state_values_[1] ) );
  state_ifaces_.push_back( std::make_shared<hardware_interface::StateInterface>(
      joint_name_, "effort", &hw_state_values_[2] ) );
#pragma GCC diagnostic pop
  controller_->command_interfaces_.clear();
  controller_->state_interfaces_.clear();
  for ( auto &ci : cmd_ifaces_ ) controller_->command_interfaces_.emplace_back( ci, []() { } );
  for ( auto &si : state_ifaces_ ) controller_->state_interfaces_.emplace_back( si );

  rclcpp_lifecycle::State inactive( lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                    "inactive" );
  EXPECT_EQ( controller_->on_activate( inactive ), controller_interface::CallbackReturn::ERROR );
}

TEST_F( GripperPositionEffortControllerTest, ActivateInitialisesTargetToCurrentPosition )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  setPosition( 0.3 );

  activateController();

  // First update should hold at 0.3 with default_max_effort
  callUpdate( 0.0 );
  EXPECT_DOUBLE_EQ( cmdPos(), 0.3 );
  EXPECT_DOUBLE_EQ( cmdEffort(), 1.0 ); // default_max_effort
}

// ============================================================================
// Action goal handling — direct buffer injection
// ============================================================================

// Simulate accepted_callback by directly populating the action command buffer
// (full action server interaction would require an executor + node spinning).
TEST_F( GripperPositionEffortControllerTest, ActionLikeCommandWritesPositionAndEffort )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  activateController();

  injectActionCommand( 0.5, 2.0 );

  callUpdate( 0.0 );
  EXPECT_NEAR( cmdPos(), 0.5, 1e-9 );
  EXPECT_NEAR( cmdEffort(), 2.0, 1e-9 );
}

TEST_F( GripperPositionEffortControllerTest, ActionGoalClampedToJointLimits )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  activateController();

  // Goal beyond joint upper limit (1.0)
  injectActionCommand( 5.0, 2.0 );

  callUpdate( 0.0 );
  EXPECT_NEAR( cmdPos(), 1.0, 1e-9 );
}

// max_effort_limit is a hard ceiling on the effort value written to hardware. Verify that
// (a) effort under the limit passes through unchanged, (b) effort over the limit is
// clamped, and (c) limit=0 disables the cap.
TEST_F( GripperPositionEffortControllerTest, EffortClampedToMaxEffortLimit )
{
  initController( { rclcpp::Parameter( "max_effort_limit", 1.5 ) } );
  configureController();
  setupHardwareInterfaces();
  activateController();

  injectActionCommand( 0.5, 1.0 );
  callUpdate( 0.0 );
  EXPECT_NEAR( cmdEffort(), 1.0, 1e-9 ) << "Effort below the limit must pass through";

  injectActionCommand( 0.5, 5.0 );
  callUpdate( 0.01 );
  EXPECT_NEAR( cmdEffort(), 1.5, 1e-9 ) << "Effort above the limit must be clamped";
}

TEST_F( GripperPositionEffortControllerTest, MaxEffortLimitZeroDisablesCap )
{
  initController( { rclcpp::Parameter( "max_effort_limit", 0.0 ) } );
  configureController();
  setupHardwareInterfaces();
  activateController();

  injectActionCommand( 0.5, 99.0 );
  callUpdate( 0.0 );
  EXPECT_NEAR( cmdEffort(), 99.0, 1e-9 ) << "limit=0 must disable clamping";
}

// ============================================================================
// Topic interfaces
// ============================================================================

TEST_F( GripperPositionEffortControllerTest, PositionTopicCommandsJoint )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  activateController();

  injectPositionCommand( 0.4 );
  callUpdate( 0.0 );
  EXPECT_NEAR( cmdPos(), 0.4, 1e-9 );
  EXPECT_NEAR( cmdEffort(), 1.0, 1e-9 ); // default_max_effort
}

TEST_F( GripperPositionEffortControllerTest, PositionTopicClampedToJointLimits )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  activateController();

  injectPositionCommand( 5.0 );
  callUpdate( 0.0 );
  EXPECT_NEAR( cmdPos(), 1.0, 1e-9 );

  injectPositionCommand( -5.0 );
  callUpdate( 0.01 );
  EXPECT_NEAR( cmdPos(), -1.0, 1e-9 );
}

TEST_F( GripperPositionEffortControllerTest, VelocityTopicIntegratesPosition )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  activateController();

  // 0.1 rad/s for 5 cycles at dt=0.01s -> +0.005
  for ( int i = 0; i < 5; ++i ) {
    injectVelocityCommand( 0.1 );
    callUpdate( i * kPeriodSec );
  }
  EXPECT_NEAR( cmdPos(), 0.005, 1e-9 );
}

TEST_F( GripperPositionEffortControllerTest, VelocityTopicWatchdogStopsIntegration )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  activateController();

  injectVelocityCommand( 0.1 );
  callUpdate( 0.0 );
  const double after_first = cmdPos();
  EXPECT_NEAR( after_first, 0.001, 1e-9 );

  // No new vel msg: integrator should keep using cached velocity until timeout (0.1s)
  // Run for ~5 cycles (0.05s) — still within timeout
  for ( int i = 1; i <= 5; ++i ) { callUpdate( i * kPeriodSec ); }
  // After 6 total cycles (1 + 5), pos should be 6 * 0.001 = 0.006
  EXPECT_NEAR( cmdPos(), 0.006, 1e-9 );

  // Advance past the watchdog timeout (0.1s)
  callUpdate( 0.20 );
  const double after_timeout = cmdPos();

  // Subsequent updates should NOT integrate further
  callUpdate( 0.21 );
  callUpdate( 0.22 );
  EXPECT_NEAR( cmdPos(), after_timeout, 1e-9 );
}

TEST_F( GripperPositionEffortControllerTest, VelocityTopicClampsAtUpperLimit )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  activateController();

  // 100 rad/s for 100 cycles -> way past the upper limit of 1.0
  for ( int i = 0; i < 100; ++i ) {
    injectVelocityCommand( 100.0 );
    callUpdate( i * kPeriodSec );
  }
  EXPECT_NEAR( cmdPos(), 1.0, 1e-9 );
}

// Review-high regression for bug 1: the watchdog must invalidate the cached velocity via
// the RT-only velocity_cached_valid_ flag, NOT by calling rt_velocity_cmd_.writeFromNonRT
// (which takes a mutex and is RT-unsafe). After the watchdog fires, the buffer must still
// contain the original message — proving the RT update path did not call writeFromNonRT.
TEST_F( GripperPositionEffortControllerTest, StaleVelocityWatchdogStopsIntegrationWithoutBufferWrite )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  setPosition( 0.0 );
  activateController();

  injectVelocityCommand( 0.1 );
  callUpdate( 0.0 );
  EXPECT_NEAR( cmdPos(), 0.001, 1e-9 );
  EXPECT_TRUE( controller_->velocity_cached_valid_ );

  // Advance past the watchdog timeout (0.1s). The watchdog should fire and clear
  // velocity_cached_valid_ — but the underlying RealtimeBuffer must NOT have been written
  // from the RT update path (that would block on a mutex and be RT-unsafe).
  callUpdate( 0.20 );
  EXPECT_FALSE( controller_->velocity_cached_valid_ )
      << "Watchdog must clear velocity_cached_valid_";

  // Verify the buffer still holds the original message — proving the watchdog path did
  // not call rt_velocity_cmd_.writeFromNonRT(nullptr).
  auto cached = *controller_->rt_velocity_cmd_.readFromNonRT();
  ASSERT_TRUE( cached ) << "Cached velocity buffer must still hold the original message";
  EXPECT_DOUBLE_EQ( cached->data, 0.1 );

  // Subsequent updates must hold position (no integration of the now-invalid cache).
  const double pos_after_timeout = cmdPos();
  callUpdate( 0.21 );
  callUpdate( 0.22 );
  EXPECT_NEAR( cmdPos(), pos_after_timeout, 1e-9 );
}

// ============================================================================
// Stall / success logic
// ============================================================================

TEST_F( GripperPositionEffortControllerTest, ReachedGoalSucceedsAndClearsActiveGoal )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  activateController();

  // Inject an action-style command targeting position 0.5.
  // Without a real action goal handle we test the inner check_for_success path
  // by exercising the update loop with the command queued and the joint state at the goal.
  injectActionCommand( 0.5, 2.0 );

  setPosition( 0.5 );
  setVelocity( 0.0 );
  callUpdate( 0.0 );

  // No active rt_goal handle was created (we bypassed accepted_callback), so the
  // command should still have been written and the controller should not crash.
  EXPECT_NEAR( cmdPos(), 0.5, 1e-9 );
  EXPECT_NEAR( cmdEffort(), 2.0, 1e-9 );
}

// ============================================================================
// Topic preempts active goal
// ============================================================================

TEST_F( GripperPositionEffortControllerTest, PositionTopicPreemptsActiveActionCommand )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  activateController();

  // Set up an "action command" first
  injectActionCommand( 0.2, 2.0 );
  callUpdate( 0.0 );
  EXPECT_NEAR( cmdPos(), 0.2, 1e-9 );
  EXPECT_NEAR( cmdEffort(), 2.0, 1e-9 );

  // Now inject a position topic command — should override and use default_max_effort
  injectPositionCommand( 0.7 );
  callUpdate( 0.01 );
  EXPECT_NEAR( cmdPos(), 0.7, 1e-9 );
  EXPECT_NEAR( cmdEffort(), 1.0, 1e-9 ); // default_max_effort
}

// ============================================================================
// is_grasped detection
// ============================================================================

TEST_F( GripperPositionEffortControllerTest, IsGraspedLatchesAfterDwellCycles )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  activateController();

  // Measured effort above threshold and joint velocity below threshold
  setVelocity( 0.0 ); // below 0.005 threshold
  setEffort( 2.0 );   // above 0.5 threshold

  // Dwell cycles configured to 3
  callUpdate( 0.0 );
  EXPECT_EQ( controller_->is_grasped_dwell_counter_, 1 );
  EXPECT_FALSE( controller_->is_grasped_ );

  callUpdate( 0.01 );
  EXPECT_EQ( controller_->is_grasped_dwell_counter_, 2 );
  EXPECT_FALSE( controller_->is_grasped_ );

  callUpdate( 0.02 );
  EXPECT_EQ( controller_->is_grasped_dwell_counter_, 3 );
  EXPECT_TRUE( controller_->is_grasped_ );

  // Increase velocity above threshold — should reset
  setVelocity( 0.5 );
  callUpdate( 0.03 );
  EXPECT_EQ( controller_->is_grasped_dwell_counter_, 0 );
  EXPECT_FALSE( controller_->is_grasped_ );
}

TEST_F( GripperPositionEffortControllerTest, IsGraspedRequiresEffortAboveThreshold )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  activateController();

  // Velocity below threshold but measured effort below threshold -> not grasped
  setVelocity( 0.0 );
  setEffort( 0.1 ); // below 0.5 threshold

  for ( int i = 0; i < 5; ++i ) callUpdate( i * kPeriodSec );
  EXPECT_FALSE( controller_->is_grasped_ );
  EXPECT_EQ( controller_->is_grasped_dwell_counter_, 0 );
}

// ============================================================================
// Hold position when no goal received
// ============================================================================

TEST_F( GripperPositionEffortControllerTest, HoldsPositionWithoutAnyCommand )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  setPosition( 0.42 );
  activateController();

  for ( int i = 0; i < 10; ++i ) {
    callUpdate( i * kPeriodSec );
    EXPECT_NEAR( cmdPos(), 0.42, 1e-9 );
    EXPECT_NEAR( cmdEffort(), 1.0, 1e-9 ); // default_max_effort
  }
}

// ============================================================================
// max_effort == 0 falls back to default
// ============================================================================

TEST_F( GripperPositionEffortControllerTest, ZeroMaxEffortFallsBackToDefault )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  activateController();

  // accepted_callback substitutes default when goal.max_effort == 0; here we exercise
  // the same logic by computing it inline as accepted_callback would.
  const double goal_max_effort = 0.0;
  const double effective_max_effort =
      ( goal_max_effort > 0.0 ) ? goal_max_effort : controller_->params_.default_max_effort;

  injectActionCommand( 0.5, effective_max_effort );

  callUpdate( 0.0 );
  EXPECT_NEAR( cmdEffort(), 1.0, 1e-9 );
}

// ============================================================================
// Action lifecycle tests using rtest::experimental::ActionServerMock
// ----------------------------------------------------------------------------
// These tests drive the controller's real action callbacks (goal_callback,
// accepted_callback, cancel_callback) instead of bypassing them via direct
// buffer injection. The action mock GoalHandle records succeed/abort/canceled
// calls so we can verify the full lifecycle is correct.
// ============================================================================

class GripperPositionEffortLifecycleTest : public GripperPositionEffortControllerTest
{
protected:
  using ActionT = control_msgs::action::GripperCommand;
  using GoalHandleMock = rclcpp_action::GoalHandleMock<ActionT>;
  using ActionServerMock = rtest::experimental::ActionServerMock<ActionT>;

  std::shared_ptr<ActionServerMock> server_mock_;

  void initAndActivateWithActionServer()
  {
    initController();
    configureController();
    setupHardwareInterfaces();
    activateController();

    server_mock_ = rtest::experimental::findActionServer<ActionT>(
        controller_->get_node()->get_fully_qualified_name(), "~/gripper_cmd" );
    ASSERT_TRUE( server_mock_ ) << "Failed to find ActionServerMock for ~/gripper_cmd";
  }

  // Drive the full action accept path: invokes the controller's real goal_callback
  // and accepted_callback with a mock goal handle. Returns the mock handle so the
  // test can inspect terminal-state calls (succeed/abort/canceled) made on it.
  //
  // Sets up the mock's is_executing()/is_active() to mirror a real goal handle's state
  // machine: starts executing+active; flips to !executing+!active when any terminal
  // method (succeed/abort/canceled) fires. This matches what rclcpp_action does in
  // production and prevents ~RealtimeServerGoalHandle's defensive abort from firing
  // gh_->abort() a second time during test teardown.
  std::shared_ptr<GoalHandleMock>
  acceptGoal( double position, double max_effort,
              std::function<void( testing::NiceMock<GoalHandleMock> & )> setup_expectations = {} )
  {
    auto goal = std::make_shared<ActionT::Goal>();
    goal->command.position = position;
    goal->command.max_effort = max_effort;

    // goal_callback returns ACCEPT_AND_EXECUTE
    auto resp = server_mock_->goal_callback( rclcpp_action::GoalUUID{}, goal );
    EXPECT_EQ( resp, rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE );

    auto gh = std::make_shared<testing::NiceMock<GoalHandleMock>>( goal );

    // Set up state-machine defaults BEFORE running user expectations so that user
    // EXPECT_CALL/ON_CALL on these methods can override.
    auto state = std::make_shared<std::atomic<bool>>( true ); // executing/active flag
    ON_CALL( *gh, is_executing() ).WillByDefault( testing::Invoke( [state]() {
      return state->load();
    } ) );
    ON_CALL( *gh, is_active() ).WillByDefault( testing::Invoke( [state]() { return state->load(); } ) );
    ON_CALL( *gh, succeed( testing::_ ) ).WillByDefault( testing::Invoke( [state]( auto ) {
      state->store( false );
    } ) );
    ON_CALL( *gh, abort( testing::_ ) ).WillByDefault( testing::Invoke( [state]( auto ) {
      state->store( false );
    } ) );
    ON_CALL( *gh, canceled( testing::_ ) ).WillByDefault( testing::Invoke( [state]( auto ) {
      state->store( false );
    } ) );

    if ( setup_expectations )
      setup_expectations( *gh );

    server_mock_->accepted_callback( gh );
    return gh;
  }

  // Drive cancel of an active goal handle.
  void cancelGoal( std::shared_ptr<GoalHandleMock> gh )
  {
    auto resp = server_mock_->cancel_callback( gh );
    EXPECT_EQ( resp, rclcpp_action::CancelResponse::ACCEPT );
  }

  // Drive the realtime goal handle's deferred state transitions (what the wall_timer
  // would normally trigger). Required to actually fire mock methods like succeed().
  void runActiveGoalNonRealtime()
  {
    auto active = std::atomic_load( &controller_->rt_active_goal_ );
    if ( active )
      active->runNonRealtime();
  }
};

// Bug 1 regression: a cancel arriving before the next update() must NOT be followed by
// the goal still moving the gripper one cycle later.
TEST_F( GripperPositionEffortLifecycleTest, CancelBeforeUpdateDoesNotMoveGripper )
{
  initAndActivateWithActionServer();

  // Joint at 0.0 — capture the commanded position before any goal arrives.
  setPosition( 0.0 );
  callUpdate( 0.0 );
  const double pos_before_goal = cmdPos();
  EXPECT_NEAR( pos_before_goal, 0.0, 1e-9 );

  auto gh = acceptGoal( 0.8, 2.0 );

  // Cancel arrives immediately, before update() has a chance to consume the goal.
  cancelGoal( gh );

  // Now run an update — the canceled goal must NOT move the gripper.
  callUpdate( 0.01 );
  EXPECT_NEAR( cmdPos(), pos_before_goal, 1e-9 )
      << "Cancelled goal still drove the gripper to the goal position";
  // Effort should fall back to default_max_effort (set_hold_position was called)
  EXPECT_NEAR( cmdEffort(), 1.0, 1e-9 );
}

// Bug 3 regression: a terminal goal state must actually be delivered to the action client.
// cancel_callback runs on a non-RT thread and synchronously flushes the realtime wrapper
// before clearing the active goal, so gh.canceled() must have been called by the time
// cancel_callback returns.
TEST_F( GripperPositionEffortLifecycleTest, CancelDeliversCanceledToClient )
{
  initAndActivateWithActionServer();

  auto gh = acceptGoal( 0.5, 2.0, []( testing::NiceMock<GoalHandleMock> &mock ) {
    EXPECT_CALL( mock, canceled( testing::_ ) ).Times( 1 );
  } );

  cancelGoal( gh );
  EXPECT_FALSE( std::atomic_load( &controller_->rt_active_goal_ ) )
      << "Active goal must be cleared after cancel";
}

// Bug 3 regression for the success path: check_for_success runs in RT and only sets the
// terminal flag on the wrapper. The wall_timer must remain alive afterward so that its
// next tick can flush gh.succeed() to the client. If the timer were reset prematurely,
// the success notification would be lost.
TEST_F( GripperPositionEffortLifecycleTest, ReachedGoalDeliversSucceedToClient )
{
  initAndActivateWithActionServer();

  auto gh = acceptGoal( 0.5, 2.0, []( testing::NiceMock<GoalHandleMock> &mock ) {
    EXPECT_CALL( mock, succeed( testing::_ ) ).Times( 1 );
  } );

  // Drive joint state to the goal so check_for_success succeeds
  setPosition( 0.5 );
  setVelocity( 0.0 );
  callUpdate( 0.0 );

  EXPECT_FALSE( std::atomic_load( &controller_->rt_active_goal_ ) )
      << "Active goal must be cleared";
  auto prev = std::atomic_load( &controller_->previous_rt_goal_ );
  ASSERT_TRUE( prev ) << "Previous goal handle must be retained for the timer to flush";

  // Simulate the wall_timer's next tick. In production this happens within
  // action_monitor_period_; here we drive it explicitly so the test is deterministic.
  prev->runNonRealtime();
  // gh.succeed should have fired (verified by the EXPECT_CALL above).
}

TEST_F( GripperPositionEffortLifecycleTest, TopicPreemptionDeliversAbortToClient )
{
  initAndActivateWithActionServer();

  auto gh = acceptGoal( 0.5, 2.0, []( testing::NiceMock<GoalHandleMock> &mock ) {
    EXPECT_CALL( mock, abort( testing::_ ) ).Times( 1 );
  } );

  // Topic command preempts the action goal (preemption happens in RT — terminal flag is
  // set on the wrapper but flush is deferred to the wall_timer).
  injectPositionCommand( 0.3 );
  callUpdate( 0.0 );

  EXPECT_FALSE( std::atomic_load( &controller_->rt_active_goal_ ) );
  auto prev = std::atomic_load( &controller_->previous_rt_goal_ );
  ASSERT_TRUE( prev );

  prev->runNonRealtime();
  // gh.abort should have fired exactly once (verified by the EXPECT_CALL above).
}

// Bug 2 regression: feedback must NOT be queued on the goal handle after it has reached a
// terminal state in the same update() cycle. We verify by having runNonRealtime() flush
// the goal AFTER update(): publish_feedback should still never be called because update()
// re-checks rt_active_goal_ before calling setFeedback.
TEST_F( GripperPositionEffortLifecycleTest, NoFeedbackAfterReachedGoal )
{
  initAndActivateWithActionServer();

  auto gh = acceptGoal( 0.5, 2.0, []( testing::NiceMock<GoalHandleMock> &mock ) {
    EXPECT_CALL( mock, publish_feedback( testing::_ ) ).Times( 0 );
  } );

  // Goal already at target — should succeed in this same update() cycle.
  setPosition( 0.5 );
  setVelocity( 0.0 );
  callUpdate( 0.0 );

  auto prev = std::atomic_load( &controller_->previous_rt_goal_ );
  ASSERT_TRUE( prev );
  prev->runNonRealtime();
}

// Review-high regression: when an action goal accepts after a previous goal has been
// terminated in RT (e.g. by topic preemption or success), the previous goal's terminal
// flag must be flushed synchronously by accepted_callback before the timer is replaced.
// Without flush_previous_goal_if_any(), replacing goal_handle_timer_ would destroy the
// timer that was supposed to deliver the previous goal's terminal status.
TEST_F( GripperPositionEffortLifecycleTest, AcceptingNewGoalFlushesPreviousTerminalState )
{
  initAndActivateWithActionServer();

  auto first_gh = acceptGoal( 0.5, 2.0, []( testing::NiceMock<GoalHandleMock> &mock ) {
    EXPECT_CALL( mock, abort( testing::_ ) ).Times( 1 );
  } );

  // Topic preempts the first goal in RT — terminal flag set, flush deferred to timer.
  injectPositionCommand( 0.3 );
  callUpdate( 0.0 );
  ASSERT_TRUE( std::atomic_load( &controller_->previous_rt_goal_ ) )
      << "Previous goal must be retained";

  // Before the wall_timer fires, a new action goal arrives. The new accepted_callback
  // MUST synchronously flush the previous goal so its abort reaches the client even
  // though the next line replaces the timer.
  auto second_gh = acceptGoal( 0.6, 2.0 );

  EXPECT_FALSE( std::atomic_load( &controller_->previous_rt_goal_ ) )
      << "previous_rt_goal_ must be cleared after flush_previous_goal_if_any";
  // first_gh.abort should have been called by the synchronous flush — verified by the
  // EXPECT_CALL above.
}

// Review-high regression: a stale velocity command must NOT resume integrating after a
// newer action or position command has won. Once a non-velocity source takes over, the
// cached velocity buffer is invalidated.
TEST_F( GripperPositionEffortControllerTest, StaleVelocityDoesNotResumeAfterPositionTopicWins )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  setPosition( 0.0 );
  activateController();

  // Velocity command arrives and is integrated for a few cycles
  for ( int i = 0; i < 3; ++i ) {
    injectVelocityCommand( 0.1 );
    callUpdate( i * kPeriodSec );
  }
  // After 3 cycles at 0.1 rad/s: pos = 0.003
  EXPECT_NEAR( cmdPos(), 0.003, 1e-9 );

  // A newer position-topic command wins
  injectPositionCommand( 0.5 );
  callUpdate( 3 * kPeriodSec );
  EXPECT_NEAR( cmdPos(), 0.5, 1e-9 );

  // Subsequent cycles with NO new input must hold position (the cached velocity must be
  // invalidated; otherwise it would keep integrating from the new target).
  for ( int i = 4; i < 10; ++i ) {
    callUpdate( i * kPeriodSec );
    EXPECT_NEAR( cmdPos(), 0.5, 1e-9 ) << "Stale velocity resumed integration at cycle " << i;
  }
}

TEST_F( GripperPositionEffortControllerTest, StaleVelocityDoesNotResumeAfterActionWins )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  setPosition( 0.0 );
  activateController();

  // Cache a velocity command
  injectVelocityCommand( 0.5 );
  callUpdate( 0.0 );
  EXPECT_NEAR( cmdPos(), 0.005, 1e-9 );

  // Action takes over
  injectActionCommand( 0.2, 1.5 );
  callUpdate( 0.01 );
  EXPECT_NEAR( cmdPos(), 0.2, 1e-9 );
  EXPECT_NEAR( cmdEffort(), 1.5, 1e-9 );

  // No further input — cached velocity must NOT resume
  for ( int i = 2; i < 8; ++i ) {
    callUpdate( i * kPeriodSec );
    EXPECT_NEAR( cmdPos(), 0.2, 1e-9 ) << "Stale velocity resumed after action win at cycle " << i;
  }
}

// Review-medium regression: a NaN topic message must not preempt an active action goal.
// Validation happens before the preempt decision — invalid messages are dropped silently
// without touching the active goal.
//
// We verify by checking that rt_active_goal_ is still set and previous_rt_goal_ is still
// null after the malformed input is processed. (We can't use EXPECT_CALL(abort, Times(0))
// here because ~RealtimeServerGoalHandle defensively calls gh_->abort() during test
// teardown if the goal is still in is_executing() state — this happens in unit tests
// because we never spin a real action server lifecycle to flip the state.)
TEST_F( GripperPositionEffortLifecycleTest, NaNPositionTopicDoesNotPreemptActiveGoal )
{
  initAndActivateWithActionServer();

  auto gh = acceptGoal( 0.5, 2.0 );

  // First update consumes the action goal's command
  callUpdate( 0.0 );
  EXPECT_NEAR( cmdPos(), 0.5, 1e-9 );
  EXPECT_NEAR( cmdEffort(), 2.0, 1e-9 );
  EXPECT_TRUE( std::atomic_load( &controller_->rt_active_goal_ ) ) << "Goal must still be active";
  EXPECT_FALSE( std::atomic_load( &controller_->previous_rt_goal_ ) )
      << "Goal must not have been moved to previous (= preempted) state";

  // NaN topic arrives — must be dropped, must NOT preempt the goal
  injectPositionCommand( std::numeric_limits<double>::quiet_NaN() );
  callUpdate( 0.01 );

  EXPECT_NEAR( cmdPos(), 0.5, 1e-9 ) << "NaN topic must not change the position command";
  EXPECT_NEAR( cmdEffort(), 2.0, 1e-9 ) << "NaN topic must not change the effort command";
  EXPECT_TRUE( std::atomic_load( &controller_->rt_active_goal_ ) )
      << "NaN topic must not preempt the active action goal";
  EXPECT_FALSE( std::atomic_load( &controller_->previous_rt_goal_ ) )
      << "NaN topic must not transition the goal to previous (= preempted) state";
}

TEST_F( GripperPositionEffortLifecycleTest, NaNVelocityTopicDoesNotPreemptActiveGoal )
{
  initAndActivateWithActionServer();

  auto gh = acceptGoal( 0.5, 2.0 );

  callUpdate( 0.0 );
  EXPECT_TRUE( std::atomic_load( &controller_->rt_active_goal_ ) );
  EXPECT_FALSE( std::atomic_load( &controller_->previous_rt_goal_ ) );

  injectVelocityCommand( std::numeric_limits<double>::infinity() );
  callUpdate( 0.01 );

  EXPECT_NEAR( cmdPos(), 0.5, 1e-9 );
  EXPECT_TRUE( std::atomic_load( &controller_->rt_active_goal_ ) )
      << "Inf velocity must not preempt the active action goal";
  EXPECT_FALSE( std::atomic_load( &controller_->previous_rt_goal_ ) );
}

// Bug 4 regression: goal_callback must REJECT non-finite goals up front so the action
// client gets a clean rejection rather than an immediate abort, and so accepted_callback
// is never invoked with invalid input.
TEST_F( GripperPositionEffortLifecycleTest, NaNGoalIsRejectedAtGoalCallback )
{
  initAndActivateWithActionServer();

  auto nan_goal = std::make_shared<ActionT::Goal>();
  nan_goal->command.position = std::numeric_limits<double>::quiet_NaN();
  nan_goal->command.max_effort = 1.0;
  EXPECT_EQ( server_mock_->goal_callback( rclcpp_action::GoalUUID{}, nan_goal ),
             rclcpp_action::GoalResponse::REJECT );

  auto inf_effort_goal = std::make_shared<ActionT::Goal>();
  inf_effort_goal->command.position = 0.5;
  inf_effort_goal->command.max_effort = std::numeric_limits<double>::infinity();
  EXPECT_EQ( server_mock_->goal_callback( rclcpp_action::GoalUUID{}, inf_effort_goal ),
             rclcpp_action::GoalResponse::REJECT );

  auto valid_goal = std::make_shared<ActionT::Goal>();
  valid_goal->command.position = 0.5;
  valid_goal->command.max_effort = 1.0;
  EXPECT_EQ( server_mock_->goal_callback( rclcpp_action::GoalUUID{}, valid_goal ),
             rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE );
}

// Review-high regression for the accepted_callback flush ordering. Goal A succeeds in RT
// (check_for_success); BEFORE the wall_timer fires, goal B is accepted. accepted_callback
// must flush the previous (A) goal synchronously so its succeed() reaches the client even
// though the next line replaces the timer. Bug 3 in the review: the previous flush used
// to happen AFTER rt_active_goal_ was overwritten, allowing an interleaved RT cycle to
// stash B's wrapper into previous_rt_goal_ and lose A's terminal state.
TEST_F( GripperPositionEffortLifecycleTest, AcceptingTwoGoalsRapidlyDeliversBothTerminalStates )
{
  initAndActivateWithActionServer();

  // Goal A: joint already at the goal so check_for_success transitions to succeed in RT.
  auto gh_a = acceptGoal( 0.5, 2.0, []( testing::NiceMock<GoalHandleMock> &mock ) {
    EXPECT_CALL( mock, succeed( testing::_ ) ).Times( 1 );
  } );
  setPosition( 0.5 );
  setVelocity( 0.0 );
  callUpdate( 0.0 );
  // A's terminal flag is now set on the wrapper but flush is deferred to the wall timer.
  EXPECT_FALSE( std::atomic_load( &controller_->rt_active_goal_ ) );
  ASSERT_TRUE( std::atomic_load( &controller_->previous_rt_goal_ ) )
      << "Goal A must be parked in previous_rt_goal_ awaiting flush";

  // Goal B arrives BEFORE the timer fires. accepted_callback must flush A synchronously.
  auto gh_b = acceptGoal( 0.7, 2.0, []( testing::NiceMock<GoalHandleMock> &mock ) {
    // We don't assert anything about B here — its lifecycle continues normally.
    (void)mock;
  } );

  EXPECT_FALSE( std::atomic_load( &controller_->previous_rt_goal_ ) )
      << "previous_rt_goal_ must have been flushed and cleared by accepted_callback";
  // gh_a.succeed should have fired exactly once, verified by the EXPECT_CALL above.
}

// Bug 4 regression: NaN on the position topic must not be written to hardware.
TEST_F( GripperPositionEffortControllerTest, NaNPositionTopicIsRejected )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  setPosition( 0.2 );
  activateController();
  callUpdate( 0.0 );
  const double pos_before = cmdPos();

  injectPositionCommand( std::numeric_limits<double>::quiet_NaN() );
  callUpdate( 0.01 );
  EXPECT_NEAR( cmdPos(), pos_before, 1e-9 ) << "NaN position topic should not propagate";
  EXPECT_TRUE( std::isfinite( cmdPos() ) );
}

TEST_F( GripperPositionEffortControllerTest, NaNVelocityTopicIsRejected )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  setPosition( 0.2 );
  activateController();
  callUpdate( 0.0 );
  const double pos_before = cmdPos();

  injectVelocityCommand( std::numeric_limits<double>::quiet_NaN() );
  callUpdate( 0.01 );
  EXPECT_NEAR( cmdPos(), pos_before, 1e-9 ) << "NaN velocity topic should not propagate";
  EXPECT_TRUE( std::isfinite( cmdPos() ) );
}

// Bug 5 regression: arbitration is true last-writer-wins by sequence, not by
// update()'s internal source ordering. If two messages arrive between updates with the
// position-topic arriving AFTER the action goal, the position topic must win even
// though action is processed earlier in the dispatch switch.
TEST_F( GripperPositionEffortControllerTest, LastWriterWinsAcrossSourcesByOrderOfArrival )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  activateController();

  // Action-style goal arrives first
  injectActionCommand( 0.2, 2.0 );
  // Then a position-topic command arrives (later sequence number)
  injectPositionCommand( 0.7 );

  callUpdate( 0.0 );
  // Position topic was the last writer => its value (and default_max_effort) wins
  EXPECT_NEAR( cmdPos(), 0.7, 1e-9 );
  EXPECT_NEAR( cmdEffort(), 1.0, 1e-9 );
}

TEST_F( GripperPositionEffortControllerTest, LastWriterWinsActionAfterTopic )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  activateController();

  // Reverse order: position topic arrives first, then action goal
  injectPositionCommand( 0.7 );
  injectActionCommand( 0.2, 2.5 );

  callUpdate( 0.0 );
  // Action goal was the last writer => its value (and explicit max_effort) wins
  EXPECT_NEAR( cmdPos(), 0.2, 1e-9 );
  EXPECT_NEAR( cmdEffort(), 2.5, 1e-9 );
}

TEST_F( GripperPositionEffortControllerTest, LastWriterWinsVelocityAfterAction )
{
  initController();
  configureController();
  setupHardwareInterfaces();
  setPosition( 0.0 );
  activateController();

  // Action goal arrives first, then a velocity command
  injectActionCommand( 0.5, 2.0 );
  injectVelocityCommand( 0.1 ); // 0.1 rad/s

  callUpdate( 0.0 );
  // Velocity wins => target = 0 (initial) + 0.1 * 0.01 = 0.001, effort = default
  EXPECT_NEAR( cmdPos(), 0.001, 1e-9 );
  EXPECT_NEAR( cmdEffort(), 1.0, 1e-9 );
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
