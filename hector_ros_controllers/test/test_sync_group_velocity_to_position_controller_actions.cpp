#include "test_helpers.hpp"

#include <rtest/action_server_mock.hpp>

#include <chrono>
#include <thread>

using VelToPosController =
    sync_group_velocity_to_position_controller::SyncGroupVelocityToPositionController;
using GroupActionState = sync_group_velocity_to_position_controller::GroupActionState;
using DriveFlipperGroupAction = hector_ros_controllers_msgs::action::DriveFlipperGroup;
using SyncFlipperGroupAction = hector_ros_controllers_msgs::action::SyncFlipperGroup;

// ============================================================================
// Action Test Fixture
//
// Builds on the same hardware-interface scaffolding as
// test_sync_group_velocity_to_position_controller.cpp, plus rtest's
// findActionServer to drive the goal/cancel/accepted callbacks directly.
//
// `handle_*_accepted` spawns a monitor thread that polls is_canceling() on the
// goal handle at 20 Hz; we use `WillRepeatedly` so the mock survives polling.
// We tick `update_and_write_commands` in a separate thread to advance time so
// the monitor thread observes COMPLETED.
// ============================================================================

class VelToPosControllerActionTest : public ::testing::Test
{
protected:
  static constexpr unsigned int kUpdateRate = 100;

  std::vector<std::string> joints_{ "joint1", "joint2", "joint3" };

  std::shared_ptr<VelToPosController> controller_;
  std::vector<double> hw_cmd_values_;
  std::vector<double> hw_state_values_;
  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> cmd_ifaces_;
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> state_ifaces_;

  void SetUp() override { controller_ = std::make_shared<VelToPosController>(); }

  void TearDown() override
  {
    if ( controller_ ) {
      // Deactivate to drain monitor threads before tearing down mock goal handles.
      try {
        rclcpp_lifecycle::State active( lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                        "active" );
        controller_->on_deactivate( active );
      } catch ( ... ) {
      }
    }
    controller_.reset();
    rtest::StaticMocksRegistry::instance().reset();
  }

  void initController( const std::vector<std::string> &sync_groups )
  {
    const auto urdf = hector_test::loadUrdfFile( "test_robot.urdf" );

    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "test_vel_to_pos_actions";
    params.robot_description = urdf;
    params.update_rate = kUpdateRate;
    params.controller_manager_update_rate = kUpdateRate;
    params.node_namespace = "";

    std::vector<rclcpp::Parameter> overrides = {
        rclcpp::Parameter( "joints", joints_ ),
        rclcpp::Parameter( "kp", 2.0 ),
        rclcpp::Parameter( "kd", 0.1 ),
        rclcpp::Parameter( "kp_sync", 1.0 ),
        rclcpp::Parameter( "passthrough_controller", std::string( "" ) ),
        rclcpp::Parameter( "e_stop_topic", std::string( "estop_board/hard_estop" ) ),
        rclcpp::Parameter( "synchronous_groups", sync_groups ),
        // Keep profiles short so the monitor thread sees COMPLETED quickly.
        rclcpp::Parameter( "max_velocity", 5.0 ),
        rclcpp::Parameter( "max_acceleration", 20.0 ),
    };

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

    controller_->reference_interfaces_.assign( joints_.size(),
                                               std::numeric_limits<double>::quiet_NaN() );
  }

  void setupHardwareInterfaces()
  {
    hw_cmd_values_.assign( joints_.size(), 0.0 );
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

  // Use the node's clock (matches what the controller uses for cmd.start_time
  // and what the monitor thread polls), so elapsed-time calculations agree.
  void callUpdate()
  {
    rclcpp::Time now = controller_->get_node()->now();
    rclcpp::Duration period( std::chrono::milliseconds( 10 ) );
    controller_->update_and_write_commands( now, period );
  }

  // Tick the controller for `count` cycles, sleeping briefly so the action
  // monitor thread (running at 20 Hz with rclcpp::Rate) gets to observe state
  // transitions between ticks.
  void tickWithMonitorPump( int count )
  {
    for ( int i = 0; i < count; ++i ) {
      callUpdate();
      std::this_thread::sleep_for( std::chrono::milliseconds( 5 ) );
    }
  }

  // Set position state for a joint by index
  void setPosition( size_t idx, double value ) { hw_state_values_[2 * idx] = value; }
};

using ::testing::_;
using ::testing::Return;
using ::testing::SaveArg;

// ============================================================================
// DriveFlipperGroup tests
// ============================================================================

// Goal handler must reject unknown groups
TEST_F( VelToPosControllerActionTest, DriveFlipperGroup_RejectsUnknownGroup )
{
  initController( { "g_a", "g_a", "g_b" } );
  configureController();
  setupHardwareInterfaces();
  activateController();

  auto server_mock = rtest::experimental::findActionServer<DriveFlipperGroupAction>(
      controller_->get_node(), "~/drive_flipper_group" );
  ASSERT_TRUE( server_mock );

  auto goal = std::make_shared<DriveFlipperGroupAction::Goal>();
  goal->group_name = "does_not_exist";
  goal->target_position = 0.5;

  rclcpp_action::GoalUUID uuid{};
  auto response = server_mock->goal_callback( uuid, goal );
  EXPECT_EQ( response, rclcpp_action::GoalResponse::REJECT );
}

// handle_drive_accepted aborts when a joint has invalid (NaN) position state
TEST_F( VelToPosControllerActionTest, DriveFlipperGroup_AbortsOnInvalidPosition )
{
  initController( { "g_a", "g_a", "g_b" } );
  configureController();
  setupHardwareInterfaces();
  activateController();

  auto server_mock = rtest::experimental::findActionServer<DriveFlipperGroupAction>(
      controller_->get_node(), "~/drive_flipper_group" );
  ASSERT_TRUE( server_mock );

  // Force the snapshot to contain NaN for joint 0 (in g_a).
  std::vector<double> nan_positions( joints_.size(), 0.0 );
  nan_positions[0] = std::numeric_limits<double>::quiet_NaN();
  controller_->rt_joint_position_snapshot_.set( nan_positions );

  auto goal = std::make_shared<DriveFlipperGroupAction::Goal>();
  goal->group_name = "g_a";
  goal->target_position = 0.5;

  auto mock_goal_handle = rtest::experimental::createMockGoalHandle<DriveFlipperGroupAction>( goal );

  std::shared_ptr<DriveFlipperGroupAction::Result> captured_result;
  EXPECT_CALL( *mock_goal_handle, abort( _ ) ).Times( 1 ).WillOnce( SaveArg<0>( &captured_result ) );

  controller_->handle_drive_accepted( mock_goal_handle );

  ASSERT_TRUE( captured_result );
  EXPECT_FALSE( captured_result->success );
  EXPECT_NE( captured_result->message.find( "invalid position" ), std::string::npos );
}

// Happy path: goal is accepted, profile completes, succeed() is called with success=true
TEST_F( VelToPosControllerActionTest, DriveFlipperGroup_HappyPath )
{
  initController( { "g_a", "g_a", "g_b" } );
  configureController();
  setupHardwareInterfaces();
  activateController();

  auto server_mock = rtest::experimental::findActionServer<DriveFlipperGroupAction>(
      controller_->get_node(), "~/drive_flipper_group" );
  ASSERT_TRUE( server_mock );

  auto goal = std::make_shared<DriveFlipperGroupAction::Goal>();
  goal->group_name = "g_a";
  goal->target_position = 0.2;
  goal->max_velocity = 5.0;
  goal->max_acceleration = 20.0;

  auto mock_goal_handle = rtest::experimental::createMockGoalHandle<DriveFlipperGroupAction>( goal );

  // Monitor thread polls these — must allow repeated calls.
  ON_CALL( *mock_goal_handle, is_canceling() ).WillByDefault( Return( false ) );
  EXPECT_CALL( *mock_goal_handle, is_canceling() ).Times( ::testing::AnyNumber() );
  EXPECT_CALL( *mock_goal_handle, publish_feedback( _ ) ).Times( ::testing::AnyNumber() );

  std::shared_ptr<DriveFlipperGroupAction::Result> captured_result;
  EXPECT_CALL( *mock_goal_handle, succeed( _ ) ).Times( 1 ).WillOnce( SaveArg<0>( &captured_result ) );
  EXPECT_CALL( *mock_goal_handle, abort( _ ) ).Times( 0 );
  EXPECT_CALL( *mock_goal_handle, canceled( _ ) ).Times( 0 );

  // Goal acceptance kicks off the monitor thread.
  rclcpp_action::GoalUUID uuid{};
  ASSERT_EQ( server_mock->goal_callback( uuid, goal ),
             rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE );
  controller_->handle_drive_accepted( mock_goal_handle );

  // Drive the controller forward. Profile: 0->0.2, max_vel=5, max_accel=20.
  // dist_for_full = 25/20 = 1.25; 0.2 < 1.25 -> triangular.
  // v_peak = sqrt(0.2*20) = 2.0; t_accel = 2.0/20 = 0.1s -> total = 0.2s = 20 cycles.
  // Tick generously past completion + monitor poll interval (50ms = 5 cycles).
  tickWithMonitorPump( 60 );

  // Allow the monitor thread one more poll iteration to call succeed().
  std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );

  ASSERT_TRUE( captured_result ) << "succeed() was never called";
  EXPECT_TRUE( captured_result->success );
}

// A non-zero velocity command on a joint in the action's group cancels the goal.
// monitor_group_actions reports this as on_abort("Cancelled by velocity command")
// and routes it to abort() (the "client" string check is for client-cancel only).
TEST_F( VelToPosControllerActionTest, DriveFlipperGroup_CancelledByVelocityCommand )
{
  initController( { "g_a", "g_a", "g_b" } );
  configureController();
  setupHardwareInterfaces();
  activateController();

  auto server_mock = rtest::experimental::findActionServer<DriveFlipperGroupAction>(
      controller_->get_node(), "~/drive_flipper_group" );
  ASSERT_TRUE( server_mock );

  auto goal = std::make_shared<DriveFlipperGroupAction::Goal>();
  goal->group_name = "g_a";
  goal->target_position = 1.0;
  goal->max_velocity = 0.5;
  goal->max_acceleration = 1.0;

  auto mock_goal_handle = rtest::experimental::createMockGoalHandle<DriveFlipperGroupAction>( goal );

  ON_CALL( *mock_goal_handle, is_canceling() ).WillByDefault( Return( false ) );
  EXPECT_CALL( *mock_goal_handle, is_canceling() ).Times( ::testing::AnyNumber() );
  EXPECT_CALL( *mock_goal_handle, publish_feedback( _ ) ).Times( ::testing::AnyNumber() );

  std::shared_ptr<DriveFlipperGroupAction::Result> captured_result;
  EXPECT_CALL( *mock_goal_handle, abort( _ ) ).Times( 1 ).WillOnce( SaveArg<0>( &captured_result ) );
  EXPECT_CALL( *mock_goal_handle, succeed( _ ) ).Times( 0 );

  rclcpp_action::GoalUUID uuid{};
  ASSERT_EQ( server_mock->goal_callback( uuid, goal ),
             rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE );
  controller_->handle_drive_accepted( mock_goal_handle );

  // Tick a few cycles, then inject a velocity command on joint 0 in g_a.
  tickWithMonitorPump( 5 );
  controller_->reference_interfaces_[0] = 0.5;
  tickWithMonitorPump( 30 );

  std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );

  ASSERT_TRUE( captured_result ) << "abort() was never called";
  EXPECT_FALSE( captured_result->success );
  EXPECT_NE( captured_result->message.find( "velocity" ), std::string::npos );
}

// ============================================================================
// SyncFlipperGroup tests
// ============================================================================

// goal_callback rejects unknown group names
TEST_F( VelToPosControllerActionTest, SyncFlipperGroup_RejectsUnknownGroup )
{
  initController( { "g_a", "g_a", "g_b" } );
  configureController();
  setupHardwareInterfaces();
  activateController();

  auto server_mock = rtest::experimental::findActionServer<SyncFlipperGroupAction>(
      controller_->get_node(), "~/sync_flipper_group" );
  ASSERT_TRUE( server_mock );

  auto goal = std::make_shared<SyncFlipperGroupAction::Goal>();
  goal->group_names = { "does_not_exist" };

  rclcpp_action::GoalUUID uuid{};
  auto response = server_mock->goal_callback( uuid, goal );
  EXPECT_EQ( response, rclcpp_action::GoalResponse::REJECT );
}

// Empty group_names should sync ALL groups; result vector ordering matches
// the deterministically sorted group_names_.
TEST_F( VelToPosControllerActionTest, SyncFlipperGroup_AllGroups_HappyPath )
{
  initController( { "g_a", "g_a", "g_b" } );
  configureController();
  setupHardwareInterfaces();

  // joint0 = 0.0, joint1 = 0.4 -> g_a avg = 0.2
  // joint2 = 0.6 -> g_b avg = 0.6
  setPosition( 0, 0.0 );
  setPosition( 1, 0.4 );
  setPosition( 2, 0.6 );
  activateController();

  auto server_mock = rtest::experimental::findActionServer<SyncFlipperGroupAction>(
      controller_->get_node(), "~/sync_flipper_group" );
  ASSERT_TRUE( server_mock );

  auto goal = std::make_shared<SyncFlipperGroupAction::Goal>();
  // Empty group_names -> sync all
  goal->max_velocity = 5.0;
  goal->max_acceleration = 20.0;

  auto mock_goal_handle = rtest::experimental::createMockGoalHandle<SyncFlipperGroupAction>( goal );

  ON_CALL( *mock_goal_handle, is_canceling() ).WillByDefault( Return( false ) );
  EXPECT_CALL( *mock_goal_handle, is_canceling() ).Times( ::testing::AnyNumber() );
  EXPECT_CALL( *mock_goal_handle, publish_feedback( _ ) ).Times( ::testing::AnyNumber() );

  std::shared_ptr<SyncFlipperGroupAction::Result> captured_result;
  EXPECT_CALL( *mock_goal_handle, succeed( _ ) ).Times( 1 ).WillOnce( SaveArg<0>( &captured_result ) );

  rclcpp_action::GoalUUID uuid{};
  ASSERT_EQ( server_mock->goal_callback( uuid, goal ),
             rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE );
  controller_->handle_sync_accepted( mock_goal_handle );

  // Two profiles run in parallel. The longer one (g_a needs to move 0.2 from 0/0.4
  // to 0.2 — both joints move 0.2). With max_vel=5, max_accel=20: triangular profile,
  // total ~0.2s = 20 cycles. Tick generously.
  tickWithMonitorPump( 80 );
  std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );

  ASSERT_TRUE( captured_result ) << "succeed() was never called";
  EXPECT_TRUE( captured_result->success );
  // synced_positions ordering follows the (sorted) target_groups order.
  // With empty group_names, target_groups = group_names_ which is sorted: [g_a, g_b]
  ASSERT_EQ( captured_result->synced_positions.size(), 2u );
  EXPECT_NEAR( captured_result->synced_positions[0], 0.2, 1e-6 );
  EXPECT_NEAR( captured_result->synced_positions[1], 0.6, 1e-6 );
}

// Subset: explicit group_names = ["g_a"] only syncs g_a; g_b is not driven.
TEST_F( VelToPosControllerActionTest, SyncFlipperGroup_PerGroupSubset )
{
  initController( { "g_a", "g_a", "g_b" } );
  configureController();
  setupHardwareInterfaces();

  setPosition( 0, 0.0 );
  setPosition( 1, 0.2 );
  setPosition( 2, 0.6 ); // g_b — should not be touched
  activateController();

  auto server_mock = rtest::experimental::findActionServer<SyncFlipperGroupAction>(
      controller_->get_node(), "~/sync_flipper_group" );
  ASSERT_TRUE( server_mock );

  auto goal = std::make_shared<SyncFlipperGroupAction::Goal>();
  goal->group_names = { "g_a" };
  goal->max_velocity = 5.0;
  goal->max_acceleration = 20.0;

  auto mock_goal_handle = rtest::experimental::createMockGoalHandle<SyncFlipperGroupAction>( goal );

  ON_CALL( *mock_goal_handle, is_canceling() ).WillByDefault( Return( false ) );
  EXPECT_CALL( *mock_goal_handle, is_canceling() ).Times( ::testing::AnyNumber() );
  EXPECT_CALL( *mock_goal_handle, publish_feedback( _ ) ).Times( ::testing::AnyNumber() );

  std::shared_ptr<SyncFlipperGroupAction::Result> captured_result;
  EXPECT_CALL( *mock_goal_handle, succeed( _ ) ).Times( 1 ).WillOnce( SaveArg<0>( &captured_result ) );

  rclcpp_action::GoalUUID uuid{};
  ASSERT_EQ( server_mock->goal_callback( uuid, goal ),
             rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE );
  controller_->handle_sync_accepted( mock_goal_handle );

  tickWithMonitorPump( 80 );
  std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );

  ASSERT_TRUE( captured_result );
  EXPECT_TRUE( captured_result->success );
  ASSERT_EQ( captured_result->synced_positions.size(), 1u );
  EXPECT_NEAR( captured_result->synced_positions[0], 0.1, 1e-6 ); // (0.0 + 0.2) / 2

  // g_b's group action state must have remained IDLE the entire run.
  size_t g_b_idx = controller_->group_index_map_["g_b"];
  EXPECT_EQ( controller_->group_action_states_[g_b_idx].load(), GroupActionState::IDLE );
}

// Verify partial-failure rollback: if an early group starts successfully but
// a later group fails (e.g. all-NaN positions), the already-started group must
// be cancelled before the goal is aborted. Otherwise the operator gets a
// "failed" result while the robot keeps moving the early group.
TEST_F( VelToPosControllerActionTest, SyncFlipperGroup_PartialFailureRollsBackStartedGroups )
{
  // Use deterministic group names so g_a sorts before g_b.
  initController( { "g_a", "g_a", "g_b" } );
  configureController();
  setupHardwareInterfaces();
  activateController();

  // Inject a position snapshot where g_a (joints 0,1) is valid but g_b (joint 2)
  // has NaN — that makes valid_count==0 for g_b and triggers the abort path.
  std::vector<double> positions{ 0.0, 0.4, std::numeric_limits<double>::quiet_NaN() };
  controller_->rt_joint_position_snapshot_.set( positions );

  auto goal = std::make_shared<SyncFlipperGroupAction::Goal>();
  // Empty group_names -> all groups, in sorted order: [g_a, g_b]. g_a starts,
  // then g_b fails with valid_count==0.
  goal->max_velocity = 5.0;
  goal->max_acceleration = 20.0;

  auto mock_goal_handle = rtest::experimental::createMockGoalHandle<SyncFlipperGroupAction>( goal );

  std::shared_ptr<SyncFlipperGroupAction::Result> captured_result;
  EXPECT_CALL( *mock_goal_handle, abort( _ ) ).Times( 1 ).WillOnce( SaveArg<0>( &captured_result ) );
  EXPECT_CALL( *mock_goal_handle, succeed( _ ) ).Times( 0 );

  controller_->handle_sync_accepted( mock_goal_handle );

  ASSERT_TRUE( captured_result );
  EXPECT_FALSE( captured_result->success );
  EXPECT_NE( captured_result->message.find( "valid joint positions" ), std::string::npos );

  // Both groups must end in a non-EXECUTING state. g_a was started and must be
  // rolled back to IDLE; g_b never started so it stays IDLE.
  size_t g_a_idx = controller_->group_index_map_["g_a"];
  size_t g_b_idx = controller_->group_index_map_["g_b"];
  EXPECT_EQ( controller_->group_action_states_[g_a_idx].load(), GroupActionState::IDLE )
      << "g_a should have been rolled back to IDLE after g_b's failure";
  EXPECT_EQ( controller_->group_action_states_[g_b_idx].load(), GroupActionState::IDLE );
}

// ============================================================================
// Overlap rejection tests
// ============================================================================

// A second drive goal targeting an EXECUTING group must be REJECTed by the
// goal callback, not silently accepted (which would hijack the first goal).
TEST_F( VelToPosControllerActionTest, DriveFlipperGroup_RejectsOverlappingGoal )
{
  initController( { "g_a", "g_a", "g_b" } );
  configureController();
  setupHardwareInterfaces();
  activateController();

  auto server_mock = rtest::experimental::findActionServer<DriveFlipperGroupAction>(
      controller_->get_node(), "~/drive_flipper_group" );
  ASSERT_TRUE( server_mock );

  // Force g_a into EXECUTING without spawning a real monitor thread.
  size_t g_a_idx = controller_->group_index_map_["g_a"];
  controller_->group_action_states_[g_a_idx].store( GroupActionState::EXECUTING );

  // Second goal on the same group must be rejected.
  auto goal = std::make_shared<DriveFlipperGroupAction::Goal>();
  goal->group_name = "g_a";
  goal->target_position = 0.5;

  rclcpp_action::GoalUUID uuid{};
  EXPECT_EQ( server_mock->goal_callback( uuid, goal ), rclcpp_action::GoalResponse::REJECT );

  // Reset so TearDown's deactivate doesn't try to cancel a fake goal handle.
  controller_->group_action_states_[g_a_idx].store( GroupActionState::IDLE );
}

// SyncFlipperGroup with explicit group_names must reject when any target group
// is already executing.
TEST_F( VelToPosControllerActionTest, SyncFlipperGroup_RejectsWhenAnyTargetGroupExecuting )
{
  initController( { "g_a", "g_a", "g_b" } );
  configureController();
  setupHardwareInterfaces();
  activateController();

  auto server_mock = rtest::experimental::findActionServer<SyncFlipperGroupAction>(
      controller_->get_node(), "~/sync_flipper_group" );
  ASSERT_TRUE( server_mock );

  // g_b is busy.
  size_t g_b_idx = controller_->group_index_map_["g_b"];
  controller_->group_action_states_[g_b_idx].store( GroupActionState::EXECUTING );

  auto goal = std::make_shared<SyncFlipperGroupAction::Goal>();
  goal->group_names = { "g_a", "g_b" };
  rclcpp_action::GoalUUID uuid{};
  EXPECT_EQ( server_mock->goal_callback( uuid, goal ), rclcpp_action::GoalResponse::REJECT );

  // Same with empty group_names (= all groups), which includes g_b.
  auto goal_all = std::make_shared<SyncFlipperGroupAction::Goal>();
  EXPECT_EQ( server_mock->goal_callback( uuid, goal_all ), rclcpp_action::GoalResponse::REJECT );

  // A goal that targets only g_a (not busy) must still be accepted.
  auto goal_a = std::make_shared<SyncFlipperGroupAction::Goal>();
  goal_a->group_names = { "g_a" };
  EXPECT_EQ( server_mock->goal_callback( uuid, goal_a ),
             rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE );

  controller_->group_action_states_[g_b_idx].store( GroupActionState::IDLE );
}

// ============================================================================
// Monitor thread reaping
// ============================================================================

// A second goal accepted after the first completes should reap the first's
// monitor thread, keeping action_monitor_threads_ small.
TEST_F( VelToPosControllerActionTest, MonitorThread_ReapsAfterCompletion )
{
  initController( { "g_a", "g_a", "g_b" } );
  configureController();
  setupHardwareInterfaces();
  activateController();

  auto server_mock = rtest::experimental::findActionServer<DriveFlipperGroupAction>(
      controller_->get_node(), "~/drive_flipper_group" );
  ASSERT_TRUE( server_mock );

  // First goal — short profile so it completes quickly.
  auto goal1 = std::make_shared<DriveFlipperGroupAction::Goal>();
  goal1->group_name = "g_a";
  goal1->target_position = 0.05;
  goal1->max_velocity = 5.0;
  goal1->max_acceleration = 20.0;

  auto mgh1 = rtest::experimental::createMockGoalHandle<DriveFlipperGroupAction>( goal1 );
  ON_CALL( *mgh1, is_canceling() ).WillByDefault( Return( false ) );
  EXPECT_CALL( *mgh1, is_canceling() ).Times( ::testing::AnyNumber() );
  EXPECT_CALL( *mgh1, publish_feedback( _ ) ).Times( ::testing::AnyNumber() );
  EXPECT_CALL( *mgh1, succeed( _ ) ).Times( 1 );

  rclcpp_action::GoalUUID u1{};
  ASSERT_EQ( server_mock->goal_callback( u1, goal1 ),
             rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE );
  controller_->handle_drive_accepted( mgh1 );

  tickWithMonitorPump( 60 );
  std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );

  // Now submit a second goal; reap_finished_monitor_threads in handle_drive_accepted
  // should remove the first thread's entry before adding the new one.
  auto goal2 = std::make_shared<DriveFlipperGroupAction::Goal>();
  goal2->group_name = "g_a";
  goal2->target_position = 0.1;
  goal2->max_velocity = 5.0;
  goal2->max_acceleration = 20.0;

  auto mgh2 = rtest::experimental::createMockGoalHandle<DriveFlipperGroupAction>( goal2 );
  ON_CALL( *mgh2, is_canceling() ).WillByDefault( Return( false ) );
  EXPECT_CALL( *mgh2, is_canceling() ).Times( ::testing::AnyNumber() );
  EXPECT_CALL( *mgh2, publish_feedback( _ ) ).Times( ::testing::AnyNumber() );
  EXPECT_CALL( *mgh2, succeed( _ ) ).Times( 1 );

  rclcpp_action::GoalUUID u2{};
  ASSERT_EQ( server_mock->goal_callback( u2, goal2 ),
             rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE );
  controller_->handle_drive_accepted( mgh2 );

  // After reaping, exactly one thread (the new one) should remain.
  {
    std::lock_guard<std::mutex> lock( controller_->action_monitor_threads_mutex_ );
    EXPECT_EQ( controller_->action_monitor_threads_.size(), 1u )
        << "First monitor thread should have been reaped before the second was added";
  }

  tickWithMonitorPump( 60 );
  std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
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
