#include "test_helpers.hpp"

using SelfCollisionAvoidanceController =
    self_collision_avoidance_controller::SelfCollisionAvoidanceController;

class SelfCollisionAvoidanceControllerTest : public ::testing::Test
{
protected:
  const std::vector<std::string> joints_{ "joint1", "joint2", "joint3" };
  std::shared_ptr<SelfCollisionAvoidanceController> controller_;
  hector_test::HandleValues hw_cmd_values_;
  hector_test::HandleValues hw_state_values_;
  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> cmd_ifaces_;
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> state_ifaces_;

  void SetUp() override { controller_ = std::make_shared<SelfCollisionAvoidanceController>(); }

  void TearDown() override
  {
    controller_.reset();
    rtest::StaticMocksRegistry::instance().reset();
  }

  void configureController()
  {
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "test_sca";
    params.robot_description = hector_test::loadUrdfFile( "test_robot.urdf" );
    params.update_rate = 100;
    params.controller_manager_update_rate = 100;
    rclcpp::NodeOptions opts;
    opts.parameter_overrides(
        { rclcpp::Parameter( "joints", joints_ ),
          rclcpp::Parameter( "joint_interface_types",
                             std::vector<std::string>{ "position", "position", "position" } ),
          rclcpp::Parameter( "robot_description_semantic",
                             std::string( "<robot name=\"test_robot\"></robot>" ) ) } );
    params.node_options = opts;
    ASSERT_EQ( controller_->init( params ), controller_interface::return_type::OK );
    ASSERT_EQ( controller_->on_configure( rclcpp_lifecycle::State() ),
               controller_interface::CallbackReturn::SUCCESS );
    ASSERT_EQ( controller_->export_reference_interfaces().size(), joints_.size() );

    hw_cmd_values_.assign( joints_.size(), 0.0 );
    for ( std::size_t i = 0; i < joints_.size(); ++i ) {
      cmd_ifaces_.push_back( hw_cmd_values_.make_command( i, joints_[i], "position" ) );
      controller_->command_interfaces_.emplace_back( cmd_ifaces_.back(), []() { } );
    }
    // The collision model reads every movable joint of the robot
    const auto state_names = controller_->state_interface_configuration().names;
    hw_state_values_.assign( state_names.size(), 0.0 );
    for ( std::size_t i = 0; i < state_names.size(); ++i ) {
      const auto joint = state_names[i].substr( 0, state_names[i].find( '/' ) );
      state_ifaces_.push_back( hw_state_values_.make_state( i, joint, "position" ) );
      controller_->state_interfaces_.emplace_back( state_ifaces_.back() );
    }
  }

  void activate()
  {
    ASSERT_EQ( controller_->on_activate( rclcpp_lifecycle::State() ),
               controller_interface::CallbackReturn::SUCCESS );
  }

  void deactivate()
  {
    ASSERT_EQ( controller_->on_deactivate( rclcpp_lifecycle::State() ),
               controller_interface::CallbackReturn::SUCCESS );
  }

  controller_interface::return_type update()
  {
    return controller_->update_and_write_commands( rclcpp::Time( 0, 0, RCL_ROS_TIME ),
                                                   rclcpp::Duration( 0, 10000000 ) );
  }
};

TEST_F( SelfCollisionAvoidanceControllerTest, ForwardsAReference )
{
  configureController();
  activate();
  hector_test::reference( *controller_, 0 ) = 0.4;
  ASSERT_EQ( update(), controller_interface::return_type::OK );
  EXPECT_DOUBLE_EQ( hw_cmd_values_[0], 0.4 );
}

// A collision before the first valid reference of an activation blocks the joint. The block
// must not replay the previous activation's command (or 0.0 before any).
TEST_F( SelfCollisionAvoidanceControllerTest, ABlockAfterReactivationReplaysNoOldCommand )
{
  configureController();
  activate();
  ASSERT_TRUE( controller_->block_joint( 0 ) );
  EXPECT_TRUE( std::isnan( hw_cmd_values_[0] ) ) << "blocked to 0.0 before any command";

  hector_test::reference( *controller_, 0 ) = 0.4;
  ASSERT_EQ( update(), controller_interface::return_type::OK );
  ASSERT_DOUBLE_EQ( hw_cmd_values_[0], 0.4 );
  deactivate();

  hw_cmd_values_[0] = 0.1;
  activate();
  ASSERT_TRUE( controller_->block_joint( 0 ) );
  EXPECT_TRUE( std::isnan( hw_cmd_values_[0] ) ) << "replayed the previous activation's command";
}

TEST_F( SelfCollisionAvoidanceControllerTest, ChainedModeSwitchInvalidatesReferences )
{
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

// Not a command to this controller: no target, so a running velocity does not keep running
TEST_F( SelfCollisionAvoidanceControllerTest, AWrongSizeCommandHolds )
{
  configureController();
  activate();
  hector_test::reference( *controller_, 0 ) = 0.2;

  auto msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
  msg->data = { 1.0 };
  controller_->rt_buffer_ptr_.writeFromNonRT( msg );
  EXPECT_EQ( controller_->update_reference_from_subscribers( rclcpp::Time( 0, 0, RCL_ROS_TIME ),
                                                             rclcpp::Duration( 0, 10000000 ) ),
             controller_interface::return_type::OK );
  EXPECT_TRUE( std::isnan( hector_test::reference( *controller_, 0 ) ) )
      << "kept the previous command";
}

int main( int argc, char **argv )
{
  testing::InitGoogleMock( &argc, argv );
  rclcpp::init( argc, argv );
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
