#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#define private public
#define protected public
#include <claim_only_controller/claim_only_controller.hpp>
#undef protected
#undef private

#include <controller_interface/controller_interface_params.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include "handle_values.hpp"

class ClaimOnlyControllerTest : public ::testing::Test
{
protected:
  controller_interface::return_type init( const std::vector<std::string> &command_interfaces )
  {
    controller_ = std::make_shared<claim_only_controller::ClaimOnlyController>();
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "test_claim_only";
    params.robot_description = "";
    params.update_rate = 100;
    params.controller_manager_update_rate = 100;
    params.node_namespace = "";
    rclcpp::NodeOptions options;
    options.parameter_overrides( { rclcpp::Parameter( "command_interfaces", command_interfaces ) } );
    params.node_options = options;
    return controller_->init( params );
  }

  std::shared_ptr<claim_only_controller::ClaimOnlyController> controller_;
};

TEST_F( ClaimOnlyControllerTest, ClaimsExactlyTheConfiguredInterfaces )
{
  ASSERT_EQ( init( { "arm_joint_1/torque_off", "arm_joint_2/torque_off" } ),
             controller_interface::return_type::OK );
  ASSERT_EQ( controller_->on_configure( rclcpp_lifecycle::State() ),
             controller_interface::CallbackReturn::SUCCESS );

  const auto commands = controller_->command_interface_configuration();
  EXPECT_EQ( commands.type, controller_interface::interface_configuration_type::INDIVIDUAL );
  EXPECT_THAT( commands.names,
               ::testing::ElementsAre( "arm_joint_1/torque_off", "arm_joint_2/torque_off" ) );
  EXPECT_EQ( controller_->state_interface_configuration().type,
             controller_interface::interface_configuration_type::NONE );
}

TEST_F( ClaimOnlyControllerTest, WritesNothing )
{
  ASSERT_EQ( init( { "arm_joint_1/torque_off" } ), controller_interface::return_type::OK );
  ASSERT_EQ( controller_->on_configure( rclcpp_lifecycle::State() ),
             controller_interface::CallbackReturn::SUCCESS );
  hector_test::HandleValues values;
  values.assign( 1, std::numeric_limits<double>::quiet_NaN() );
  auto handle = values.make_command( 0, "arm_joint_1", "torque_off" );
  controller_->command_interfaces_.emplace_back( handle, []() { } );
  ASSERT_EQ( controller_->on_activate( rclcpp_lifecycle::State() ),
             controller_interface::CallbackReturn::SUCCESS );

  for ( int i = 0; i < 10; ++i ) {
    ASSERT_EQ( controller_->update( rclcpp::Time( 0, 0, RCL_ROS_TIME ),
                                    rclcpp::Duration::from_seconds( 0.01 ) ),
               controller_interface::return_type::OK );
  }
  EXPECT_TRUE( std::isnan( values[0] ) ) << "the claim is the command, no value may be written";
}

TEST_F( ClaimOnlyControllerTest, AnEmptyListIsRefused )
{ EXPECT_NE( init( {} ), controller_interface::return_type::OK ); }

int main( int argc, char **argv )
{
  testing::InitGoogleMock( &argc, argv );
  rclcpp::init( argc, argv );
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
