#pragma once

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rtest/publisher_mock.hpp>
#include <rtest/service_mock.hpp>
#include <rtest/subscription_mock.hpp>

// Access private/protected members for testing
#define private public
#define protected public
#include <controller_interface/chainable_controller_interface.hpp>
#include <safety_forward_controller/safety_forward_controller.hpp>
#include <safety_position_controller/safety_position_controller.hpp>
#include <velocity_to_position_command_controller/velocity_to_position_command_controller.hpp>
#undef protected
#undef private

#include <controller_interface/controller_interface_params.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rtest/static_registry.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace hector_test
{

// ---- URDF loading from installed test config directory ----

inline std::string loadUrdfFile( const std::string &filename )
{
  const std::string path =
      ament_index_cpp::get_package_share_directory( "hector_ros_controllers" ) + "/test/config/" +
      filename;
  std::ifstream ifs( path );
  if ( !ifs.is_open() ) {
    throw std::runtime_error( "Cannot open test URDF file: " + path );
  }
  return std::string( std::istreambuf_iterator<char>( ifs ), std::istreambuf_iterator<char>() );
}

// ---- CRTP base fixture for safety controller tests ----
//
// Derived must provide (as public members):
//   using ControllerType = ...;
//   using StatusMsgType  = ...;
//   std::shared_ptr<ControllerType> controller_;
//   std::shared_ptr<rtest::PublisherMock<StatusMsgType>> status_pub_mock_;
//   void resetControllerSpecificMocks();
//   void findControllerSpecificMocks( const std::string &node_name );

template<typename Derived>
class SafetyControllerTestBase : public ::testing::Test
{
protected:
  static constexpr unsigned int kUpdateRate = 100;

  // Hardware interface backing storage
  std::vector<double> hw_cmd_values_;
  std::vector<double> hw_state_values_;
  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> cmd_ifaces_;
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> state_ifaces_;

  void TearDown() override
  {
    auto &d = derived();
    d.status_pub_mock_.reset();
    d.resetControllerSpecificMocks();
    d.controller_.reset();
    rtest::StaticMocksRegistry::instance().reset();
  }

  void activateController()
  {
    rclcpp_lifecycle::State inactive( lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                      "inactive" );
    auto cb = derived().controller_->on_activate( inactive );
    ASSERT_EQ( cb, controller_interface::CallbackReturn::SUCCESS );
  }

  void deactivateController()
  {
    rclcpp_lifecycle::State active( lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active" );
    auto cb = derived().controller_->on_deactivate( active );
    ASSERT_EQ( cb, controller_interface::CallbackReturn::SUCCESS );
  }

  controller_interface::return_type callUpdate()
  {
    rclcpp::Time now( 0, 0, RCL_ROS_TIME );
    rclcpp::Duration period( std::chrono::milliseconds( 10 ) );
    return derived().controller_->update_and_write_commands( now, period );
  }

  void findMocks()
  {
    auto node_name = std::string( derived().controller_->get_node()->get_fully_qualified_name() );
    auto status_topic = node_name + "/status";
    derived().status_pub_mock_ =
        rtest::findPublisher<typename Derived::StatusMsgType>( node_name, status_topic );
    ASSERT_TRUE( derived().status_pub_mock_ ) << "Failed to find status publisher mock for node '"
                                              << node_name << "' topic '" << status_topic << "'";
    derived().findControllerSpecificMocks( node_name );
  }

  void sendEstop( bool active ) { derived().controller_->estop_active_.store( active ); }

private:
  Derived &derived() { return static_cast<Derived &>( *this ); }
  const Derived &derived() const { return static_cast<const Derived &>( *this ); }
};

} // namespace hector_test
