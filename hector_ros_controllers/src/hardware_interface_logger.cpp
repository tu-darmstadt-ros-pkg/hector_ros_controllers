#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <controller_manager_msgs/srv/list_hardware_interfaces.hpp>
#include <memory>

using namespace std::chrono_literals;

class HardwareInterfaceLogger : public rclcpp::Node
{
public:
  HardwareInterfaceLogger() : Node( "hardware_interface_logger" )
  {
    client_ = this->create_client<controller_manager_msgs::srv::ListHardwareInterfaces>(
        "/controller_manager/list_hardware_interfaces" );

    timer_ =
        this->create_wall_timer( 100ms, std::bind( &HardwareInterfaceLogger::call_service, this ) );

    RCLCPP_INFO( this->get_logger(), "HardwareInterfaceLogger started, polling every 2s..." );
  }

private:
  void call_service()
  {
    if ( !client_->wait_for_service( 1s ) ) {
      RCLCPP_WARN( this->get_logger(), "Service not available yet..." );
      return;
    }

    auto request = std::make_shared<controller_manager_msgs::srv::ListHardwareInterfaces::Request>();

    auto future = client_->async_send_request(
        request,
        [this]( rclcpp::Client<controller_manager_msgs::srv::ListHardwareInterfaces>::SharedFuture
                    future_resp ) {
          try {
            auto resp = future_resp.get();
            RCLCPP_INFO( this->get_logger(), "---- Hardware Interfaces ----" );

            RCLCPP_INFO( this->get_logger(),
                         "Command interfaces (%zu):", resp->command_interfaces.size() );
            for ( const auto &iface : resp->command_interfaces ) {
              RCLCPP_INFO( this->get_logger(), "  - %s [available=%s, claimed=%s]",
                           iface.name.c_str(), iface.is_available ? "true" : "false",
                           iface.is_claimed ? "true" : "false" );
            }

            RCLCPP_INFO( this->get_logger(),
                         "State interfaces (%zu):", resp->state_interfaces.size() );
            for ( const auto &iface : resp->state_interfaces ) {
              RCLCPP_INFO( this->get_logger(), "  - %s [available=%s]", iface.name.c_str(),
                           iface.is_available ? "true" : "false" );
            }
            RCLCPP_INFO( this->get_logger(), "---------------------------------" );
          } catch ( const std::exception &e ) {
            RCLCPP_ERROR( this->get_logger(), "Exception in service callback: %s", e.what() );
          }
        } );
  }

  rclcpp::Client<controller_manager_msgs::srv::ListHardwareInterfaces>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<HardwareInterfaceLogger>() );
  rclcpp::shutdown();
  return 0;
}
