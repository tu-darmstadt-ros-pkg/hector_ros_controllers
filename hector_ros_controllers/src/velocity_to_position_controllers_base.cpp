#include "velocity_to_position_command_controller/velocity_to_position_controllers_base.hpp"

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace velocity_to_position_command_controller
{
VelocityToPositionControllersBase::VelocityToPositionControllersBase()
    : controller_interface::ControllerInterface(), rt_command_ptr_( nullptr ),
      joints_command_subscriber_( nullptr )
{
}

controller_interface::CallbackReturn VelocityToPositionControllersBase::on_init()
{
  try {
    declare_parameters();
  } catch ( const std::exception &e ) {
    fprintf( stderr, "Exception thrown during init stage with message: %s \n", e.what() );
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
VelocityToPositionControllersBase::on_configure( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  auto ret = this->read_parameters();
  if ( ret != controller_interface::CallbackReturn::SUCCESS ) {
    return ret;
  }

  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
      "~/commands", rclcpp::SystemDefaultsQoS(),
      [this]( const CmdType::SharedPtr msg ) { rt_command_ptr_.writeFromNonRT( msg ); } );

  RCLCPP_INFO( get_node()->get_logger(), "configure successful" );

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
VelocityToPositionControllersBase::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_types_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
VelocityToPositionControllersBase::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interface_config;
  state_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interface_config.names = state_interface_types_;

  return state_interface_config;
}

controller_interface::CallbackReturn
VelocityToPositionControllersBase::on_activate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_interfaces;
  if ( !controller_interface::get_ordered_interfaces( command_interfaces_, command_interface_types_,
                                                      std::string( "" ), ordered_interfaces ) ||
       command_interface_types_.size() != ordered_interfaces.size() ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
                  command_interface_types_.size(), ordered_interfaces.size() );
    return controller_interface::CallbackReturn::ERROR;
  }

  // reset command buffer if a command came through callback when controller was inactive
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>( nullptr );

  RCLCPP_INFO( get_node()->get_logger(), "activate successful" );
  for ( auto index = 0ul; index < command_interfaces_.size(); ++index ) {
    last_positions_[index] = state_interfaces_[index].get_value();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
VelocityToPositionControllersBase::on_deactivate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  // reset command buffer
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>( nullptr );
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
VelocityToPositionControllersBase::update( const rclcpp::Time & /*time*/,
                                           const rclcpp::Duration &p /*period*/ )
{
  auto joint_vel_commands = rt_command_ptr_.readFromRT();
  // no command received yet
  if ( !joint_vel_commands || !( *joint_vel_commands ) ) {
    return controller_interface::return_type::OK;
  }

  bool successful = true;
  // Set commands for joints
  for ( auto index = 0ul; index < command_interfaces_.size(); ++index ) {
    double vel_command = ( *joint_vel_commands )->data[index];
    auto limits = joint_limits_[index];

    if ( limits ) {

      if ( limits->velocity )
        vel_command = std::clamp( vel_command, -limits->velocity, limits->velocity );

      double pos_command = last_positions_[index] + vel_command * p.seconds();
      if ( limits->upper )
        pos_command = std::clamp( pos_command, -DBL_MAX, limits->upper );
      if ( limits->lower )
        pos_command = std::clamp( pos_command, limits->lower, DBL_MAX );

      successful = command_interfaces_[index].set_value( pos_command );
      last_positions_[index] = pos_command;
    } else {
      double pos_command = last_positions_[index] + vel_command * p.seconds();
      successful = command_interfaces_[index].set_value( pos_command );
      last_positions_[index] = pos_command;
    }
  }

  if ( !successful )
    return controller_interface::return_type::ERROR;

  return controller_interface::return_type::OK;
}

} // namespace velocity_to_position_command_controller
