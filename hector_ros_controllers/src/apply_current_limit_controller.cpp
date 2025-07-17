//
// Created by aljoscha-schmidt on 7/17/25.
//
#include "apply_current_limit_controller/apply_current_limit_controller.hpp"

#include <controller_interface/helpers.hpp>

namespace apply_current_limit_controller
{
controller_interface::CallbackReturn ApplyCurrentLimitController::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>( get_node() );
    params_ = param_listener_->get_params();

  } catch ( const std::exception &e ) {
    fprintf( stderr, "Exception thrown during init stage with message: %s \n", e.what() );
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ApplyCurrentLimitController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for ( const std::string &joint_name : params_.arm_joints ) {
    command_interfaces_config.names.push_back( joint_name + "/position" );
    command_interfaces_config.names.push_back( joint_name + "/current" );
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
ApplyCurrentLimitController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for ( const std::string &joint_name : params_.arm_joints ) {
    state_interfaces_config.names.push_back( joint_name + "/position" );
    state_interfaces_config.names.push_back( joint_name + "/velocity" );
  }

  return state_interfaces_config;
}

controller_interface::return_type ApplyCurrentLimitController::update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/ )
{
  auto joint_commands = rt_buffer_ptr_.readFromRT();
  // message is valid
  if ( !( !joint_commands || !( *joint_commands ) ) ) {
    if ( reference_interfaces_.size() != ( *joint_commands )->data.size() ) {
      RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(), *( get_node()->get_clock() ), 1000,
          "command size (%zu) does not match number of reference interfaces (%zu)",
          ( *joint_commands )->data.size(), reference_interfaces_.size() );
      return controller_interface::return_type::ERROR;
    }
    reference_interfaces_ = ( *joint_commands )->data;
  }

  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface>
ApplyCurrentLimitController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;

  for ( size_t i = 0; i < reference_interface_names_.size(); ++i ) {
    reference_interfaces.push_back( hardware_interface::CommandInterface(
        get_node()->get_name(), reference_interface_names_[i], &reference_interfaces_[i] ) );
  }

  return reference_interfaces;
}

controller_interface::CallbackReturn
ApplyCurrentLimitController::on_configure( const rclcpp_lifecycle::State & /*previous_state*/ )
{

  RCLCPP_INFO( this->get_node()->get_logger(), "configure successful" );

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ApplyCurrentLimitController::on_activate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_interfaces;
  if ( !controller_interface::get_ordered_interfaces( command_interfaces_, params_.arm_joints,
                                                      std::string( "" ), ordered_interfaces ) ||
       params_.arm_joints.size() != ordered_interfaces.size() ) {
    RCLCPP_ERROR( this->get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
                  command_interface_names_.size(), ordered_interfaces.size() );
    return controller_interface::CallbackReturn::ERROR;
  }

  // reset command buffer if a command came through callback when controller was inactive
  rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>>( nullptr );

  RCLCPP_INFO( this->get_node()->get_logger(), "activate successful" );

  std::fill( reference_interfaces_.begin(), reference_interfaces_.end(),
             std::numeric_limits<double>::quiet_NaN() );

  return controller_interface::CallbackReturn::SUCCESS;
}

} // namespace apply_current_limit_controller