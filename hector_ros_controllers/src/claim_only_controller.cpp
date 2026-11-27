#include "claim_only_controller/claim_only_controller.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace claim_only_controller
{

controller_interface::CallbackReturn ClaimOnlyController::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>( get_node() );
  } catch ( const std::exception &e ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Could not declare the parameters: %s", e.what() );
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ClaimOnlyController::on_configure( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  // The parameter library refuses an empty list, which would make an active controller claim nothing
  params_ = param_listener_->get_params();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ClaimOnlyController::command_interface_configuration() const
{
  return { controller_interface::interface_configuration_type::INDIVIDUAL,
           params_.command_interfaces };
}

controller_interface::InterfaceConfiguration ClaimOnlyController::state_interface_configuration() const
{ return { controller_interface::interface_configuration_type::NONE, {} }; }

controller_interface::return_type ClaimOnlyController::update( const rclcpp::Time & /*time*/,
                                                               const rclcpp::Duration & /*period*/ )
{ return controller_interface::return_type::OK; }

} // namespace claim_only_controller

PLUGINLIB_EXPORT_CLASS( claim_only_controller::ClaimOnlyController,
                        controller_interface::ControllerInterface )
