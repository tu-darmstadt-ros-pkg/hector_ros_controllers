#include "current_limit_controller/current_limit_controller.hpp"
#include <controller_interface/helpers.hpp>
#include <limits>

namespace current_limit_controller
{

CurrentLimitController::CurrentLimitController()
    : controller_interface::ControllerInterface(), compliance_enabled_( false )
{
}

controller_interface::CallbackReturn CurrentLimitController::on_init()
{
  param_listener_ = std::make_shared<ParamListener>( get_node() );
  params_ = param_listener_->get_params();

  return read_parameters();
}

controller_interface::CallbackReturn CurrentLimitController::read_parameters()
{
  // clear all in case of re-init / parameter change
  command_interface_names_.clear();
  compliant_limits_.clear();
  stiff_limits_.clear();

  if ( !param_listener_ ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Error encountered during init" );
    return controller_interface::CallbackReturn::ERROR;
  }
  if ( params_.joints.empty() ) {
    RCLCPP_ERROR( get_node()->get_logger(), "'joints' parameter was empty" );
    return controller_interface::CallbackReturn::ERROR;
  }

  joints_ = params_.joints;

  command_interface_names_.reserve( joints_.size() );

  for ( const auto &jn : joints_ ) {
    command_interface_names_.push_back( jn + "/current" );

    const auto &jl = params_.current_limits.joints_map.at( jn );
    compliant_limits_.push_back( jl.compliant_limit );
    stiff_limits_.push_back( jl.stiff_limit );
  }

  command_interface_names_.shrink_to_fit();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CurrentLimitController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_names_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration CurrentLimitController::state_interface_configuration() const
{
  // Empty
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  return state_interfaces_config;
}

controller_interface::CallbackReturn
CurrentLimitController::on_configure( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  enable_compliant_limits_srv_ = get_node()->create_service<std_srvs::srv::SetBool>(
      "enable_compliance", [this]( std_srvs::srv::SetBool::Request::SharedPtr req,
                                   std_srvs::srv::SetBool::Response::SharedPtr resp ) {
        compliance_enabled_ = req->data;
        resp->success = true;
        resp->message = compliance_enabled_ ? "Compliance enabled" : "Compliance disabled";
      } );

  RCLCPP_INFO( this->get_node()->get_logger(), "configure successful" );
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CurrentLimitController::on_activate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  RCLCPP_INFO( this->get_node()->get_logger(), "activate successful" );
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CurrentLimitController::on_deactivate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
CurrentLimitController::update( const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/ )
{
  const std::vector<double> &limits_to_apply =
      compliance_enabled_ ? compliant_limits_ : stiff_limits_;

  bool success = true;
  for ( size_t i = 0; i < joints_.size(); ++i ) {
    success = success && command_interfaces_[i].set_value( limits_to_apply[i] );
  }

  return success ? controller_interface::return_type::OK : controller_interface::return_type::ERROR;
}

} // namespace current_limit_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS( current_limit_controller::CurrentLimitController,
                        controller_interface::ControllerInterface )
