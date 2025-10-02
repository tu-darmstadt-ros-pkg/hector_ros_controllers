//
// Created by aljoscha-schmidt on 7/17/25.
//
#include "apply_current_limit_controller/apply_current_limit_controller.hpp"
#include <controller_interface/helpers.hpp>

namespace apply_current_limit_controller
{

ApplyCurrentLimitController::ApplyCurrentLimitController()
    : controller_interface::ChainableControllerInterface()
{
}

controller_interface::CallbackReturn ApplyCurrentLimitController::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>( get_node() );
    params_ = param_listener_->get_params();

  } catch ( const std::exception &e ) {
    fprintf( stderr, "Exception thrown during init stage with message: %s \n", e.what() );
    return controller_interface::CallbackReturn::ERROR;
  }

  return process_params();
}

bool ApplyCurrentLimitController::on_set_chained_mode( bool /*chained_mode*/ ) { return true; }

controller_interface::CallbackReturn ApplyCurrentLimitController::process_params()
{

  if ( !param_listener_ ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Error encountered during init" );
    return controller_interface::CallbackReturn::ERROR;
  }
  if ( params_.joints.empty() ) {
    RCLCPP_ERROR( get_node()->get_logger(), "'joints' parameter was empty" );
    return controller_interface::CallbackReturn::ERROR;
  }

  joints_ = params_.joints;
  command_interface_names_.reserve( 2 * joints_.size() );
  exported_state_interface_names_.reserve( 2 * joints_.size() );

  command_interface_types_ = { "position", "current" };
  state_interface_types_ = { "position", "velocity" };

  for ( size_t i = 0ul; i < joints_.size(); i++ ) {

    for ( size_t j = 0ul; j < command_interface_types_.size(); j++ )
      command_interface_names_.push_back( joints_[i] + "/" + command_interface_types_[i] );

    for ( size_t j = 0ul; j < command_interface_types_.size(); j++ )
      exported_state_interface_names_.push_back( joints_[i] + "/" + state_interface_types_[i] );

    compliant_limits_.push_back( params_.current_limits.joints_map.at( joints_[i] ).compliant_limit );
    stiff_limits_.push_back( params_.current_limits.joints_map.at( joints_[i] ).stiff_limit );
  }

  // The names should be in the same order as for command interfaces for easier matching
  for ( auto i = 0ul; i < command_interface_names_.size(); i++ ) {
    reference_interface_names_.push_back( command_interface_names_[i] );
  }
  //  for any case make reference interfaces size of command interfaces
  reference_interfaces_.resize( reference_interface_names_.size(),
                                std::numeric_limits<double>::quiet_NaN() );

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ApplyCurrentLimitController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_names_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
ApplyCurrentLimitController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = exported_state_interface_names_;

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

std::vector<hardware_interface::StateInterface>
ApplyCurrentLimitController::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> exported_state_interfaces;
  for ( size_t i = 0; i < exported_state_interface_names_.size(); ++i ) {

    exported_state_interfaces.push_back( hardware_interface::StateInterface(
        get_node()->get_name(), exported_state_interface_names_[i], &state_interfaces_values_[i] ) );
  }

  return exported_state_interfaces;
}

controller_interface::CallbackReturn
ApplyCurrentLimitController::on_configure( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  enable_compliant_limits_srv_ = get_node()->create_service<std_srvs::srv::SetBool>(
      "enable_compliance", [this]( std_srvs::srv::SetBool::Request::SharedPtr req,
                                   std_srvs::srv::SetBool::Response::SharedPtr resp ) {
        compliance_enabled_ = req->data;
      } );

  RCLCPP_INFO( this->get_node()->get_logger(), "configure successful" );

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ApplyCurrentLimitController::on_activate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter

  for ( std::string interface_type : command_interface_types_ ) {
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_interfaces;
    if ( !controller_interface::get_ordered_interfaces( command_interfaces_, joints_,
                                                        interface_type, ordered_interfaces ) ||
         joints_.size() != ordered_interfaces.size() ) {
      RCLCPP_ERROR( this->get_node()->get_logger(), "Expected %zu command interfaces for %s, got %zu",
                    command_interface_names_.size(), interface_type.c_str(),
                    ordered_interfaces.size() );
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  for ( std::string interface_type : state_interface_types_ ) {
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> ordered_interfaces;
    if ( !controller_interface::get_ordered_interfaces( state_interfaces_, joints_, interface_type,
                                                        ordered_interfaces ) ||
         joints_.size() != ordered_interfaces.size() ) {
      RCLCPP_ERROR( this->get_node()->get_logger(), "Expected %zu state interfaces for %s, got %zu",
                    command_interface_names_.size(), interface_type.c_str(),
                    ordered_interfaces.size() );
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // reset command buffer if a command came through callback when controller was inactive
  rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>>( nullptr );
  std::fill( reference_interfaces_.begin(), reference_interfaces_.end(),
             std::numeric_limits<double>::quiet_NaN() );

  RCLCPP_INFO( this->get_node()->get_logger(), "activate successful" );
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ApplyCurrentLimitController::on_deactivate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  // reset command buffer
  rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>>( nullptr );
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
ApplyCurrentLimitController::update_and_write_commands( const rclcpp::Time & /*time*/,
                                                        const rclcpp::Duration & )
{
  const auto &limits = compliance_enabled_ ? compliant_limits_ : stiff_limits_;

  bool success = true;
  for ( size_t i = 0; i < joints_.size(); i++ ) {
    if ( std::isnan( reference_interfaces_[i] ) )
      continue;

    success &= command_interfaces_[2 * i].set_value( reference_interfaces_[i] ) &&
               command_interfaces_[2 * i + 1].set_value( limits[i] );
  }

  if ( !success )
    return controller_interface::return_type::ERROR;

  return controller_interface::return_type::OK;
}

} // namespace apply_current_limit_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS( apply_current_limit_controller::ApplyCurrentLimitController,
                        controller_interface::ChainableControllerInterface )