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

  // Define types we want

  command_interface_names_.reserve( joints_.size() );

  for ( const auto &jn : joints_ ) {
    command_interface_names_.push_back( jn + "/current" );

    const auto &jl = params_.current_limits.joints_map.at( jn );
    compliant_limits_.push_back( jl.compliant_limit );
    stiff_limits_.push_back( jl.stiff_limit );
  }

  command_interface_names_.shrink_to_fit();

  std::string cmd_interfaces = "";
  for ( auto const &entry : command_interface_names_ ) { cmd_interfaces += "|" + entry; }
  RCLCPP_INFO( get_node()->get_logger(), "Claim cmd interfaces : %s", cmd_interfaces.c_str() );

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

/*controller_interface::return_type CurrentLimitController::update_reference_from_subscribers(
    const rclcpp::Time & /*time*///, const rclcpp::Duration & /*period*/ )
/**{

  auto joint_commands = rt_buffer_ptr_.readFromRT();
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
}*/

/*std::vector<hardware_interface::StateInterface>
CurrentLimitController::on_export_state_interfaces()
{
  RCLCPP_INFO( this->get_node()->get_logger(), "State interfaces" );
  std::vector<hardware_interface::StateInterface> states;
  states.reserve( exported_state_interface_names_.size() );
  for ( size_t i = 0; i < exported_state_interface_names_.size(); ++i ) {
    states.emplace_back( get_node()->get_name(), exported_state_interface_names_[i],
                         &state_interfaces_values_[i] );
  }
  return states;
}*/

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
  RCLCPP_INFO( this->get_node()->get_logger(), "Start activation" );
  /*try {
    // Validate we have exactly one interface per (joint,type); use ordered views for checks
    for ( const std::string &type : command_interface_types_ ) {
      std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_cmd;
      const bool ok = controller_interface::get_ordered_interfaces( command_interfaces_, joints_,
                                                                    type, ordered_cmd );
      if ( !ok || ordered_cmd.size() != joints_.size() ) {
        RCLCPP_ERROR( this->get_node()->get_logger(),
                      "Expected %zu command interfaces for type '%s', got %zu", joints_.size(),
                      type.c_str(), ordered_cmd.size() );
        return controller_interface::CallbackReturn::ERROR;
      }
    }

    for ( const std::string &type : state_interface_types_ ) {
      std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> ordered_state;
      const bool ok = controller_interface::get_ordered_interfaces( state_interfaces_, joints_,
                                                                    type, ordered_state );
      if ( !ok || ordered_state.size() != joints_.size() ) {
        RCLCPP_ERROR( this->get_node()->get_logger(),
                      "Expected %zu state interfaces for type '%s', got %zu", joints_.size(),
                      type.c_str(), ordered_state.size() );
        return controller_interface::CallbackReturn::ERROR;
      }
    }
    RCLCPP_INFO( this->get_node()->get_logger(), "Sanity checks completed" );
  }

  catch ( const std::exception &e ) {
    std::cerr << e.what() << '\n';
  }
  */

  // reset command buffer if a command came through callback when controller was inactive
  // rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>>( nullptr );
  // std::fill( reference_interfaces_.begin(), reference_interfaces_.end(),
  //           std::numeric_limits<double>::quiet_NaN() );

  RCLCPP_INFO( this->get_node()->get_logger(), "activate successful" );
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CurrentLimitController::on_deactivate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  // reset command buffer
  // rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>>( nullptr );
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
CurrentLimitController::update( const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/ )
{

  RCLCPP_INFO( this->get_node()->get_logger(), "Run update cycle" );

  /*// Get ordered views every cycle to avoid relying on any internal ordering.
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> pos_cmd;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> cur_cmd;
  if ( !controller_interface::get_ordered_interfaces( command_interfaces_, joints_, "position",
                                                      pos_cmd ) ||
       pos_cmd.size() != joints_.size() )
    return controller_interface::return_type::ERROR;
  if ( !controller_interface::get_ordered_interfaces( command_interfaces_, joints_, "current",
                                                      cur_cmd ) ||
       cur_cmd.size() != joints_.size() )
    return controller_interface::return_type::ERROR;

  const auto &limits = compliance_enabled_ ? compliant_limits_ : stiff_limits_;
  if ( limits.size() != joints_.size() )
    return controller_interface::return_type::ERROR;*/

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
