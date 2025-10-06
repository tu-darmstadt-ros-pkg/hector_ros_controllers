//
// Created by aljoscha-schmidt on 7/17/25.
//
#include "apply_current_limit_controller/apply_current_limit_controller.hpp"
#include <controller_interface/helpers.hpp>
#include <limits>

namespace apply_current_limit_controller
{

ApplyCurrentLimitController::ApplyCurrentLimitController()
    : controller_interface::ChainableControllerInterface(), compliance_enabled_( false ),
      chained_mode_( false )
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

bool ApplyCurrentLimitController::on_set_chained_mode( bool chained_mode )
{
  // Remember whether we are a "following" controller in a chain
  chained_mode_ = chained_mode;
  return true;
}

controller_interface::CallbackReturn ApplyCurrentLimitController::process_params()
{
  // clear all in case of re-init / parameter change
  command_interface_names_.clear();
  exported_state_interface_names_.clear();
  reference_interface_names_.clear();
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
  command_interface_types_ = { "position", "current" };
  state_interface_types_ = { "position", "velocity" };

  command_interface_names_.reserve( command_interface_types_.size() * joints_.size() );
  exported_state_interface_names_.reserve( state_interface_types_.size() * joints_.size() );

  for ( const auto &jn : joints_ ) {
    for ( const auto &t : command_interface_types_ )
      command_interface_names_.push_back( jn + "/" + t );

    for ( const auto &t : state_interface_types_ )
      exported_state_interface_names_.push_back( jn + "/" + t );

    // throws if missing – that’s good: fail-fast on misconfig
    const auto &jl = params_.current_limits.joints_map.at( jn );
    compliant_limits_.push_back( jl.compliant_limit );
    stiff_limits_.push_back( jl.stiff_limit );
  }

  // Reference interfaces mirror the command interfaces (same ordering)
  reference_interface_names_ = command_interface_names_;

  // Backing storage
  reference_interfaces_.assign( reference_interface_names_.size(),
                                std::numeric_limits<double>::quiet_NaN() );
  state_interfaces_values_.assign( exported_state_interface_names_.size(),
                                   std::numeric_limits<double>::quiet_NaN() );

  // Sanity
  if ( reference_interface_names_.size() != command_interface_names_.size() ) {
    RCLCPP_ERROR( get_node()->get_logger(),
                  "reference_interface_names_ size (%zu) != command_interface_names_ size (%zu)",
                  reference_interface_names_.size(), command_interface_names_.size() );
    return controller_interface::CallbackReturn::ERROR;
  }
  if ( state_interfaces_values_.size() != exported_state_interface_names_.size() ) {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "state_interfaces_values_ size (%zu) != exported_state_interface_names_ size (%zu)",
        state_interfaces_values_.size(), exported_state_interface_names_.size() );
    return controller_interface::CallbackReturn::ERROR;
  }

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
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // If we're following in a chain, don't claim HW state interfaces
  cfg.names = chained_mode_ ? std::vector<std::string>{} : exported_state_interface_names_;
  return cfg;
}

controller_interface::return_type ApplyCurrentLimitController::update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/ )
{
  // In chained mode the previous controller will feed our reference interfaces directly.
  // We still accept subscriber updates if present, but typically none will be published in that mode.
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
}

std::vector<hardware_interface::CommandInterface>
ApplyCurrentLimitController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> refs;
  refs.reserve( reference_interface_names_.size() );
  for ( size_t i = 0; i < reference_interface_names_.size(); ++i ) {
    refs.emplace_back( get_node()->get_name(), reference_interface_names_[i],
                       &reference_interfaces_[i] );
  }
  return refs;
}

std::vector<hardware_interface::StateInterface>
ApplyCurrentLimitController::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> states;
  states.reserve( exported_state_interface_names_.size() );
  for ( size_t i = 0; i < exported_state_interface_names_.size(); ++i ) {
    states.emplace_back( get_node()->get_name(), exported_state_interface_names_[i],
                         &state_interfaces_values_[i] );
  }
  return states;
}

controller_interface::CallbackReturn
ApplyCurrentLimitController::on_configure( const rclcpp_lifecycle::State & /*previous_state*/ )
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
ApplyCurrentLimitController::on_activate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
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
    const bool ok = controller_interface::get_ordered_interfaces( state_interfaces_, joints_, type,
                                                                  ordered_state );
    if ( !ok || ordered_state.size() != joints_.size() ) {
      RCLCPP_ERROR( this->get_node()->get_logger(),
                    "Expected %zu state interfaces for type '%s', got %zu", joints_.size(),
                    type.c_str(), ordered_state.size() );
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
                                                        const rclcpp::Duration & /*period*/ )
{
  // If we are following another controller, we do not write to HW.
  if ( chained_mode_ )
    return controller_interface::return_type::OK;

  // Get ordered views every cycle to avoid relying on any internal ordering.
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
    return controller_interface::return_type::ERROR;

  bool ok = true;
  for ( size_t i = 0; i < joints_.size(); ++i ) {
    if ( std::isnan( reference_interfaces_[i] ) )
      continue;

    ok &= pos_cmd[i].get().set_value( reference_interfaces_[i] );
    ok &= cur_cmd[i].get().set_value( limits[i] );
  }

  return ok ? controller_interface::return_type::OK : controller_interface::return_type::ERROR;
}

} // namespace apply_current_limit_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS( apply_current_limit_controller::ApplyCurrentLimitController,
                        controller_interface::ChainableControllerInterface )
