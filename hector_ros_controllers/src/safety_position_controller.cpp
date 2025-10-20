#include "safety_position_controller/safety_position_controller.hpp"

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <urdf_parser/urdf_parser.h>

namespace safety_position_controller
{

SafetyPositionController::SafetyPositionController()
    : controller_interface::ChainableControllerInterface()
{
}

bool SafetyPositionController::on_set_chained_mode( const bool chained_mode )
{
  is_chained_ = chained_mode;
  return true;
}

controller_interface::CallbackReturn SafetyPositionController::on_init()
{
  const auto node = get_node();
  if ( !node ) {
    RCLCPP_ERROR( rclcpp::get_logger( "SafetyPositionController" ), "No node in on_init()" );
    return controller_interface::CallbackReturn::ERROR;
  }

  try {
    param_listener_ = std::make_shared<ParamListener>( get_node() );
    params_ = param_listener_->get_params();

  } catch ( const std::exception &e ) {
    RCLCPP_WARN( get_node()->get_logger(), "Exception thrown during init stage with message: %s \n",
                 e.what() );
    return controller_interface::CallbackReturn::ERROR;
  }

  if ( params_.set_current_limits ) {
    enforce_current_limits_service_ = node->create_service<std_srvs::srv::SetBool>(
        "~/enforce_current_limits",
        [this]( const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                std::shared_ptr<std_srvs::srv::SetBool::Response> response ) {
          in_compliant_mode_ = request->data;
          response->success = true;
          response->message = std::string( "Set enforce_current_limits to " ) +
                              ( in_compliant_mode_ ? "true" : "false" );
          RCLCPP_INFO( get_node()->get_logger(), "%s", response->message.c_str() );
        } );
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SafetyPositionController::on_configure( const rclcpp_lifecycle::State & )
{
  const auto node = get_node();

  if ( params_.joints.empty() ) {
    RCLCPP_ERROR( node->get_logger(), "'joints' parameter must not be empty." );
    return controller_interface::CallbackReturn::ERROR;
  }

  if ( !parse_urdf_and_fill_joint_info( this->get_robot_description() ) ) {
    RCLCPP_ERROR( node->get_logger(), "Failed to parse URDF / joint limits." );
    return controller_interface::CallbackReturn::ERROR;
  }

  const size_t n = params_.joints.size();
  reference_interfaces_.assign( n, std::numeric_limits<double>::quiet_NaN() );
  state_interface_index_.assign( n, -1 );

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SafetyPositionController::on_activate( const rclcpp_lifecycle::State & )
{
  if ( !is_chained_ ) {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "SafetyPositionController is CHAINED-ONLY. Enable chained_mode for this controller." );
    return controller_interface::CallbackReturn::SUCCESS;
  }

  const auto joints = params_.joints;
  if ( param_listener_->try_update_params( params_ ) ) {
    // make sure joints didn't change
    if ( joints != params_.joints ) {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "Joints parameter changed during runtime reconfiguration. This is not supported." );
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  if ( !gather_interface_indices() ) {
    RCLCPP_ERROR( get_node()->get_logger(),
                  "Failed to gather state interface indices for joints." );
    return controller_interface::CallbackReturn::ERROR;
  }

  // check order of command interfaces
  // TODO: if this fails use command interface reordering function or indexing as for state interfaces
  for ( size_t i = 0; i < params_.joints.size(); ++i ) {
    if ( command_interfaces_[i].get_name() != params_.joints[i] + "/position" ) {
      RCLCPP_ERROR( get_node()->get_logger(), "Command interfaces are not in the expected order." );
      return controller_interface::CallbackReturn::ERROR;
    }
    if ( params_.set_current_limits && command_interfaces_[i + params_.joints.size()].get_name() !=
                                           params_.joints[i] + "/current" ) {
      RCLCPP_ERROR( get_node()->get_logger(),
                    "Current limit command interfaces are not in the expected order." );
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SafetyPositionController::on_deactivate( const rclcpp_lifecycle::State & )
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
SafetyPositionController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for ( const auto &j : params_.joints ) conf.names.emplace_back( j + "/position" );
  if ( params_.set_current_limits ) {
    for ( const auto &j : params_.joints ) conf.names.emplace_back( j + "/current" );
  }
  return conf;
}

controller_interface::InterfaceConfiguration
SafetyPositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for ( const auto &j : params_.joints ) conf.names.emplace_back( j + "/position" );
  return conf;
}

std::vector<hardware_interface::CommandInterface>
SafetyPositionController::on_export_reference_interfaces()
{
  const size_t n = params_.joints.size();
  std::vector<hardware_interface::CommandInterface> refs;
  refs.reserve( n );

  for ( size_t i = 0; i < n; ++i ) {
    refs.emplace_back( get_node()->get_name(), params_.joints[i] + "/position",
                       &reference_interfaces_[i] );
  }
  return refs;
}

controller_interface::return_type
SafetyPositionController::update_reference_from_subscribers( const rclcpp::Time &,
                                                             const rclcpp::Duration & )
{
  // Chained-only; no direct subscribers
  return controller_interface::return_type::OK;
}

controller_interface::return_type
SafetyPositionController::update_and_write_commands( const rclcpp::Time &, const rclcpp::Duration & )
{
  if ( !is_chained_ ) {
    RCLCPP_ERROR_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 2000,
                           "SafetyPositionController is CHAINED-ONLY." );
  }

  bool success = true;
  const size_t n = params_.joints.size();

  for ( size_t i = 0; i < n; ++i ) {
    double current;
    const auto &opt =
        state_interfaces_[static_cast<size_t>( state_interface_index_[i] )].get_optional();
    if ( opt.has_value() ) {
      current = opt.value();
    } else {
      RCLCPP_ERROR_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 2000,
                             "Cannot get joint state for joint '%s'", params_.joints[i].c_str() );
      success = false;
      continue;
    }

    const double target_wrapped = reference_interfaces_[i];
    if ( std::isnan( target_wrapped ) )
      continue;

    double commanded = target_wrapped;
    switch ( kinds_[i] ) {
    case JointType::CONTINUOUS:
      if ( params_.unwrap_continuous_joints )
        commanded = unwrap_to_nearest( current, target_wrapped );
      break;
    case JointType::REVOLUTE_BOUNDED:
      commanded = unwrap_to_nearest( current, target_wrapped );
      if ( params_.enforce_position_limits )
        commanded = clamp( i, commanded );
      break;
    case JointType::PRISMATIC_BOUNDED:
      if ( params_.enforce_position_limits )
        commanded = clamp( i, commanded );
      break;
    case JointType::FIXED:
    case JointType::OTHER:
    default:
      if ( params_.enforce_position_limits && has_limits_[i] )
        commanded = clamp( i, commanded );
      break;
    }

    success &= command_interfaces_[i].set_value( commanded );

    // set current limit if enabled and command interfaces are requested
    if ( params_.set_current_limits && command_interfaces_.size() > params_.joints.size() ) {

      const auto &limit = in_compliant_mode_
                              ? params_.current_limits.joints_map[params_.joints[i]].compliant_limit
                              : params_.current_limits.joints_map[params_.joints[i]].stiff_limit;
      success &= command_interfaces_[i + params_.joints.size()].set_value( limit );
    }
  }

  return success ? controller_interface::return_type::OK : controller_interface::return_type::ERROR;
}

// ===== Helpers =====

double SafetyPositionController::unwrap_to_nearest( const double current, const double target )
{
  const double k = std::round( ( current - target ) / ( 2.0 * M_PI ) );
  return target + k * ( 2.0 * M_PI );
}

double SafetyPositionController::clamp( size_t i, double value ) const
{
  if ( !has_limits_[i] )
    return value;

  const double lo = lower_limits_[i];
  const double hi = upper_limits_[i];
  if ( value < lo ) {
    RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "Clamping joint '%s' to lower limit %.3f", params_.joints[i].c_str(), lo );
    return lo;
  }
  if ( value > hi ) {
    RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "Clamping joint '%s' to upper limit %.3f", params_.joints[i].c_str(), hi );
    return hi;
  }
  return value;
}

bool SafetyPositionController::parse_urdf_and_fill_joint_info( const std::string &urdf_xml )
{
  const auto model = urdf::parseURDF( urdf_xml );
  if ( !model )
    return false;

  const size_t n = params_.joints.size();
  kinds_.assign( n, JointType::OTHER );
  has_limits_.assign( n, false );
  lower_limits_.assign( n, 0.0 );
  upper_limits_.assign( n, 0.0 );

  for ( size_t i = 0; i < n; ++i ) {
    const auto jn = params_.joints[i];
    auto urdf_joint = model->getJoint( jn );
    if ( !urdf_joint )
      continue;

    switch ( urdf_joint->type ) {
    case urdf::Joint::CONTINUOUS:
      kinds_[i] = JointType::CONTINUOUS;
      break;
    case urdf::Joint::REVOLUTE:
      kinds_[i] = JointType::REVOLUTE_BOUNDED;
      if ( urdf_joint->limits ) {
        has_limits_[i] = true;
        lower_limits_[i] = urdf_joint->limits->lower;
        upper_limits_[i] = urdf_joint->limits->upper;
      }
      break;
    case urdf::Joint::PRISMATIC:
      kinds_[i] = JointType::PRISMATIC_BOUNDED;
      if ( urdf_joint->limits ) {
        has_limits_[i] = true;
        lower_limits_[i] = urdf_joint->limits->lower;
        upper_limits_[i] = urdf_joint->limits->upper;
      }
      break;
    case urdf::Joint::FIXED:
      kinds_[i] = JointType::FIXED;
      break;
    default:
      kinds_[i] = JointType::OTHER;
      break;
    }

    if ( has_limits_[i] && !( lower_limits_[i] < upper_limits_[i] ) ) {
      RCLCPP_WARN( get_node()->get_logger(), "Joint '%s' has invalid limits [%.3f, %.3f]",
                   jn.c_str(), lower_limits_[i], upper_limits_[i] );
      has_limits_[i] = false;
    }
  }

  return true;
}

bool SafetyPositionController::gather_interface_indices()
{
  bool success = true;
  for ( size_t i = 0; i < params_.joints.size(); ++i ) {
    state_interface_index_[i] = -1;
    for ( size_t s = 0; s < state_interfaces_.size(); ++s ) {
      auto name = state_interfaces_[s].get_name();
      auto interface_name = state_interfaces_[s].get_interface_name();
      if ( state_interfaces_[s].get_name() == params_.joints[i] + "/position" &&
           state_interfaces_[s].get_interface_name() == "position" ) {
        state_interface_index_[i] = static_cast<int>( s );
        break;
      }
    }
    if ( state_interface_index_[i] < 0 )
      RCLCPP_WARN( get_node()->get_logger(), "No state interface 'position' found for joint '%s'.",
                   params_.joints[i].c_str() );
    success &= ( state_interface_index_[i] >= 0 );
  }
  return success;
}

} // namespace safety_position_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS( safety_position_controller::SafetyPositionController,
                        controller_interface::ChainableControllerInterface )
