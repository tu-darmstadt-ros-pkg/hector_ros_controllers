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

bool SafetyPositionController::on_set_chained_mode( bool chained_mode )
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

  node->declare_parameter<std::vector<std::string>>( "joints", std::vector<std::string>() );
  node->declare_parameter<std::string>( "robot_description", robot_description_param_ );
  node->declare_parameter<bool>( "unwrap_continuous_joints", unwrap_continuous_ );
  node->declare_parameter<bool>( "enforce_position_limits", enforce_limits_ );

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SafetyPositionController::on_configure( const rclcpp_lifecycle::State & )
{
  const auto node = get_node();

  joint_names_ = node->get_parameter( "joints" ).as_string_array();
  robot_description_param_ = node->get_parameter( "robot_description" ).as_string();
  unwrap_continuous_ = node->get_parameter( "unwrap_continuous_joints" ).as_bool();
  enforce_limits_ = node->get_parameter( "enforce_position_limits" ).as_bool();

  if ( joint_names_.empty() ) {
    RCLCPP_ERROR( node->get_logger(), "'joints' parameter must not be empty." );
    return controller_interface::CallbackReturn::ERROR;
  }

  // Load URDF from parameter server
  std::string urdf_xml = this->get_robot_description();
  if ( urdf_xml.empty() ) {
    RCLCPP_ERROR(
        node->get_logger(),
        "URDF is empty. Ensure parameter '%s' (or 'robot_description') is set on this node.",
        robot_description_param_.c_str() );
    return controller_interface::CallbackReturn::ERROR;
  }

  if ( !parse_urdf_and_fill_joint_info( urdf_xml ) ) {
    RCLCPP_ERROR( node->get_logger(), "Failed to parse URDF / joint limits." );
    return controller_interface::CallbackReturn::ERROR;
  }

  const size_t n = joint_names_.size();
  last_unwrapped_cmd_.assign( n, 0.0 );
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

  gather_interface_indices();

  // Initialize last command from current states
  for ( size_t i = 0; i < joint_names_.size(); ++i ) {
    const int idx = state_interface_index_[i];
    if ( idx >= 0 ) {
      const auto &opt = state_interfaces_[static_cast<size_t>( idx )].get_optional();
      if ( opt.has_value() )
        last_unwrapped_cmd_[i] = opt.value();
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
  for ( const auto &j : joint_names_ ) conf.names.emplace_back( j + "/position" );
  return conf;
}

controller_interface::InterfaceConfiguration
SafetyPositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for ( const auto &j : joint_names_ ) conf.names.emplace_back( j + "/position" );
  return conf;
}

std::vector<hardware_interface::CommandInterface>
SafetyPositionController::on_export_reference_interfaces()
{
  const size_t n = joint_names_.size();
  std::vector<hardware_interface::CommandInterface> refs;
  refs.reserve( n );

  for ( size_t i = 0; i < n; ++i ) {
    refs.emplace_back( get_node()->get_name(), joint_names_[i] + "/position",
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
  const size_t n = joint_names_.size();

  for ( size_t i = 0; i < n; ++i ) {
    double current = last_unwrapped_cmd_[i];
    const int idx = state_interface_index_[i];
    if ( idx >= 0 ) {
      const auto &opt = state_interfaces_[static_cast<size_t>( idx )].get_optional();
      if ( opt.has_value() )
        current = opt.value();
    }

    const double target_wrapped = reference_interfaces_[i];
    if ( std::isnan( target_wrapped ) )
      continue;

    double commanded = target_wrapped;
    switch ( kinds_[i] ) {
    case JointType::CONTINUOUS:
      if ( unwrap_continuous_ )
        commanded = unwrap_to_nearest( current, target_wrapped );
      break;
    case JointType::REVOLUTE_BOUNDED:
      commanded = unwrap_to_nearest( current, target_wrapped );
      if ( enforce_limits_ )
        commanded = clamp( i, commanded );
      break;
    case JointType::PRISMATIC_BOUNDED:
      if ( enforce_limits_ )
        commanded = clamp( i, commanded );
      break;
    case JointType::FIXED:
    case JointType::OTHER:
    default:
      if ( enforce_limits_ && has_limits_[i] )
        commanded = clamp( i, commanded );
      break;
    }

    success &= command_interfaces_[i].set_value( commanded );
    last_unwrapped_cmd_[i] = commanded;
  }

  return success ? controller_interface::return_type::OK : controller_interface::return_type::ERROR;
}

// ===== Helpers =====

double SafetyPositionController::unwrap_to_nearest( double current, double target )
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
                          "Clamping joint '%s' to lower limit %.3f", joint_names_[i].c_str(), lo );
    return lo;
  }
  if ( value > hi ) {
    RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "Clamping joint '%s' to upper limit %.3f", joint_names_[i].c_str(), hi );
    return hi;
  }
  return value;
}

bool SafetyPositionController::parse_urdf_and_fill_joint_info( const std::string &urdf_xml )
{
  auto model = urdf::parseURDF( urdf_xml );
  if ( !model )
    return false;

  const size_t n = joint_names_.size();
  kinds_.assign( n, JointType::OTHER );
  has_limits_.assign( n, false );
  lower_limits_.assign( n, 0.0 );
  upper_limits_.assign( n, 0.0 );

  for ( size_t i = 0; i < n; ++i ) {
    const auto jn = joint_names_[i];
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

void SafetyPositionController::gather_interface_indices()
{
  for ( size_t i = 0; i < joint_names_.size(); ++i ) {
    state_interface_index_[i] = -1;
    for ( size_t s = 0; s < state_interfaces_.size(); ++s ) {
      auto name = state_interfaces_[s].get_name();
      auto interface_name = state_interfaces_[s].get_interface_name();
      RCLCPP_INFO_STREAM( get_node()->get_logger(),
                          "Available state interface: " << name << ", interface_name"
                                                        << interface_name );
      if ( state_interfaces_[s].get_name() == joint_names_[i] + "/position" &&
           state_interfaces_[s].get_interface_name() == "position" ) {
        state_interface_index_[i] = static_cast<int>( s );
        break;
      }
    }
    if ( state_interface_index_[i] < 0 )
      RCLCPP_WARN( get_node()->get_logger(), "No state interface 'position' found for joint '%s'.",
                   joint_names_[i].c_str() );
  }
}

} // namespace safety_position_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS( safety_position_controller::SafetyPositionController,
                        controller_interface::ChainableControllerInterface )
