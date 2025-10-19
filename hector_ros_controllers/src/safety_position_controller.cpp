#include "safety_position_controller/safety_position_controller.hpp"

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>

// URDF parsing
#include <urdf_model/model.h>
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
  node->declare_parameter<std::string>( "command_interface", command_interface_name_ );
  node->declare_parameter<std::string>( "state_interface", state_interface_name_ );
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
  command_interface_name_ = node->get_parameter( "command_interface" ).as_string();
  state_interface_name_ = node->get_parameter( "state_interface" ).as_string();
  robot_description_param_ = node->get_parameter( "robot_description" ).as_string();
  unwrap_continuous_ = node->get_parameter( "unwrap_continuous_joints" ).as_bool();
  enforce_limits_ = node->get_parameter( "enforce_position_limits" ).as_bool();

  if ( joint_names_.empty() ) {
    RCLCPP_ERROR( node->get_logger(), "'joints' parameter must not be empty." );
    return controller_interface::CallbackReturn::ERROR;
  }

  std::string urdf_xml;
  if ( !node->get_parameter( robot_description_param_, urdf_xml ) ) {
    (void)node->get_parameter( "robot_description", urdf_xml );
  }
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
  ref_buffer_.assign( n, 0.0 );
  last_unwrapped_cmd_.assign( n, 0.0 );
  cmd_handles_.assign( n, nullptr );
  state_handles_.assign( n, nullptr );
  ref_ptrs_.clear();

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

  gather_interface_handles();

  for ( size_t i = 0; i < joint_names_.size(); ++i ) {
    if ( state_handles_[i] ) {
      auto optional = state_handles_[i]->get_optional();
      if ( optional.has_value() )
        last_unwrapped_cmd_[i] = optional.value();
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
  conf.names.reserve( joint_names_.size() );
  for ( const auto &j : joint_names_ ) {
    conf.names.emplace_back( j + "/" + command_interface_name_ );
  }
  return conf;
}

controller_interface::InterfaceConfiguration
SafetyPositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve( joint_names_.size() );
  for ( const auto &j : joint_names_ ) {
    conf.names.emplace_back( j + "/" + state_interface_name_ );
  }
  return conf;
}

std::vector<hardware_interface::CommandInterface>
SafetyPositionController::on_export_reference_interfaces()
{
  const size_t n = joint_names_.size();
  std::vector<hardware_interface::CommandInterface> refs;
  refs.reserve( n );

  ref_ptrs_.assign( n, nullptr );
  for ( size_t i = 0; i < n; ++i ) {
    refs.emplace_back( joint_names_[i], command_interface_name_, &ref_buffer_[i] );
    ref_ptrs_[i] = &ref_buffer_[i];
  }
  return refs;
}

controller_interface::return_type
SafetyPositionController::update_and_write_commands( const rclcpp::Time &, const rclcpp::Duration & )
{
  if ( !is_chained_ ) {
    RCLCPP_ERROR_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 2000,
                           "SafetyPositionController is CHAINED-ONLY; nothing will be written." );
  }

  const size_t n = joint_names_.size();
  if ( cmd_handles_.size() != n || state_handles_.size() != n ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Interface handles not assigned correctly." );
    return controller_interface::return_type::ERROR;
  }
  bool success = true;
  for ( size_t i = 0; i < n; ++i ) {
    auto state = state_handles_[i]->get_optional();
    const double current = state.has_value() ? state.value() : last_unwrapped_cmd_[i];
    const double target_wrapped = ref_buffer_[i];

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

    if ( cmd_handles_[i] )
      success &= cmd_handles_[i]->set_value( commanded );
    last_unwrapped_cmd_[i] = commanded;
  }
  if ( !success )
    return controller_interface::return_type::ERROR;
  return controller_interface::return_type::OK;
}

// ===== Helpers =====

double SafetyPositionController::unwrap_to_nearest( const double current, const double target )
{
  const double k = std::round( ( current - target ) / ( 2.0 * M_PI ) );
  return target + k * ( 2.0 * M_PI );
}

double SafetyPositionController::clamp( const size_t i, const double value ) const
{
  if ( !has_limits_[i] )
    return value;

  const double lo = lower_limits_[i];
  const double hi = upper_limits_[i];

  if ( value < lo ) {
    RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "Clamping joint '%s' command %.6g to lower limit %.6g.",
                          joint_names_[i].c_str(), value, lo );
    return lo;
  }
  if ( value > hi ) {
    RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "Clamping joint '%s' command %.6g to upper limit %.6g.",
                          joint_names_[i].c_str(), value, hi );
    return hi;
  }
  return value;
}

bool SafetyPositionController::parse_urdf_and_fill_joint_info( const std::string &urdf_xml )
{
  const auto model = urdf::parseURDF( urdf_xml );
  if ( !model )
    return false;

  const size_t n = joint_names_.size();
  kinds_.assign( n, JointType::OTHER );
  has_limits_.assign( n, false );
  lower_limits_.assign( n, 0.0 );
  upper_limits_.assign( n, 0.0 );

  for ( size_t i = 0; i < n; ++i ) {
    const auto &jn = joint_names_[i];
    auto urdf_joint = model->getJoint( jn );
    if ( !urdf_joint ) {
      RCLCPP_WARN( get_node()->get_logger(),
                   "Joint '%s' not found in URDF. Treating as OTHER (no limits).", jn.c_str() );
      continue;
    }

    switch ( urdf_joint->type ) {
    case urdf::Joint::CONTINUOUS:
      kinds_[i] = JointType::CONTINUOUS;
      has_limits_[i] = false;
      break;

    case urdf::Joint::REVOLUTE:
      kinds_[i] = JointType::REVOLUTE_BOUNDED;
      if ( urdf_joint->limits ) {
        lower_limits_[i] = urdf_joint->limits->lower;
        upper_limits_[i] = urdf_joint->limits->upper;
        has_limits_[i] = true;
      }
      break;

    case urdf::Joint::PRISMATIC:
      kinds_[i] = JointType::PRISMATIC_BOUNDED;
      if ( urdf_joint->limits ) {
        lower_limits_[i] = urdf_joint->limits->lower;
        upper_limits_[i] = urdf_joint->limits->upper;
        has_limits_[i] = true;
      }
      break;

    case urdf::Joint::FIXED:
      kinds_[i] = JointType::FIXED;
      has_limits_[i] = false;
      break;

    default:
      kinds_[i] = JointType::OTHER;
      has_limits_[i] = false;
      break;
    }

    if ( has_limits_[i] && !( lower_limits_[i] < upper_limits_[i] ) ) {
      RCLCPP_WARN( get_node()->get_logger(),
                   "Joint '%s' has non-increasing limits [%.6g, %.6g]. Disabling clamping.",
                   jn.c_str(), lower_limits_[i], upper_limits_[i] );
      has_limits_[i] = false;
    }
  }

  return true;
}

void SafetyPositionController::gather_interface_handles()
{
  auto &cmds = this->command_interfaces_;
  auto &states = this->state_interfaces_;

  cmd_handles_.assign( joint_names_.size(), nullptr );
  state_handles_.assign( joint_names_.size(), nullptr );

  auto find_cmd = [&]( const std::string &name,
                       const std::string &iface ) -> hardware_interface::LoanedCommandInterface * {
    for ( auto &ci : cmds ) {
      if ( ci.get_name() == name && ci.get_interface_name() == iface )
        return &ci;
    }
    return nullptr;
  };

  auto find_state =
      [&]( const std::string &name,
           const std::string &iface ) -> const hardware_interface::LoanedStateInterface * {
    for ( const auto &si : states ) {
      if ( si.get_name() == name && si.get_interface_name() == iface )
        return &si;
    }
    return nullptr;
  };

  for ( size_t i = 0; i < joint_names_.size(); ++i ) {
    cmd_handles_[i] = find_cmd( joint_names_[i], command_interface_name_ );
    state_handles_[i] = find_state( joint_names_[i], state_interface_name_ );

    if ( !cmd_handles_[i] ) {
      RCLCPP_ERROR( get_node()->get_logger(), "Missing command interface %s/%s",
                    joint_names_[i].c_str(), command_interface_name_.c_str() );
    }
    if ( !state_handles_[i] ) {
      RCLCPP_WARN( get_node()->get_logger(),
                   "Missing state interface %s/%s; using last_unwrapped_cmd_ as current.",
                   joint_names_[i].c_str(), state_interface_name_.c_str() );
    }
  }
}

} // namespace safety_position_controller

PLUGINLIB_EXPORT_CLASS( safety_position_controller::SafetyPositionController,
                        controller_interface::ChainableControllerInterface )
