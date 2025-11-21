#include "safety_position_controller/safety_position_controller.hpp"

#include <cmath>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
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
  auto qos = rclcpp::QoS( 10 );
  qos.transient_local();
  semantic_description_sub_ = get_node()->create_subscription<std_msgs::msg::String>(
      "robot_description_semantic", qos, [this]( const std_msgs::msg::String::SharedPtr msg ) {
        srdf_ = msg->data;
        srdf_received_ = true;
      } );

  if ( params_.check_self_collisions )
    collision_checker_ = std::make_unique<CollisionChecker>( node, params_.collision_padding,
                                                             params_.debug_visualize_collisions );

  if ( !parse_urdf_and_fill_joint_info( this->get_robot_description() ) ) {
    RCLCPP_ERROR( node->get_logger(), "Failed to parse URDF / joint limits." );
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

  if ( !wait_for_srdf() )
    return controller_interface::CallbackReturn::ERROR;
  if ( collision_checker_ )
    collision_checker_->initFromXml( this->get_robot_description(), srdf_, params_.joints, false );

  const size_t n = params_.joints.size();
  reference_interfaces_.assign( n, std::numeric_limits<double>::quiet_NaN() );
  joint_index_.assign( n, -1 );
  cmd_positions_.assign( n, std::numeric_limits<double>::quiet_NaN() );
  current_positions_.assign( n, std::numeric_limits<double>::quiet_NaN() );

  if ( !gather_joint_indices() ) {
    RCLCPP_ERROR( get_node()->get_logger(),
                  "Failed to gather state interface indices for joints." );
    return controller_interface::CallbackReturn::ERROR;
  }

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

  on_hold_ = false;

  // update params in case they changed
  param_listener_->try_update_params( params_ );
  params_.block_if_too_far = params_.check_self_collisions ? true : params_.block_if_too_far;
  if ( collision_checker_ ) {
    collision_checker_->updateCollisionPadding( params_.collision_padding );
    collision_checker_->updateCollisionCacheEpsilon( params_.collision_cache_epsilon );
    collision_checker_->updateDoDebugVisualization( params_.debug_visualize_collisions );
  }

  for ( size_t n = 0; n < params_.joints.size(); ++n ) {
    max_allowed_distance_per_cycle_[n] =
        velocity_limits_[n] / get_update_rate() * params_.block_velocity_scaling;
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
  for ( const auto &j : all_joint_names_ ) conf.names.emplace_back( j + "/position" );
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
  bool success = read_current_positions();
  const size_t n = params_.joints.size();

  // handle non-chained mode -> hold positions
  if ( !is_chained_ ) {
    RCLCPP_ERROR_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 2000,
                           "SafetyPositionController is CHAINED-ONLY." );
    if ( !on_hold_ ) {
      // store hold positions
      hold_positions_ = current_positions_;
      on_hold_ = true;
    }
    // write hold positions
    success &= write_position_commands( hold_positions_ );
    return success ? controller_interface::return_type::OK : controller_interface::return_type::ERROR;
  }

  // resolve continuous joints & enforce limits
  enforce_limits();
  // make sure movement is not too large
  if ( params_.block_if_too_far )
    block_if_too_far();
  if ( params_.set_current_limits )
    success &= write_current_limits();

  // check collisions with the new commands
  if ( !params_.check_self_collisions || !collision_checker_ ) {
    write_position_commands( cmd_positions_ );
  } else {
    // prepare collision checker input
    bool success_cc_setup = true;
    for ( size_t i = 0; i < all_joint_names_.size(); i++ ) {
      const auto opt = state_interfaces_[i].get_optional();
      if ( opt.has_value() )
        cc_positions_[all_joint_names_[i]] = opt.value();
      else
        success_cc_setup = false;
    }
    for ( size_t i = 0; i < n; i++ ) { cc_positions_[params_.joints[i]] = cmd_positions_[i]; }
    if ( success_cc_setup && !collision_checker_->checkCollision( cc_positions_ ) ) {
      // write commands if no collision detected
      write_position_commands( cmd_positions_ );
    } else {
      if ( !success_cc_setup )
        RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(),
                              throttle_logging_msg, "Failed to setup collision checking." );
      else
        // write current positions to hold position in case of collision
        RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(),
                              throttle_logging_msg,
                              "Collision detected! Holding current positions." );
      write_position_commands( current_positions_ );
      // success = false; // make sure parent controllers are unloaded
    }
  }

  return success ? controller_interface::return_type::OK : controller_interface::return_type::ERROR;
}

// ===== Helpers =====

bool SafetyPositionController::read_current_positions()
{
  for ( size_t i = 0; i < params_.joints.size(); ++i ) {
    if ( joint_index_[i] < 0 || static_cast<size_t>( joint_index_[i] ) >= state_interfaces_.size() ) {
      RCLCPP_ERROR_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 2000,
                             "Invalid joint index for joint '%s' (%d) but should be in [0, %zu)",
                             params_.joints[i].c_str(), joint_index_[i], state_interfaces_.size() );
      return false;
    }
    const auto &opt = state_interfaces_[static_cast<size_t>( joint_index_[i] )].get_optional();
    if ( opt.has_value() ) {
      current_positions_[i] = opt.value();
    } else {
      RCLCPP_ERROR_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 2000,
                             "Cannot get joint state for joint '%s'", params_.joints[i].c_str() );
      return false;
    }
  }
  return true;
}

bool SafetyPositionController::write_position_commands( const std::vector<double> &commands )
{
  bool success = true;
  for ( size_t i = 0; i < params_.joints.size(); ++i ) {
    if ( !std::isnan( commands[i] ) ) {
      success &= command_interfaces_[i].set_value( commands[i] );
    }
  }
  return success;
}

void SafetyPositionController::enforce_limits()
{
  // enforce limits and write updated commands into cmd_positions
  for ( size_t i = 0; i < params_.joints.size(); ++i ) {

    const double target_wrapped = reference_interfaces_[i];
    if ( std::isnan( target_wrapped ) )
      continue;

    double commanded = target_wrapped;
    switch ( kinds_[i] ) {
    case JointType::CONTINUOUS:
      if ( params_.unwrap_continuous_joints )
        commanded = unwrap_to_nearest( current_positions_[i], target_wrapped );
      break;
    case JointType::REVOLUTE_BOUNDED: // behavior for revolute and prismatic is the same
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

    cmd_positions_[i] = commanded;
  }
}
void SafetyPositionController::block_if_too_far()
{
  // check if any joint is too far from the command
  for ( size_t i = 0; i < params_.joints.size(); ++i ) {
    const double target_distance = std::abs( cmd_positions_[i] - current_positions_[i] );
    if ( !isnan( velocity_limits_[i] ) ) {
      if ( target_distance > max_allowed_distance_per_cycle_[i] ) {
        RCLCPP_WARN_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), throttle_logging_msg,
            "Joint '%s' is too far from command (target_distance=%.3f > allowed=%.3f). "
            "Reducing Target Position.",
            params_.joints[i].c_str(), target_distance, max_allowed_distance_per_cycle_[i] );
        // set command to current position to hold
        cmd_positions_[i] =
            current_positions_[i] + ( ( cmd_positions_[i] > current_positions_[i] ) ? 1.0 : -1.0 ) *
                                        max_allowed_distance_per_cycle_[i];
      }
    }
  }
}
bool SafetyPositionController::write_current_limits()
{
  bool success = true;
  for ( size_t i = 0; i < params_.joints.size(); ++i ) {

    // set current limit if enabled and command interfaces are requested
    if ( params_.set_current_limits && command_interfaces_.size() > params_.joints.size() ) {

      const auto &limit = in_compliant_mode_
                              ? params_.current_limits.joints_map[params_.joints[i]].compliant_limit
                              : params_.current_limits.joints_map[params_.joints[i]].stiff_limit;
      success &= command_interfaces_[i + params_.joints.size()].set_value( limit );
    }
  }
  return success;
}

double SafetyPositionController::unwrap_to_nearest( const double current, const double target )
{
  const double k = std::round( ( current - target ) / ( 2.0 * M_PI ) );
  return target + k * ( 2.0 * M_PI );
}

double SafetyPositionController::clamp( size_t i, double value ) const
{
  if ( !has_limits_[i] )
    return value;

  const double lo = std::min( lower_limits_[i], current_positions_[i] );
  const double hi = std::max( upper_limits_[i], current_positions_[i] );
  if ( value < lo ) {
    RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), throttle_logging_msg,
                          "Clamping joint '%s' to lower limit %.3f", params_.joints[i].c_str(), lo );
    return lo;
  }
  if ( value > hi ) {
    RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), throttle_logging_msg,
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
  for ( const auto &[name, joint] : model->joints_ ) {
    if ( joint->type != urdf::Joint::FIXED )
      all_joint_names_.push_back( name );
  }

  const size_t n = params_.joints.size();
  kinds_.assign( n, JointType::OTHER );
  has_limits_.assign( n, false );
  lower_limits_.assign( n, std::numeric_limits<double>::lowest() );
  upper_limits_.assign( n, std::numeric_limits<double>::max() );
  velocity_limits_.assign( n, std::numeric_limits<double>::max() );
  max_allowed_distance_per_cycle_.assign( n, 0.0 );

  for ( size_t i = 0; i < n; ++i ) {
    const auto jn = params_.joints[i];
    auto urdf_joint = model->getJoint( jn );
    if ( !urdf_joint )
      continue;

    switch ( urdf_joint->type ) {
    case urdf::Joint::CONTINUOUS:
      kinds_[i] = JointType::CONTINUOUS;
      velocity_limits_[i] = ( urdf_joint->limits ) ? urdf_joint->limits->velocity
                                                   : std::numeric_limits<double>::max();
      break;
    case urdf::Joint::REVOLUTE:
      kinds_[i] = JointType::REVOLUTE_BOUNDED;
      if ( urdf_joint->limits ) {
        has_limits_[i] = true;
        lower_limits_[i] = urdf_joint->limits->lower;
        upper_limits_[i] = urdf_joint->limits->upper;
        velocity_limits_[i] = urdf_joint->limits->velocity;
      }
      break;
    case urdf::Joint::PRISMATIC:
      kinds_[i] = JointType::PRISMATIC_BOUNDED;
      if ( urdf_joint->limits ) {
        has_limits_[i] = true;
        lower_limits_[i] = urdf_joint->limits->lower;
        upper_limits_[i] = urdf_joint->limits->upper;
        velocity_limits_[i] = urdf_joint->limits->velocity;
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

bool SafetyPositionController::gather_joint_indices()
{
  bool success = true;
  for ( size_t i = 0; i < params_.joints.size(); ++i ) {
    const auto &jn = params_.joints[i];
    for ( size_t s = 0; s < all_joint_names_.size(); ++s ) {
      const auto &candidate = all_joint_names_[s];
      if ( candidate == jn ) {
        joint_index_[i] = static_cast<int>( s );
        break;
      }
    }
    if ( joint_index_[i] < 0 )
      RCLCPP_WARN( get_node()->get_logger(), "Error in joint indexing '%s'.",
                   params_.joints[i].c_str() );
    success &= ( joint_index_[i] >= 0 );
    RCLCPP_DEBUG( get_node()->get_logger(), "Joint '%s' mapped to state interface index %d.",
                  params_.joints[i].c_str(), joint_index_[i] );
  }
  return success;
}

bool SafetyPositionController::wait_for_srdf()
{
  // wait for the semantic description message to be received
  rclcpp::Rate rate( 3 );
  int attempt = 0;
  int max_attempts = 50;
  while ( !srdf_received_ ) {
    rate.sleep();
    attempt++;
    if ( attempt % 10 == 0 )
      RCLCPP_INFO( get_node()->get_logger(), "Waiting for semantic robot description on topic " );
    if ( attempt > max_attempts )
      return false;
  }
  return true;
}

} // namespace safety_position_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS( safety_position_controller::SafetyPositionController,
                        controller_interface::ChainableControllerInterface )
