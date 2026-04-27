#include "safety_position_controller/safety_position_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

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
  // invalidate reference interfaces
  for ( auto &ref : reference_interfaces_ ) { ref = std::numeric_limits<double>::quiet_NaN(); }
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
    RCLCPP_WARN( get_node()->get_logger(), "Exception thrown during init stage with message: %s",
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

  if ( params_.check_self_collisions ) {
    collision_checker_ = std::make_unique<CollisionChecker>( node, params_.collision_padding,
                                                             params_.collision_cache_epsilon,
                                                             params_.debug_visualize_collisions );
  }

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
          publish_status();
        } );
  }

  // Service to temporarily bypass safety checks (collision + relaxed joint limits)
  bypass_safety_checks_service_ = node->create_service<std_srvs::srv::SetBool>(
      "~/bypass_safety_checks",
      [this]( const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
              std::shared_ptr<std_srvs::srv::SetBool::Response> response ) {
        if ( request->data ) {
          // Enable bypass: start timer to auto-disable
          safety_bypass_active_.store( true, std::memory_order_relaxed );
          const double timeout_sec = params_.safety_bypass_timeout;
          safety_bypass_timer_ =
              get_node()->create_wall_timer( std::chrono::duration<double>( timeout_sec ), [this]() {
                safety_bypass_active_.store( false, std::memory_order_relaxed );
                safety_bypass_timer_->cancel();
                RCLCPP_WARN( get_node()->get_logger(),
                             "Safety bypass timeout expired. Safety checks re-enabled." );
                publish_status();
              } );
          response->success = true;
          response->message = "Safety bypass ENABLED. Collision checks disabled, joint limits "
                              "relaxed. Will auto-disable after " +
                              std::to_string( timeout_sec ) + " seconds.";
          RCLCPP_WARN( get_node()->get_logger(), "%s", response->message.c_str() );
          publish_status();
        } else {
          // Disable bypass: cancel timer and re-enable safety
          safety_bypass_active_.store( false, std::memory_order_relaxed );
          if ( safety_bypass_timer_ ) {
            safety_bypass_timer_->cancel();
            safety_bypass_timer_.reset();
          }
          response->success = true;
          response->message = "Safety bypass DISABLED. Normal safety checks restored.";
          RCLCPP_INFO( get_node()->get_logger(), "%s", response->message.c_str() );
          publish_status();
        }
      } );

  // Status publisher (latched)
  auto qos_latched = rclcpp::QoS( 1 ).transient_local().reliable();
  status_pub_ =
      node->create_publisher<hector_ros_controllers_msgs::msg::SafetyPositionControllerStatus>(
          "~/status", qos_latched );

  // Periodic status publishing
  if ( params_.status_publish_rate > 0.0 ) {
    const auto period = std::chrono::duration<double>( 1.0 / params_.status_publish_rate );
    status_timer_ = node->create_wall_timer( period, [this]() { publish_status(); } );
  }

  // Debug joint state publishers (dynamically reconfigurable)
  param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>( node );
  update_debug_publishers( params_.publish_debug_joint_states );
  cb_handle_debug_pubs_ = param_subscriber_->add_parameter_callback(
      "publish_debug_joint_states",
      [this]( const rclcpp::Parameter &p ) { update_debug_publishers( p.as_bool() ); },
      node->get_name() );

  // Non-chained command subscriber (RT buffer)
  joints_command_subscriber_ = node->create_subscription<CmdType>(
      "~/commands", rclcpp::SystemDefaultsQoS(),
      [this]( const CmdType::SharedPtr msg ) { rt_command_ptr_.writeFromNonRT( msg ); } );

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

  if ( !wait_for_srdf() ) {
    return controller_interface::CallbackReturn::ERROR;
  }

  if ( collision_checker_ ) {
    collision_checker_->setBroadphase( params_.use_broadphase );
    if ( !collision_checker_->initFromXml( this->get_robot_description(), srdf_, params_.joints,
                                           false ) ) {
      RCLCPP_ERROR( node->get_logger(), "Failed to initialize collision checker from URDF/SRDF." );
      return controller_interface::CallbackReturn::ERROR;
    }

    // Build velocity-space index mapping for directional collision scaling
    joint_v_index_.resize( params_.joints.size(), -1 );
    for ( size_t i = 0; i < params_.joints.size(); ++i ) {
      joint_v_index_[i] = collision_checker_->getJointVelocityIndex( params_.joints[i] );
      if ( joint_v_index_[i] < 0 ) {
        RCLCPP_WARN( node->get_logger(), "Joint '%s' not found in collision model velocity space",
                     params_.joints[i].c_str() );
      }
    }
  }

  const size_t n = params_.joints.size();
  reference_interfaces_.assign( n, std::numeric_limits<double>::quiet_NaN() );
  joint_index_.assign( n, -1 );
  cmd_positions_.assign( n, std::numeric_limits<double>::quiet_NaN() );
  current_positions_.assign( n, std::numeric_limits<double>::quiet_NaN() );
  hold_positions_.assign( n, std::numeric_limits<double>::quiet_NaN() );

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
  // reset reference interfaces
  for ( auto &ref : reference_interfaces_ ) { ref = std::numeric_limits<double>::quiet_NaN(); }

  // update params in case they changed
  param_listener_->try_update_params( params_ );
  if ( params_.check_self_collisions && params_.collision_safety_zone <= params_.collision_padding ) {
    RCLCPP_ERROR( get_node()->get_logger(),
                  "collision_safety_zone (%.4f) must be > collision_padding (%.4f)",
                  params_.collision_safety_zone, params_.collision_padding );
    return controller_interface::CallbackReturn::ERROR;
  }
  last_min_distance_ = std::numeric_limits<double>::max();
  last_safety_zone_pairs_.clear();
  last_distance_scale_ = 1.0;
  last_effective_scale_ = 1.0;
  last_worst_directional_derivative_ = std::numeric_limits<double>::max();
  if ( collision_checker_ ) {
    collision_checker_->updateCollisionPadding( params_.collision_padding );
    collision_checker_->updateCollisionCacheEpsilon( params_.collision_cache_epsilon );
    collision_checker_->updateDoDebugVisualization( params_.debug_visualize_collisions );
    collision_checker_->updatePublishCollisionDistances( params_.publish_collision_distances );
  }

  RCLCPP_INFO( get_node()->get_logger(),
               "SafetyPositionController config: joints=%zu, collisions=%s, broadphase=%s, "
               "padding=%.4f, safety_zone=%.4f, cache_eps=%.1e, directional=%s, debug_viz=%s, "
               "publish_distances=%s",
               params_.joints.size(), params_.check_self_collisions ? "ON" : "OFF",
               params_.use_broadphase ? "ON" : "OFF", params_.collision_padding,
               params_.collision_safety_zone, params_.collision_cache_epsilon,
               params_.directional_collision_scaling ? "ON" : "OFF",
               params_.debug_visualize_collisions ? "ON" : "OFF",
               params_.publish_collision_distances ? "ON" : "OFF" );

  // compute max allowed distance per cycle
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

  // E-stop subscription
  estop_subscriber_ = get_node()->create_subscription<std_msgs::msg::Bool>(
      "~/safety_estop", rclcpp::SystemDefaultsQoS(),
      [this]( const std_msgs::msg::Bool::SharedPtr msg ) {
        const bool prev = estop_active_.load( std::memory_order_relaxed );
        estop_active_.store( msg->data, std::memory_order_relaxed );
        if ( msg->data != prev ) {
          RCLCPP_WARN( get_node()->get_logger(), "E-STOP %s", msg->data ? "ENGAGED" : "DISENGAGED" );
        }
      } );

  if ( !is_chained_ ) {
    // Non-chained mode: re-create command subscriber (destroyed in on_deactivate)
    joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
        "~/commands", rclcpp::SystemDefaultsQoS(),
        [this]( const CmdType::SharedPtr msg ) { rt_command_ptr_.writeFromNonRT( msg ); } );
  }

  // reset RT buffer
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>( nullptr );

  estop_engaged_.store( false, std::memory_order_relaxed );

  publish_status();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SafetyPositionController::on_deactivate( const rclcpp_lifecycle::State & )
{
  estop_subscriber_.reset();
  joints_command_subscriber_.reset();

  estop_active_.store( false, std::memory_order_relaxed );
  estop_engaged_.store( false, std::memory_order_relaxed );

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
SafetyPositionController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for ( const auto &j : params_.joints ) { conf.names.emplace_back( j + "/position" ); }
  if ( params_.set_current_limits ) {
    for ( const auto &j : params_.joints ) { conf.names.emplace_back( j + "/current" ); }
  }
  return conf;
}

controller_interface::InterfaceConfiguration
SafetyPositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for ( const auto &j : all_joint_names_ ) { conf.names.emplace_back( j + "/position" ); }
  return conf;
}

std::vector<hardware_interface::CommandInterface>
SafetyPositionController::on_export_reference_interfaces()
{
  const size_t n = params_.joints.size();
  std::vector<hardware_interface::CommandInterface> refs;
  refs.reserve( n );

  const std::string controller_name = get_node()->get_name();

  for ( size_t i = 0; i < n; ++i ) {
    const std::string resource_name = controller_name + "/" + params_.joints[i];
    refs.emplace_back( resource_name, hardware_interface::HW_IF_POSITION, &reference_interfaces_[i] );
  }
  return refs;
}

controller_interface::return_type
SafetyPositionController::update_reference_from_subscribers( const rclcpp::Time &,
                                                             const rclcpp::Duration & )
{
  // In chained mode, references come from upstream controller
  if ( is_in_chained_mode() ) {
    return controller_interface::return_type::OK;
  }

  // Non-chained mode: read from RT buffer
  const auto cmd = rt_command_ptr_.readFromRT();
  if ( !cmd || !( *cmd ) ) {
    // no new command → keep previous reference_interfaces_
    return controller_interface::return_type::OK;
  }

  const auto &data = ( *cmd )->data;
  const size_t n_expected = params_.joints.size();

  if ( data.size() < n_expected ) {
    RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), throttle_logging_msg,
                          "Received command size %zu, expected %zu. Using prefix.", data.size(),
                          n_expected );
  }

  const size_t n = std::min( n_expected, data.size() );
  for ( size_t i = 0; i < n; ++i ) { reference_interfaces_[i] = data[i]; }

  return controller_interface::return_type::OK;
}

controller_interface::return_type
SafetyPositionController::update_and_write_commands( const rclcpp::Time &, const rclcpp::Duration & )
{
  bool success = read_current_positions();
  if ( !success ) {
    return controller_interface::return_type::ERROR;
  }

  const size_t n = params_.joints.size();

  // Debug: incoming joint states
  publish_debug_joint_state_in();

  // E-stop edge handling
  const bool estop_active = estop_active_.load( std::memory_order_relaxed );
  bool estop_engaged = estop_engaged_.load( std::memory_order_relaxed );

  if ( estop_active != estop_engaged ) {
    if ( estop_active ) {
      // engage E-stop: record hold positions
      hold_positions_ = current_positions_;
      estop_engaged_.store( true, std::memory_order_relaxed );
      estop_engaged = true;
      RCLCPP_WARN( get_node()->get_logger(), "E-STOP engaged: holding positions for %zu joints", n );
      publish_status();
    } else {
      // release E-stop
      estop_engaged_.store( false, std::memory_order_relaxed );
      RCLCPP_WARN( get_node()->get_logger(), "E-STOP released: resuming normal commands" );
      publish_status();
      // on release, invalidate old commands once
      for ( auto &ref : reference_interfaces_ ) ref = std::numeric_limits<double>::quiet_NaN();
      return controller_interface::return_type::OK;
    }
  }

  // If E-stop engaged → always hold recorded positions (no checks)
  if ( estop_engaged ) {
    success &= write_position_commands( hold_positions_ );
    return success ? controller_interface::return_type::OK : controller_interface::return_type::ERROR;
  }

  bool nan_in_refs = std::any_of( reference_interfaces_.begin(), reference_interfaces_.end(),
                                  []( double v ) { return std::isnan( v ); } );

  if ( nan_in_refs ) {
    RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), throttle_logging_msg,
                          "NaN detected in reference interfaces. Not writing commands." );
    return controller_interface::return_type::OK;
  }

  // resolve continuous joints & enforce limits
  enforce_limits();

  // ---- Distance-based velocity scaling ----
  const bool bypass_active = safety_bypass_active_.load( std::memory_order_relaxed );
  const bool collision_checks_active =
      !bypass_active && params_.check_self_collisions && collision_checker_;

  // Compute velocity scale factor from previous cycle's min distance
  double distance_scale = 1.0;
  if ( collision_checks_active ) {
    const auto &d = last_min_distance_;
    const auto &d_pad = params_.collision_padding;
    const auto &d_zone = params_.collision_safety_zone;

    if ( d <= d_pad ) {
      distance_scale = 0.0;
    } else if ( d < d_zone ) {
      distance_scale = ( d - d_pad ) / ( d_zone - d_pad );
    }
    // else: distance_scale remains 1.0 (full speed)
  }

  // Always apply velocity-limited stepping when collision checks are active
  double effective_scale = distance_scale;
  double worst_directional_derivative = std::numeric_limits<double>::max();
  if ( collision_checks_active ) {
    // Directional scaling: only slow down if moving toward any collision in safety zone
    if ( distance_scale < 1.0 && params_.directional_collision_scaling &&
         !last_safety_zone_pairs_.empty() ) {
      for ( const auto &pair_info : last_safety_zone_pairs_ ) {
        const double dir_deriv = compute_directional_derivative( pair_info.gradient );
        worst_directional_derivative = std::min( worst_directional_derivative, dir_deriv );
      }
      if ( worst_directional_derivative >= 0.0 ) {
        // ALL safety-zone pairs say motion moves away or is tangent → allow full speed
        effective_scale = 1.0;
      }
      // else: at least one pair worsens → keep distance_scale
    }
    apply_velocity_limits( effective_scale );
  }
  last_distance_scale_ = distance_scale;
  last_effective_scale_ = effective_scale;
  last_worst_directional_derivative_ = worst_directional_derivative;

  if ( params_.set_current_limits ) {
    success &= write_current_limits();
  }

  // ---- Collision check ----
  if ( !collision_checks_active ) {
    if ( bypass_active && params_.check_self_collisions && collision_checker_ ) {
      RCLCPP_DEBUG_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(),
                             throttle_logging_msg, "Safety bypass active: skipping collision check" );
    }
    write_position_commands( cmd_positions_ );
  } else {
    // prepare collision checker input
    bool success_cc_setup = true;
    for ( size_t i = 0; i < all_joint_names_.size(); ++i ) {
      const auto opt = state_interfaces_[i].get_optional();
      if ( opt.has_value() ) {
        cc_positions_[all_joint_names_[i]] = opt.value();
      } else {
        success_cc_setup = false;
      }
    }
    for ( size_t i = 0; i < n; ++i ) { cc_positions_[params_.joints[i]] = cmd_positions_[i]; }

    if ( success_cc_setup ) {
      // Request gradient computation when inside safety zone (or when directional scaling is on)
      const double gradient_threshold = ( params_.directional_collision_scaling &&
                                          last_min_distance_ < params_.collision_safety_zone )
                                            ? params_.collision_safety_zone
                                            : 0.0;
      collision_checker_->setSafetyZoneThreshold( gradient_threshold );
      const auto cc_result = collision_checker_->checkCollision( cc_positions_ );
      last_min_distance_ = cc_result.min_distance;
      last_safety_zone_pairs_ = cc_result.safety_zone_pairs;

      // Provide directional derivative info to collision checker for visualization
      if ( params_.debug_visualize_collisions || params_.publish_collision_distances ) {
        const std::size_t num_pairs = collision_checker_->getNumCollisionPairs();
        std::vector<double> per_pair_dir_derivs( num_pairs, std::numeric_limits<double>::quiet_NaN() );
        for ( const auto &pi : last_safety_zone_pairs_ ) {
          if ( pi.pair_index < num_pairs ) {
            per_pair_dir_derivs[pi.pair_index] = compute_directional_derivative( pi.gradient );
          }
        }
        collision_checker_->setDirectionalInfo( per_pair_dir_derivs, params_.collision_safety_zone );
      }

      if ( !cc_result.in_collision ) {
        write_position_commands( cmd_positions_ );
      } else {
        RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(),
                              throttle_logging_msg,
                              "Collision detected (min_dist=%.4f)! Holding current positions.",
                              cc_result.min_distance );
        write_position_commands( current_positions_ );
      }
    } else {
      RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(),
                            throttle_logging_msg, "Failed to setup collision checking." );
      write_position_commands( current_positions_ );
    }
  }

  // Compute manipulability index if collision checker is available (FK already done)
  if ( collision_checker_ && !params_.manipulability_ee_frame.empty() ) {
    last_manipulability_ =
        collision_checker_->computeManipulability( params_.manipulability_ee_frame );
  }

  // Publish a snapshot of last_*_ to rt_status_buffer_ so publish_status() (called from
  // the wall timer or from any other thread) reads a consistent view without locking.
  update_status_snapshot();

  return success ? controller_interface::return_type::OK : controller_interface::return_type::ERROR;
}

// ===== Helpers =====

bool SafetyPositionController::read_current_positions()
{
  for ( size_t i = 0; i < params_.joints.size(); ++i ) {
    if ( joint_index_[i] < 0 || static_cast<size_t>( joint_index_[i] ) >= state_interfaces_.size() ) {
      RCLCPP_ERROR_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), throttle_logging_msg,
                             "Invalid joint index for joint '%s' (%d) but should be in [0, %zu)",
                             params_.joints[i].c_str(), joint_index_[i], state_interfaces_.size() );
      return false;
    }
    const auto &opt = state_interfaces_[static_cast<size_t>( joint_index_[i] )].get_optional();
    if ( opt.has_value() ) {
      current_positions_[i] = opt.value();
    } else {
      RCLCPP_ERROR_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), throttle_logging_msg,
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
  publish_debug_joint_state_out( commands );
  return success;
}

void SafetyPositionController::enforce_limits()
{
  const bool bypass_active = safety_bypass_active_.load( std::memory_order_relaxed );

  // enforce limits and write updated commands into cmd_positions_
  for ( size_t i = 0; i < params_.joints.size(); ++i ) {
    const double target_wrapped = reference_interfaces_[i];
    if ( std::isnan( target_wrapped ) ) {
      continue;
    }

    double commanded = target_wrapped;
    if ( kinds_[i] == JointType::CONTINUOUS ) {
      // Joint wrapping is ALWAYS active, even during bypass
      if ( params_.unwrap_continuous_joints ) {
        commanded = unwrap_to_nearest( current_positions_[i], target_wrapped );
      }
    } else {
      if ( params_.enforce_position_limits ) {
        commanded = clamp( i, commanded, bypass_active ); // checks if the joint has limits
      }
    }

    cmd_positions_[i] = commanded;
  }
}

void SafetyPositionController::apply_velocity_limits( const double distance_scale )
{
  // distance_scale: 1.0 → full speed, 0.0 → stop; can be used to smoothly reduce speed when close to collisions
  const double clamped_scale = std::clamp( distance_scale, 0.0, 1.0 );
  for ( size_t i = 0; i < params_.joints.size(); ++i ) {
    if ( !std::isnan( velocity_limits_[i] ) ) {
      // shortest signed distance from current -> command
      const double diff = ( kinds_[i] == JointType::CONTINUOUS )
                              ? get_signed_distance( current_positions_[i], cmd_positions_[i] )
                              : ( cmd_positions_[i] - current_positions_[i] );
      const double max_step = max_allowed_distance_per_cycle_[i] * clamped_scale;

      // RCLCPP_INFO( get_node()->get_logger(),
      //             "Joint '%s': distance=%.4f, max_step=%.4f, scale=%.3f",
      //             params_.joints[i].c_str(), diff, max_step, clamped_scale );

      if ( std::abs( diff ) > max_step ) {
        RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(),
                              throttle_logging_msg,
                              "Joint '%s' step limited (|diff|=%.4f > allowed=%.4f, "
                              "dist_scale=%.3f). [current=%.3f, cmd=%.3f]",
                              params_.joints[i].c_str(), std::abs( diff ), max_step, clamped_scale,
                              current_positions_[i], cmd_positions_[i] );

        if ( max_step <= 0.0 ) {
          cmd_positions_[i] = current_positions_[i]; // zero speed = hold
        } else {
          cmd_positions_[i] = current_positions_[i] + std::copysign( max_step, diff );
        }
      }
    }
  }
}

double SafetyPositionController::compute_directional_derivative( const Eigen::VectorXd &gradient ) const
{
  double dot_product = 0.0;
  for ( size_t i = 0; i < params_.joints.size(); ++i ) {
    if ( joint_v_index_[i] < 0 || joint_v_index_[i] >= gradient.size() )
      continue;

    // For continuous joints, use shortest-path angular distance
    double delta_q_i;
    if ( kinds_[i] == JointType::CONTINUOUS ) {
      delta_q_i = get_signed_distance( current_positions_[i], cmd_positions_[i] );
    } else {
      delta_q_i = cmd_positions_[i] - current_positions_[i];
    }

    dot_product += gradient[joint_v_index_[i]] * delta_q_i;
  }
  return dot_product;
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

double SafetyPositionController::get_signed_distance( double value_a, double value_b )
{
  // Normalize into [-2π, 2π)
  double diff = std::fmod( value_b - value_a, 2.0 * M_PI );

  // Wrap into [-π, π]
  if ( diff > M_PI ) {
    diff -= 2.0 * M_PI;
  } else if ( diff < -M_PI ) {
    diff += 2.0 * M_PI;
  }

  return diff;
}

double SafetyPositionController::clamp( const size_t i, const double value,
                                        const bool bypass_active ) const
{
  if ( !has_limits_[i] ) {
    return value;
  }

  // Calculate tolerance: when bypass is active, extend limits by the configured tolerance factor
  const double range = upper_limits_[i] - lower_limits_[i];
  const double tolerance =
      bypass_active ? ( range * params_.safety_bypass_joint_limit_tolerance ) : 0.0;

  const double lo = std::min( lower_limits_[i] - tolerance, current_positions_[i] );
  const double hi = std::max( upper_limits_[i] + tolerance, current_positions_[i] );
  if ( value < lo ) {
    RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), throttle_logging_msg,
                          "Clamping joint '%s' to lower limit %.3f%s", params_.joints[i].c_str(),
                          lo, bypass_active ? " (bypass active, tolerance applied)" : "" );
    return lo;
  }
  if ( value > hi ) {
    RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), throttle_logging_msg,
                          "Clamping joint '%s' to upper limit %.3f%s", params_.joints[i].c_str(),
                          hi, bypass_active ? " (bypass active, tolerance applied)" : "" );
    return hi;
  }
  return value;
}

bool SafetyPositionController::parse_urdf_and_fill_joint_info( const std::string &urdf_xml )
{
  const auto model = urdf::parseURDF( urdf_xml );
  if ( !model ) {
    return false;
  }
  for ( const auto &[name, joint] : model->joints_ ) {
    if ( joint->type != urdf::Joint::FIXED ) {
      all_joint_names_.push_back( name );
    }
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
    if ( !urdf_joint ) {
      continue;
    }

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
    if ( joint_index_[i] < 0 ) {
      RCLCPP_WARN( get_node()->get_logger(), "Error in joint indexing '%s'.",
                   params_.joints[i].c_str() );
    }
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
  constexpr int max_attempts = 50;
  while ( !srdf_received_ ) {
    rate.sleep();
    ++attempt;
    if ( attempt % 10 == 0 ) {
      RCLCPP_INFO( get_node()->get_logger(),
                   "Waiting for semantic robot description on topic 'robot_description_semantic'" );
    }
    if ( attempt > max_attempts ) {
      return false;
    }
  }
  return true;
}

void SafetyPositionController::update_debug_publishers( bool enable )
{
  if ( enable ) {
    if ( !debug_in_js_pub_ ) {
      debug_in_js_pub_ =
          get_node()->create_publisher<sensor_msgs::msg::JointState>( "~/debug_in_joint_states", 10 );
    }
    if ( !debug_out_js_pub_ ) {
      debug_out_js_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
          "~/debug_out_joint_states", 10 );
    }
    RCLCPP_INFO( get_node()->get_logger(), "Debug joint state publishers enabled" );
  } else {
    debug_in_js_pub_.reset();
    debug_out_js_pub_.reset();
  }
}

void SafetyPositionController::publish_debug_joint_state_in()
{
  if ( !debug_in_js_pub_ ) {
    return;
  }

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = get_node()->now();
  msg.name = params_.joints;
  msg.position = reference_interfaces_;
  debug_in_js_pub_->publish( msg );
}

void SafetyPositionController::publish_debug_joint_state_out( const std::vector<double> &positions )
{
  if ( !debug_out_js_pub_ ) {
    return;
  }

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = get_node()->now();
  msg.name = params_.joints;
  msg.position = positions;
  debug_out_js_pub_->publish( msg );
}

void SafetyPositionController::update_status_snapshot()
{
  StatusSnapshot snap;
  snap.min_distance = last_min_distance_;
  snap.distance_scale = last_distance_scale_;
  snap.effective_scale = last_effective_scale_;
  snap.worst_directional_derivative = last_worst_directional_derivative_;
  snap.manipulability = last_manipulability_;
  snap.num_pairs_in_safety_zone = static_cast<uint32_t>( last_safety_zone_pairs_.size() );
  rt_status_buffer_.writeFromNonRT( snap );
}

void SafetyPositionController::publish_status()
{
  if ( !status_pub_ ) {
    return;
  }
  const StatusSnapshot snap = *rt_status_buffer_.readFromNonRT();
  hector_ros_controllers_msgs::msg::SafetyPositionControllerStatus msg;
  msg.header.stamp = get_node()->now();
  msg.safety_bypass_active = safety_bypass_active_.load( std::memory_order_relaxed );
  msg.compliant_mode = in_compliant_mode_;
  msg.current_limits_enabled = params_.set_current_limits;
  msg.collision_check_enabled = params_.check_self_collisions;
  msg.estop_engaged = estop_engaged_.load( std::memory_order_relaxed );
  msg.position_limits_enforced = params_.enforce_position_limits;
  msg.min_collision_distance = snap.min_distance;
  msg.distance_scale = snap.distance_scale;
  msg.effective_scale = snap.effective_scale;
  msg.worst_directional_derivative = snap.worst_directional_derivative;
  msg.num_pairs_in_safety_zone = snap.num_pairs_in_safety_zone;
  msg.manipulability = snap.manipulability;

  // Populate active current limits per joint (only meaningful when current_limits_enabled)
  if ( params_.set_current_limits ) {
    msg.joint_names = params_.joints;
    msg.current_limits.reserve( params_.joints.size() );
    for ( size_t i = 0; i < params_.joints.size(); ++i ) {
      const auto &limit = in_compliant_mode_
                              ? params_.current_limits.joints_map[params_.joints[i]].compliant_limit
                              : params_.current_limits.joints_map[params_.joints[i]].stiff_limit;
      msg.current_limits.push_back( limit );
    }
  }

  status_pub_->publish( msg );
}

} // namespace safety_position_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS( safety_position_controller::SafetyPositionController,
                        controller_interface::ChainableControllerInterface )
