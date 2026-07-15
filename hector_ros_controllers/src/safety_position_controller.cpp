#include "safety_position_controller/safety_position_controller.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

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

  // Get SRDF: prefer parameter, fall back to topic subscription
  if ( get_node()->has_parameter( "robot_description_semantic" ) ) {
    srdf_ = get_node()->get_parameter( "robot_description_semantic" ).as_string();
  }
  if ( !srdf_.empty() ) {
    srdf_received_ = true;
    RCLCPP_INFO( get_node()->get_logger(), "Loaded robot_description_semantic from parameter." );
  } else {
    RCLCPP_INFO( get_node()->get_logger(),
                 "robot_description_semantic parameter not set, subscribing to topic." );
    auto qos = rclcpp::QoS( 10 );
    qos.transient_local();
    semantic_description_sub_ = get_node()->create_subscription<std_msgs::msg::String>(
        "robot_description_semantic", qos, [this]( const std_msgs::msg::String::SharedPtr msg ) {
          srdf_ = msg->data;
          srdf_received_ = true;
        } );
  }

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

  // Debug joint state publishers (dynamically reconfigurable)
  param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>( node );
  update_debug_publishers( params_.publish_debug_joint_states );
  cb_handle_debug_pubs_ = param_subscriber_->add_parameter_callback(
      "publish_debug_joint_states",
      [this]( const rclcpp::Parameter &p ) { update_debug_publishers( p.as_bool() ); },
      node->get_name() );

  // QP debug introspection publisher (dynamically reconfigurable)
  qp_debug_pub_ =
      node->create_publisher<hector_ros_controllers_msgs::msg::SafetyQpDebug>( "~/qp_debug", 10 );
  qp_debug_enabled_.store( params_.publish_qp_debug, std::memory_order_relaxed );
  cb_handle_qp_debug_ = param_subscriber_->add_parameter_callback(
      "publish_qp_debug",
      [this]( const rclcpp::Parameter &p ) {
        qp_debug_enabled_.store( p.as_bool(), std::memory_order_relaxed );
        RCLCPP_INFO( get_node()->get_logger(), "QP debug publishing %s",
                     p.as_bool() ? "enabled" : "disabled" );
      },
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
  was_in_collision_ = false;

  status_timer_.reset();
  if ( params_.status_publish_rate > 0.0 ) {
    const auto period = std::chrono::duration<double>( 1.0 / params_.status_publish_rate );
    status_timer_ = get_node()->create_wall_timer( period, [this]() { publish_status(); } );
  }
  if ( collision_checker_ ) {
    collision_checker_->updateCollisionPadding( params_.collision_padding );
    collision_checker_->updateCollisionCacheEpsilon( params_.collision_cache_epsilon );
    collision_checker_->updateDoDebugVisualization( params_.debug_visualize_collisions );
    collision_checker_->updatePublishCollisionDistances( params_.publish_collision_distances );
  }

  RCLCPP_INFO( get_node()->get_logger(),
               "SafetyPositionController config: joints=%zu, collisions=%s, broadphase=%s, "
               "padding=%.4f, safety_zone=%.4f, cache_eps=%.1e, debug_viz=%s, "
               "publish_distances=%s",
               params_.joints.size(), params_.check_self_collisions ? "ON" : "OFF",
               params_.use_broadphase ? "ON" : "OFF", params_.collision_padding,
               params_.collision_safety_zone, params_.collision_cache_epsilon,
               params_.debug_visualize_collisions ? "ON" : "OFF",
               params_.publish_collision_distances ? "ON" : "OFF" );

  if ( !setup_qp_on_activate() ) {
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
  status_timer_.reset();

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
    qp_state_valid_ = false; // rebase to the measured state on release
    success &= write_position_commands( hold_positions_ );
    return success ? controller_interface::return_type::OK : controller_interface::return_type::ERROR;
  }

  // NaN references need no special handling: per-joint NaN targets simply demand zero
  // velocity, so the arm brakes to a smooth stop instead of freezing.

  // resolve continuous joints & enforce limits
  enforce_limits();

  success &= update_qp_mode();
  if ( params_.set_current_limits ) {
    success &= write_current_limits();
  }
  // Manipulability at the last checked configuration (FK already done)
  if ( collision_checker_ && !params_.manipulability_ee_frame.empty() ) {
    last_manipulability_ =
        collision_checker_->computeManipulability( params_.manipulability_ee_frame );
  }
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

bool SafetyPositionController::setup_qp_on_activate()
{
  const size_t n = params_.joints.size();
  const auto ni = static_cast<Eigen::Index>( n );
  const double dt = 1.0 / get_update_rate();

  SafetyQpParams qp_params;
  qp_params.dt = dt;
  qp_params.v_max.resize( ni );
  qp_params.a_acc.resize( ni );
  qp_params.a_dec.resize( ni );
  for ( size_t i = 0; i < n; ++i ) {
    const auto idx = static_cast<Eigen::Index>( i );
    qp_params.v_max[idx] = velocity_limits_[i];
    qp_params.a_acc[idx] = params_.acceleration_limits.joints_map[params_.joints[i]].limit;
    qp_params.a_dec[idx] = qp_params.a_acc[idx] * params_.deceleration_scale;
  }
  qp_params.damper_xi = params_.qp_damper_xi;
  qp_params.d_pad = params_.collision_padding;
  qp_params.d_zone = params_.collision_safety_zone;
  qp_params.max_repulsion_speed = params_.qp_max_repulsion_speed;
  qp_params.contact_crawl_speed = params_.qp_contact_crawl_speed;
  qp_params.max_collision_constraints = static_cast<std::size_t>( params_.qp_max_pair_constraints );

  try {
    qp_limiter_ = std::make_unique<SafetyQpLimiter>( n, qp_params );
  } catch ( const std::invalid_argument &e ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Failed to construct safety QP: %s", e.what() );
    return false;
  }

  if ( collision_checker_ ) {
    collision_checker_->setMaxSafetyZonePairs(
        static_cast<std::size_t>( params_.qp_max_pair_constraints ) );
  }

  // Tunneling check: one full-speed step must not cross the whole braking zone,
  // otherwise the damper can be skipped over between two collision checks.
  const double zone_width = params_.collision_safety_zone - params_.collision_padding;
  const double max_step = qp_params.v_max.maxCoeff() * dt;
  if ( params_.check_self_collisions && max_step >= zone_width ) {
    RCLCPP_WARN( get_node()->get_logger(),
                 "Max per-cycle step (%.4f) >= safety zone width (%.4f): fast joints could "
                 "tunnel through the damper zone. Increase collision_safety_zone or the "
                 "update rate.",
                 max_step, zone_width );
  }

  qp_cmd_ = Eigen::VectorXd::Zero( ni );
  qp_vel_ = Eigen::VectorXd::Zero( ni );
  qp_ref_leashed_ = Eigen::VectorXd::Zero( ni );
  qp_a_dec_ = qp_params.a_dec;
  qp_cmd_std_.assign( n, std::numeric_limits<double>::quiet_NaN() );
  qp_input_.v_des = Eigen::VectorXd::Zero( ni );
  qp_input_.v_prev = Eigen::VectorXd::Zero( ni );
  qp_input_.q = Eigen::VectorXd::Zero( ni );
  qp_input_.q_lo = Eigen::VectorXd::Constant( ni, -std::numeric_limits<double>::infinity() );
  qp_input_.q_hi = Eigen::VectorXd::Constant( ni, std::numeric_limits<double>::infinity() );
  qp_input_.collisions.reserve( qp_params.max_collision_constraints );
  qp_last_result_ = SafetyQpResult{};
  qp_state_valid_ = false;
  stall_time_ = 0.0;
  stalled_ = false;
  parked_ = false;
  parked_reference_.assign( n, std::numeric_limits<double>::quiet_NaN() );
  if ( params_.stall_park_timeout > 0.0 && params_.stall_park_timeout <= params_.qp_stall_timeout ) {
    RCLCPP_WARN( get_node()->get_logger(),
                 "stall_park_timeout (%.2f s) <= qp_stall_timeout (%.2f s): the limb will park "
                 "as soon as the stall is reported.",
                 params_.stall_park_timeout, params_.qp_stall_timeout );
  }

  RCLCPP_INFO( get_node()->get_logger(),
               "Safety QP: joints=%zu, xi=%.2f, max_pairs=%ld, decel_scale=%.1f, "
               "crawl=%.2f, leash(ref=%.2fs, track=%.2frad)",
               n, params_.qp_damper_xi, params_.qp_max_pair_constraints, params_.deceleration_scale,
               params_.qp_contact_crawl_speed, params_.reference_leash_time, params_.tracking_leash );
  return true;
}

bool SafetyPositionController::update_qp_mode()
{
  const size_t n = params_.joints.size();
  const double dt = 1.0 / get_update_rate();

  if ( !qp_state_valid_ ) {
    for ( size_t i = 0; i < n; ++i ) {
      qp_cmd_[static_cast<Eigen::Index>( i )] = current_positions_[i];
    }
    qp_vel_.setZero();
    stall_time_ = 0.0;
    stalled_ = false;
    qp_state_valid_ = true;
  }

  const bool bypass_active = safety_bypass_active_.load( std::memory_order_relaxed );
  const bool collision_checks_active =
      !bypass_active && params_.check_self_collisions && collision_checker_;

  // ---- Desired velocity toward the (leashed) reference ----
  bool wants_motion = false;
  for ( size_t i = 0; i < n; ++i ) {
    const auto idx = static_cast<Eigen::Index>( i );
    const double target = cmd_positions_[i];
    double diff = 0.0;
    if ( !std::isnan( target ) ) {
      diff = ( kinds_[i] == JointType::CONTINUOUS ) ? get_signed_distance( qp_cmd_[idx], target )
                                                    : ( target - qp_cmd_[idx] );
      if ( params_.reference_leash_time > 0.0 ) {
        const double leash = velocity_limits_[i] * params_.reference_leash_time;
        diff = std::clamp( diff, -leash, leash );
      }
    }
    qp_ref_leashed_[idx] = qp_cmd_[idx] + diff;
    qp_input_.v_des[idx] =
        SafetyQpLimiter::desiredVelocity( diff, velocity_limits_[i], qp_a_dec_[idx], dt );
    wants_motion |= std::abs( qp_input_.v_des[idx] ) > params_.qp_stall_velocity_threshold;

    // Position limits (damper handled inside the QP); bypass extends them like clamp()
    if ( has_limits_[i] && kinds_[i] != JointType::CONTINUOUS ) {
      const double tolerance = bypass_active ? ( upper_limits_[i] - lower_limits_[i] ) *
                                                   params_.safety_bypass_joint_limit_tolerance
                                             : 0.0;
      qp_input_.q_lo[idx] = lower_limits_[i] - tolerance;
      qp_input_.q_hi[idx] = upper_limits_[i] + tolerance;
    } else {
      qp_input_.q_lo[idx] = -std::numeric_limits<double>::infinity();
      qp_input_.q_hi[idx] = std::numeric_limits<double>::infinity();
    }

    // Per-joint deviation box around the leashed reference: bounds how far every link
    // may leave the upstream-validated path. One-sided (widened to include the current
    // command): prevents drifting further, never demands catch-up.
    const double dev_limit =
        bypass_active ? 0.0 : params_.joint_deviation_limits.joints_map[params_.joints[i]].limit;
    if ( dev_limit > 0.0 ) {
      qp_input_.q_lo[idx] = std::max( qp_input_.q_lo[idx],
                                      std::min( qp_ref_leashed_[idx] - dev_limit, qp_cmd_[idx] ) );
      qp_input_.q_hi[idx] = std::min( qp_input_.q_hi[idx],
                                      std::max( qp_ref_leashed_[idx] + dev_limit, qp_cmd_[idx] ) );
    }
  }
  // ---- Parked: the latched reference is abandoned; hold until a NEW command ----
  if ( parked_ ) {
    bool new_command = false;
    for ( size_t i = 0; i < n && !new_command; ++i ) {
      const double ref = cmd_positions_[i];
      if ( std::isnan( ref ) ) {
        continue;
      }
      if ( std::isnan( parked_reference_[i] ) ) {
        new_command = true;
        break;
      }
      const double delta = ( kinds_[i] == JointType::CONTINUOUS )
                               ? get_signed_distance( parked_reference_[i], ref )
                               : ( ref - parked_reference_[i] );
      new_command = std::abs( delta ) > params_.park_resume_reference_threshold;
    }
    if ( new_command ) {
      parked_ = false;
      stalled_ = false;
      stall_time_ = 0.0;
      RCLCPP_INFO( get_node()->get_logger(),
                   "New reference received — resuming from parked state." );
      publish_status();
    } else {
      qp_input_.v_des.setZero();
      wants_motion = false;
    }
  }

  qp_input_.v_prev = qp_vel_;
  qp_input_.q = qp_cmd_;

  // ---- Collision damper constraints (evaluated at the commanded configuration) ----
  qp_input_.collisions.clear();
  qp_constraint_pair_names_.clear();
  bool collision_state_observed = false;
  if ( collision_checks_active ) {
    bool cc_setup_ok = true;
    for ( size_t i = 0; i < all_joint_names_.size(); ++i ) {
      const auto opt = state_interfaces_[i].get_optional();
      if ( opt.has_value() ) {
        cc_positions_[all_joint_names_[i]] = opt.value();
      } else {
        cc_setup_ok = false;
      }
    }
    for ( size_t i = 0; i < n; ++i ) {
      cc_positions_[params_.joints[i]] = qp_cmd_[static_cast<Eigen::Index>( i )];
    }

    if ( cc_setup_ok ) {
      collision_state_observed = true;
      // Always request gradients for the full zone: they ARE the constraints.
      collision_checker_->setSafetyZoneThreshold( params_.collision_safety_zone );
      const auto cc_result = collision_checker_->checkCollision( cc_positions_ );
      last_min_distance_ = cc_result.min_distance;
      last_min_distance_pair_index_ = cc_result.min_distance_pair_index;
      last_safety_zone_pairs_ = cc_result.safety_zone_pairs;

      if ( cc_result.in_collision ) {
        if ( !was_in_collision_ ) {
          const std::string pairs_str =
              format_collision_pairs( cc_result.safety_zone_pairs, params_.collision_padding,
                                      cc_result.min_distance_pair_index );
          RCLCPP_WARN( get_node()->get_logger(),
                       "Collision detected (min_dist=%.4f m). Pairs in collision: %s. "
                       "QP holding/pushing out.",
                       cc_result.min_distance, pairs_str.c_str() );
          was_in_collision_ = true;
        }
      } else {
        was_in_collision_ = false;
      }

      for ( const auto &pair_info : cc_result.safety_zone_pairs ) {
        QpCollisionConstraint c;
        c.distance = pair_info.distance;
        c.normal.resize( static_cast<Eigen::Index>( n ) );
        for ( size_t i = 0; i < n; ++i ) {
          const auto idx = static_cast<Eigen::Index>( i );
          c.normal[idx] = ( joint_v_index_[i] >= 0 && joint_v_index_[i] < pair_info.gradient.size() )
                              ? pair_info.gradient[joint_v_index_[i]]
                              : 0.0;
        }
        // A pair the controlled joints cannot influence must not constrain (or even
        // infeasible-block) the QP.
        if ( c.normal.norm() > 1e-12 ) {
          qp_input_.collisions.push_back( std::move( c ) );
          const auto [name_a, name_b] = collision_checker_->getPairNames( pair_info.pair_index );
          qp_constraint_pair_names_.push_back( name_a + "<->" + name_b );
        }
      }

      if ( cc_result.in_collision && qp_input_.collisions.empty() ) {
        // In collision but no usable constraints (e.g. NaN/Inf positions → blanket
        // collision without pairs): state untrusted → stop demanding motion, QP brakes.
        qp_input_.v_des.setZero();
        wants_motion = false;
      }
    } else {
      RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), throttle_logging_msg,
                            "Failed to setup collision checking. Braking to a stop." );
      was_in_collision_ = false;
      // Safety state unobservable → stop demanding motion; QP brakes smoothly.
      qp_input_.v_des.setZero();
      wants_motion = false;
    }
  } else {
    was_in_collision_ = false;
    last_min_distance_ = std::numeric_limits<double>::max();
    last_safety_zone_pairs_.clear();
  }

  // ---- Solve, integrate, clamp ----
  qp_last_result_ = qp_limiter_->solve( qp_input_ );
  qp_vel_ = qp_last_result_.v;
  qp_cmd_ += qp_vel_ * dt;

  if ( params_.tracking_leash > 0.0 ) {
    // Anti-windup: never run further ahead of the measured position than the leash
    for ( size_t i = 0; i < n; ++i ) {
      const auto idx = static_cast<Eigen::Index>( i );
      const double ahead = qp_cmd_[idx] - current_positions_[i];
      if ( std::abs( ahead ) > params_.tracking_leash ) {
        qp_cmd_[idx] = current_positions_[i] + std::copysign( params_.tracking_leash, ahead );
      }
    }
  }

  // ---- Stall detection: reference demands motion but the QP output is ~zero ----
  const bool moving = qp_vel_.cwiseAbs().maxCoeff() > params_.qp_stall_velocity_threshold;
  if ( wants_motion && !moving ) {
    stall_time_ += dt;
    if ( !stalled_ && stall_time_ >= params_.qp_stall_timeout ) {
      stalled_ = true;
      const std::string blocked_str = format_blocked_directions();
      RCLCPP_WARN( get_node()->get_logger(),
                   "Motion stalled for %.1f s: no feasible direction toward the reference "
                   "(min_dist=%.4f m).%s Upstream should replan.",
                   stall_time_, last_min_distance_, blocked_str.c_str() );
      publish_status();
    }
    if ( !parked_ && params_.stall_park_timeout > 0.0 && stall_time_ >= params_.stall_park_timeout ) {
      parked_ = true;
      parked_reference_ = cmd_positions_;
      RCLCPP_WARN( get_node()->get_logger(),
                   "Stalled for %.1f s — parking: the current reference is abandoned and the "
                   "limb will hold position (even if the blockage clears) until a new command "
                   "arrives.",
                   stall_time_ );
      publish_status();
    }
  } else if ( !parked_ ) { // while parked, wants_motion is forced false — keep stall state
    if ( stalled_ ) {
      RCLCPP_INFO( get_node()->get_logger(), "Motion resumed after stall." );
      publish_status();
    }
    stall_time_ = 0.0;
    stalled_ = false;
  }

  // ---- Write ----
  for ( size_t i = 0; i < n; ++i ) { qp_cmd_std_[i] = qp_cmd_[static_cast<Eigen::Index>( i )]; }
  const bool write_ok = write_position_commands( qp_cmd_std_ );

  // Directional info for RViz distance-line coloring (green = moving away)
  if ( collision_state_observed &&
       ( params_.debug_visualize_collisions || params_.publish_collision_distances ) ) {
    const std::size_t num_pairs = collision_checker_->getNumCollisionPairs();
    std::vector<double> per_pair_dir( num_pairs, std::numeric_limits<double>::quiet_NaN() );
    for ( const auto &pi : last_safety_zone_pairs_ ) {
      if ( pi.pair_index < num_pairs ) {
        double dot = 0.0;
        for ( size_t i = 0; i < n; ++i ) {
          if ( joint_v_index_[i] >= 0 && joint_v_index_[i] < pi.gradient.size() ) {
            dot += pi.gradient[joint_v_index_[i]] * qp_vel_[static_cast<Eigen::Index>( i )];
          }
        }
        per_pair_dir[pi.pair_index] = dot;
      }
    }
    collision_checker_->setDirectionalInfo( per_pair_dir, params_.collision_safety_zone );
  }

  // Diagnostics consumed by update_status_snapshot()
  if ( qp_debug_enabled_.load( std::memory_order_relaxed ) ) {
    publish_qp_debug();
  }

  return write_ok;
}

void SafetyPositionController::publish_qp_debug()
{
  if ( !qp_debug_pub_ || !qp_limiter_ ) {
    return;
  }
  const size_t n = params_.joints.size();

  hector_ros_controllers_msgs::msg::SafetyQpDebug msg;
  msg.header.stamp = get_node()->now();
  msg.joint_names = params_.joints;
  msg.v_des.resize( n );
  msg.v_cmd.resize( n );
  msg.box_lb.resize( n );
  msg.box_ub.resize( n );
  msg.q_cmd.resize( n );
  msg.q_ref.resize( n );
  for ( size_t i = 0; i < n; ++i ) {
    const auto idx = static_cast<Eigen::Index>( i );
    msg.v_des[i] = qp_input_.v_des[idx];
    msg.v_cmd[i] = qp_vel_[idx];
    msg.box_lb[i] = qp_limiter_->lastBoxLower()[idx];
    msg.box_ub[i] = qp_limiter_->lastBoxUpper()[idx];
    msg.q_cmd[i] = qp_cmd_[idx];
    msg.q_ref[i] = cmd_positions_[i];
  }

  const auto num_cc = static_cast<size_t>( qp_last_result_.num_collision_constraints );
  msg.pair_names.assign(
      qp_constraint_pair_names_.begin(),
      qp_constraint_pair_names_.begin() +
          static_cast<std::ptrdiff_t>( std::min( num_cc, qp_constraint_pair_names_.size() ) ) );
  msg.pair_distances.resize( msg.pair_names.size() );
  msg.pair_rhs.resize( msg.pair_names.size() );
  msg.pair_velocities.resize( msg.pair_names.size() );
  for ( size_t k = 0; k < msg.pair_names.size(); ++k ) {
    const auto idx = static_cast<Eigen::Index>( k );
    msg.pair_distances[k] = qp_input_.collisions[k].distance;
    msg.pair_rhs[k] = ( idx < qp_last_result_.collision_rhs.size() )
                          ? qp_last_result_.collision_rhs[idx]
                          : std::numeric_limits<double>::quiet_NaN();
    msg.pair_velocities[k] = ( idx < qp_last_result_.collision_velocity.size() )
                                 ? qp_last_result_.collision_velocity[idx]
                                 : std::numeric_limits<double>::quiet_NaN();
  }

  msg.solved = qp_last_result_.solved;
  msg.braking = qp_last_result_.braking;
  msg.push_out_relaxed = qp_last_result_.push_out_relaxed;
  msg.bounds_conflict = qp_last_result_.bounds_conflict;
  msg.solve_time_us = qp_last_result_.solve_time_us;
  msg.iterations = qp_last_result_.iterations;

  qp_debug_pub_->publish( msg );
}

std::string SafetyPositionController::format_blocked_directions() const
{
  // For each joint the reference wants to move but that is not moving, name the
  // constraint that most strongly opposes the desired direction (normal component
  // against the motion). Gives an immediate answer to "why is joint X stuck".
  std::ostringstream oss;
  const double thr = params_.qp_stall_velocity_threshold;
  for ( size_t i = 0; i < params_.joints.size(); ++i ) {
    const auto idx = static_cast<Eigen::Index>( i );
    const double v_des = qp_input_.v_des[idx];
    if ( std::abs( v_des ) <= thr || std::abs( qp_vel_[idx] ) > thr ) {
      continue;
    }
    const double dir = v_des > 0.0 ? 1.0 : -1.0;
    // Most opposing constraint: largest -(normal_i * dir)
    double worst_opposition = 0.0;
    size_t worst_k = qp_input_.collisions.size();
    for ( size_t k = 0; k < qp_input_.collisions.size(); ++k ) {
      const double opposition = -qp_input_.collisions[k].normal[idx] * dir;
      if ( opposition > worst_opposition ) {
        worst_opposition = opposition;
        worst_k = k;
      }
    }
    oss << " " << params_.joints[i] << "[" << ( dir > 0.0 ? "+" : "-" ) << "]: ";
    if ( worst_k < qp_input_.collisions.size() && worst_k < qp_constraint_pair_names_.size() ) {
      oss << "blocked by '" << qp_constraint_pair_names_[worst_k] << "' (d=" << std::fixed
          << std::setprecision( 4 ) << qp_input_.collisions[worst_k].distance
          << " m, g_i=" << std::setprecision( 3 ) << -worst_opposition * dir << ");";
    } else {
      oss << "no opposing collision constraint (velocity/position bounds?);";
    }
  }
  const std::string s = oss.str();
  return s.empty() ? std::string( " No blocked joints identified." )
                   : std::string( " Blocked directions:" ) + s;
}

std::string SafetyPositionController::format_collision_pairs(
    const std::vector<CollisionResult::PairInfo> &pairs, const double max_distance,
    const std::size_t fallback_pair_index ) const
{
  if ( !collision_checker_ )
    return "unknown";

  std::ostringstream oss;
  bool first = true;
  for ( const auto &pi : pairs ) {
    if ( pi.distance > max_distance )
      continue;
    const auto [name_a, name_b] = collision_checker_->getPairNames( pi.pair_index );
    if ( name_a.empty() || name_b.empty() )
      continue;
    if ( !first )
      oss << ", ";
    oss << "'" << name_a << "' <-> '" << name_b << "': " << std::fixed << std::setprecision( 4 )
        << pi.distance << " m";
    first = false;
  }
  if ( !first )
    return oss.str();

  // Fallback: no per-pair list available (e.g. directional scaling off → empty safety_zone_pairs).
  const auto [name_a, name_b] = collision_checker_->getPairNames( fallback_pair_index );
  if ( name_a.empty() || name_b.empty() )
    return "unknown";
  std::ostringstream fallback;
  fallback << "'" << name_a << "' <-> '" << name_b << "'";
  return fallback.str();
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

    // Fallback: joints without a usable URDF velocity limit would otherwise be stepped
    // unbounded (max()) and could jump to a far-away target in a single cycle.
    if ( !std::isfinite( velocity_limits_[i] ) || velocity_limits_[i] <= 0.0 ) {
      RCLCPP_WARN(
          get_node()->get_logger(),
          "Joint '%s' has no usable URDF velocity limit; using default_velocity_limit=%.3f",
          jn.c_str(), params_.default_velocity_limit );
      velocity_limits_[i] = params_.default_velocity_limit;
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
  snap.manipulability = last_manipulability_;
  snap.num_pairs_in_safety_zone = static_cast<uint32_t>( last_safety_zone_pairs_.size() );
  snap.qp_solved = qp_last_result_.solved;
  snap.qp_braking = qp_last_result_.braking;
  snap.qp_push_out_relaxed = qp_last_result_.push_out_relaxed;
  snap.qp_num_collision_constraints =
      static_cast<uint32_t>( qp_last_result_.num_collision_constraints );
  snap.qp_solve_time_us = qp_last_result_.solve_time_us;
  snap.stalled = stalled_;
  snap.parked = parked_;
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
  msg.num_pairs_in_safety_zone = snap.num_pairs_in_safety_zone;
  msg.manipulability = snap.manipulability;
  msg.qp_solved = snap.qp_solved;
  msg.qp_braking = snap.qp_braking;
  msg.qp_push_out_relaxed = snap.qp_push_out_relaxed;
  msg.qp_num_collision_constraints = snap.qp_num_collision_constraints;
  msg.qp_solve_time_us = snap.qp_solve_time_us;
  msg.stalled = snap.stalled;
  msg.parked = snap.parked;

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
