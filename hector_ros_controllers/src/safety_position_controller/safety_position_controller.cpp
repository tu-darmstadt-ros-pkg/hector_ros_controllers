#include "safety_position_controller/safety_position_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <rclcpp/rclcpp.hpp>

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

  auto parsed = parse_joint_infos( this->get_robot_description(), params_.joints,
                                   params_.default_velocity_limit );
  for ( const auto &warning : parsed.warnings ) {
    RCLCPP_WARN( node->get_logger(), "%s", warning.c_str() );
  }
  if ( !parsed.ok ) {
    RCLCPP_ERROR( node->get_logger(), "Failed to parse URDF / joint limits." );
    return controller_interface::CallbackReturn::ERROR;
  }
  all_joint_names_ = std::move( parsed.all_joint_names );
  joint_infos_ = std::move( parsed.joints );

  // Status / debug ROS I/O (needed by the service callbacks below)
  diagnostics_ = std::make_unique<SafetyDiagnostics>( node, params_, collision_checker_.get() );

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

    // Build velocity-space index mapping for the collision constraint gradients
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
  processed_reference_.assign( n, std::numeric_limits<double>::quiet_NaN() );
  current_positions_.assign( n, std::numeric_limits<double>::quiet_NaN() );
  hold_positions_.assign( n, std::numeric_limits<double>::quiet_NaN() );

  if ( !gather_joint_indices() ) {
    RCLCPP_ERROR( get_node()->get_logger(),
                  "Failed to gather state interface indices for joints." );
    return controller_interface::CallbackReturn::ERROR;
  }

  collision_observer_ = std::make_unique<CollisionObserver>(
      collision_checker_.get(), all_joint_names_, params_.joints, joint_v_index_ );

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SafetyPositionController::on_activate( const rclcpp_lifecycle::State & )
{
  // reset reference interfaces and the processed references derived from them: a target
  // from before the deactivation must never be resumed on activation.
  for ( auto &ref : reference_interfaces_ ) { ref = std::numeric_limits<double>::quiet_NaN(); }
  std::fill( processed_reference_.begin(), processed_reference_.end(),
             std::numeric_limits<double>::quiet_NaN() );

  // update params in case they changed
  param_listener_->try_update_params( params_ );
  if ( params_.check_self_collisions && params_.collision_safety_zone <= params_.collision_padding ) {
    RCLCPP_ERROR( get_node()->get_logger(),
                  "collision_safety_zone (%.4f) must be > collision_padding (%.4f)",
                  params_.collision_safety_zone, params_.collision_padding );
    return controller_interface::CallbackReturn::ERROR;
  }
  if ( collision_observer_ ) {
    collision_observer_->reset();
  }

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

  if ( !setup_pipeline_on_activate() ) {
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
  diagnostics_->publishJointStateIn( reference_interfaces_ );

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
    if ( pipeline_ ) {
      pipeline_->invalidate(); // rebase to the measured state on release
    }
    success &= write_position_commands( hold_positions_ );
    return success ? controller_interface::return_type::OK : controller_interface::return_type::ERROR;
  }

  // Per-joint NaN references are propagated by enforce_limits() and turned into a zero
  // velocity demand by the pipeline, so the arm brakes to a smooth stop instead of
  // freezing or resuming a stale target.

  // resolve continuous joints & enforce limits
  enforce_limits();

  success &= run_safety_pipeline();
  if ( params_.set_current_limits ) {
    success &= write_current_limits();
  }
  // Manipulability at the last checked configuration (FK already done)
  if ( collision_checker_ && !params_.manipulability_ee_frame.empty() ) {
    last_manipulability_ =
        collision_checker_->computeManipulability( params_.manipulability_ee_frame );
  }
  diagnostics_->updateSnapshot( pipeline_.get(), collision_observer_->lastMinDistance(),
                                collision_observer_->lastSafetyZonePairs().size(),
                                last_manipulability_ );
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
  diagnostics_->publishJointStateOut( commands );
  return success;
}

void SafetyPositionController::enforce_limits()
{
  const bool bypass_active = safety_bypass_active_.load( std::memory_order_relaxed );

  // enforce limits and write updated commands into processed_reference_
  for ( size_t i = 0; i < params_.joints.size(); ++i ) {
    const double target_wrapped = reference_interfaces_[i];
    if ( std::isnan( target_wrapped ) ) {
      // A NaN reference means "no target". It MUST be propagated: keeping the previous
      // processed reference would make the pipeline continue tracking a stale target
      // (e.g. after a reactivation or an E-stop release, which both reset the reference
      // interfaces to NaN). The pipeline turns NaN into a zero velocity demand, so the
      // joint brakes to a smooth stop and holds.
      processed_reference_[i] = std::numeric_limits<double>::quiet_NaN();
      continue;
    }

    double commanded = target_wrapped;
    if ( joint_infos_[i].type == JointType::CONTINUOUS ) {
      // Joint wrapping is ALWAYS active, even during bypass
      if ( params_.unwrap_continuous_joints ) {
        commanded = unwrap_to_nearest( current_positions_[i], target_wrapped );
      }
    } else {
      if ( params_.enforce_position_limits ) {
        commanded = clamp( i, commanded, bypass_active ); // checks if the joint has limits
      }
    }

    processed_reference_[i] = commanded;
  }
}

bool SafetyPositionController::setup_pipeline_on_activate()
{
  const size_t n = params_.joints.size();
  const auto ni = static_cast<Eigen::Index>( n );
  const double dt = 1.0 / get_update_rate();

  SafetyPipeline::Config cfg;
  cfg.dt = dt;
  cfg.joints = joint_infos_;
  cfg.qp.dt = dt;
  cfg.qp.v_max.resize( ni );
  cfg.qp.a_acc.resize( ni );
  cfg.qp.a_dec.resize( ni );
  cfg.deviation_limits.resize( n );
  for ( size_t i = 0; i < n; ++i ) {
    const auto idx = static_cast<Eigen::Index>( i );
    cfg.qp.v_max[idx] = joint_infos_[i].velocity_limit;
    cfg.qp.a_acc[idx] = params_.acceleration_limits.joints_map[params_.joints[i]].limit;
    cfg.qp.a_dec[idx] = cfg.qp.a_acc[idx] * params_.deceleration_scale;
    cfg.deviation_limits[i] = params_.joint_deviation_limits.joints_map[params_.joints[i]].limit;
  }
  cfg.qp.damper_xi = params_.qp_damper_xi;
  cfg.qp.d_pad = params_.collision_padding;
  cfg.qp.d_zone = params_.collision_safety_zone;
  cfg.qp.max_repulsion_speed = params_.qp_max_repulsion_speed;
  cfg.qp.contact_crawl_speed = params_.qp_contact_crawl_speed;
  cfg.qp.max_collision_constraints = static_cast<std::size_t>( params_.qp_max_pair_constraints );
  cfg.joint_v_index = joint_v_index_;
  cfg.reference_leash_time = params_.reference_leash_time;
  cfg.tracking_leash = params_.tracking_leash;
  cfg.bypass_limit_tolerance = params_.safety_bypass_joint_limit_tolerance;
  cfg.stall_velocity_threshold = params_.qp_stall_velocity_threshold;
  cfg.park_resume_threshold = params_.park_resume_reference_threshold;
  cfg.stall_park.stall_timeout = params_.qp_stall_timeout;
  cfg.stall_park.park_timeout = params_.stall_park_timeout;

  // Tunneling check: one full-speed step must not cross the whole braking zone,
  // otherwise the damper can be skipped over between two collision checks.
  const double zone_width = params_.collision_safety_zone - params_.collision_padding;
  const double max_step = cfg.qp.v_max.maxCoeff() * dt;
  if ( params_.check_self_collisions && max_step >= zone_width ) {
    RCLCPP_WARN( get_node()->get_logger(),
                 "Max per-cycle step (%.4f) >= safety zone width (%.4f): fast joints could "
                 "tunnel through the damper zone. Increase collision_safety_zone or the "
                 "update rate.",
                 max_step, zone_width );
  }

  try {
    pipeline_ = std::make_unique<SafetyPipeline>( std::move( cfg ) );
  } catch ( const std::invalid_argument &e ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Failed to construct safety pipeline: %s", e.what() );
    return false;
  }

  if ( collision_checker_ ) {
    collision_checker_->setMaxSafetyZonePairs(
        static_cast<std::size_t>( params_.qp_max_pair_constraints ) );
  }

  qp_cmd_std_.assign( n, std::numeric_limits<double>::quiet_NaN() );
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

bool SafetyPositionController::run_safety_pipeline()
{
  const size_t n = params_.joints.size();
  const bool bypass_active = safety_bypass_active_.load( std::memory_order_relaxed );
  const bool collision_checks_active =
      !bypass_active && params_.check_self_collisions && collision_checker_ != nullptr;

  if ( pipeline_->prepare( processed_reference_, current_positions_, bypass_active ) ) {
    RCLCPP_INFO( get_node()->get_logger(), "New reference received — resuming from parked state." );
    publish_status();
  }

  // ---- Collision observation at the commanded configuration ----
  const auto snapshot = collision_observer_->observe( collision_checks_active, state_interfaces_,
                                                      pipeline_->commandedPositions(),
                                                      params_.collision_safety_zone );
  if ( snapshot.collision_started ) {
    const std::string pairs_str = diagnostics_->formatCollisionPairs(
        collision_observer_->lastSafetyZonePairs(), params_.collision_padding,
        collision_observer_->lastMinDistancePairIndex() );
    RCLCPP_WARN( get_node()->get_logger(),
                 "Collision detected (min_dist=%.4f m). Pairs in collision: %s. "
                 "QP holding/pushing out.",
                 collision_observer_->lastMinDistance(), pairs_str.c_str() );
  }
  if ( snapshot.observation.checks_active && !snapshot.observation.state_valid ) {
    RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), throttle_logging_msg,
                          "Failed to setup collision checking. Braking to a stop." );
  }

  // ---- Solve, integrate, stall/park ----
  const auto events = pipeline_->step( snapshot.observation );
  if ( events.stall.stalled ) {
    const std::string blocked_str = diagnostics_->formatBlockedDirections( *pipeline_ );
    RCLCPP_WARN( get_node()->get_logger(),
                 "Motion stalled for %.1f s: no feasible direction toward the reference "
                 "(min_dist=%.4f m).%s Upstream should replan.",
                 pipeline_->stallTime(), collision_observer_->lastMinDistance(),
                 blocked_str.c_str() );
    publish_status();
  }
  if ( events.stall.parked ) {
    RCLCPP_WARN( get_node()->get_logger(),
                 "Stalled for %.1f s — parking: the current reference is abandoned and the "
                 "limb will hold position (even if the blockage clears) until a new command "
                 "arrives.",
                 pipeline_->stallTime() );
    publish_status();
  }
  if ( events.stall.resumed ) {
    RCLCPP_INFO( get_node()->get_logger(), "Motion resumed after stall." );
    publish_status();
  }

  // ---- Write ----
  const auto &qp_cmd = pipeline_->commandedPositions();
  for ( size_t i = 0; i < n; ++i ) { qp_cmd_std_[i] = qp_cmd[static_cast<Eigen::Index>( i )]; }
  const bool write_ok = write_position_commands( qp_cmd_std_ );

  // Directional info for RViz distance-line coloring (green = moving away)
  if ( params_.debug_visualize_collisions || params_.publish_collision_distances ) {
    collision_observer_->publishDirectionalInfo( pipeline_->velocity(),
                                                 params_.collision_safety_zone );
  }

  diagnostics_->maybePublishQpDebug( *pipeline_, processed_reference_ );

  return write_ok;
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

double SafetyPositionController::clamp( const size_t i, const double value,
                                        const bool bypass_active ) const
{
  const JointInfo &joint = joint_infos_[i];
  if ( !joint.has_position_limits ) {
    return value;
  }

  // Calculate tolerance: when bypass is active, extend limits by the configured tolerance factor
  const double range = joint.upper_limit - joint.lower_limit;
  const double tolerance =
      bypass_active ? ( range * params_.safety_bypass_joint_limit_tolerance ) : 0.0;

  const double lo = std::min( joint.lower_limit - tolerance, current_positions_[i] );
  const double hi = std::max( joint.upper_limit + tolerance, current_positions_[i] );
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

void SafetyPositionController::publish_status()
{
  if ( !diagnostics_ ) {
    return;
  }
  SafetyDiagnostics::StatusFlags flags;
  flags.bypass_active = safety_bypass_active_.load( std::memory_order_relaxed );
  flags.compliant_mode = in_compliant_mode_.load( std::memory_order_relaxed );
  flags.estop_engaged = estop_engaged_.load( std::memory_order_relaxed );
  diagnostics_->publishStatus( flags );
}

} // namespace safety_position_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS( safety_position_controller::SafetyPositionController,
                        controller_interface::ChainableControllerInterface )
