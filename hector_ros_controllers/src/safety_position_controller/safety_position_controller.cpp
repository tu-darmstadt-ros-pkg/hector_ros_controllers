#include "safety_position_controller/safety_position_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

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

bool SafetyPositionController::on_set_chained_mode( const bool )
{
  // switching the reference source must not resume a stale target
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
                                                             params_.collision_cache_epsilon );
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
          const double timeout_sec = params_.safety_bypass_timeout;
          const auto deadline = std::chrono::steady_clock::now() +
                                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                    std::chrono::duration<double>( timeout_sec ) );
          safety_bypass_deadline_.store( deadline.time_since_epoch().count(),
                                         std::memory_order_relaxed );
          safety_bypass_active_.store( true, std::memory_order_relaxed );
          response->success = true;
          response->message = "Safety bypass ENABLED. Collision checks disabled, joint limits "
                              "relaxed. Will auto-disable after " +
                              std::to_string( timeout_sec ) + " seconds.";
          RCLCPP_WARN( get_node()->get_logger(), "%s", response->message.c_str() );
          publish_status();
        } else {
          clear_bypass();
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

  // Transient local, matching the e-stop manager, which publishes its aggregated state
  // once per change and latches it: a volatile subscriber joining afterwards is told
  // nothing and would run as if no stop were in effect. A depth of one would be enough
  // for the latch, but an engage and the release after it can arrive together while the
  // executor is busy, and the engage must not be overwritten before its callback runs.
  estop_subscriber_ = node->create_subscription<std_msgs::msg::Bool>(
      params_.e_stop_topic, rclcpp::QoS( rclcpp::KeepLast( 10 ) ).reliable().transient_local(),
      [this]( const std_msgs::msg::Bool::SharedPtr msg ) { note_estop_request( msg->data ); } );
  RCLCPP_INFO( node->get_logger(), "Listening for the soft e-stop on '%s'.",
               params_.e_stop_topic.c_str() );

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
    if ( collision_checker_->getNumCollisionPairs() == 0 ) {
      // Every distance query would answer "no collision" at DBL_MAX, so collision
      // checking would be silently off while reporting healthy.
      RCLCPP_ERROR( node->get_logger(),
                    "Collision checking is enabled but no collision pair is left to check. "
                    "Check that the URDF has collision geometry, that the SRDF does not "
                    "disable every pair, and that the 'joints' parameter names joints of "
                    "this model." );
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
  current_positions_.assign( n, std::numeric_limits<double>::quiet_NaN() );
  hold_positions_.assign( n, std::numeric_limits<double>::quiet_NaN() );
  all_positions_.assign( all_joint_names_.size(), std::numeric_limits<double>::quiet_NaN() );

  if ( !gather_joint_indices() ) {
    RCLCPP_ERROR( get_node()->get_logger(),
                  "Failed to gather state interface indices for joints." );
    return controller_interface::CallbackReturn::ERROR;
  }

  collision_observer_ = std::make_unique<CollisionObserver>(
      collision_checker_.get(), all_joint_names_, params_.joints, joint_v_index_ );
  for ( const auto &joint : collision_observer_->unmodeledJoints() ) {
    RCLCPP_WARN( node->get_logger(),
                 "Joint '%s' is not in the collision model (unknown or unsupported DoF "
                 "layout); it stays at its neutral position in every check.",
                 joint.c_str() );
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SafetyPositionController::on_activate( const rclcpp_lifecycle::State & )
{
  // a target from before the deactivation must never be resumed on activation
  for ( auto &ref : reference_interfaces_ ) { ref = std::numeric_limits<double>::quiet_NaN(); }
  // Nobody is supervising a controller that is only now starting, so it starts guarded
  // whatever was in effect before. First, so no failing check below can skip it.
  clear_bypass();

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

  if ( collision_checker_ ) {
    collision_checker_->updateCollisionPadding( params_.collision_padding );
    collision_checker_->updateCollisionCacheEpsilon( params_.collision_cache_epsilon );
    if ( params_.debug_visualize_collisions || params_.publish_collision_distances ) {
      if ( !collision_visualizer_ ) {
        collision_visualizer_ =
            std::make_unique<CollisionVisualizer>( get_node(), params_.collision_visualization_rate );
      } else {
        // Kept across a deactivate/activate cycle so the topic stays advertised; the
        // rate still has to follow a parameter change.
        collision_visualizer_->setPublishRate( params_.collision_visualization_rate );
      }
    } else {
      collision_visualizer_.reset();
    }
    if ( !collision_checker_->setManipulabilityFrame( params_.manipulability_ee_frame ) ) {
      RCLCPP_WARN( get_node()->get_logger(),
                   "manipulability_ee_frame '%s' is not in the model; manipulability disabled.",
                   params_.manipulability_ee_frame.c_str() );
    }
  }

  // Resolve the per-joint current limits once; the update loop must not do map lookups.
  if ( params_.set_current_limits ) {
    stiff_current_limits_.resize( params_.joints.size() );
    compliant_current_limits_.resize( params_.joints.size() );
    for ( size_t i = 0; i < params_.joints.size(); ++i ) {
      const auto &limits = params_.current_limits.joints_map.at( params_.joints[i] );
      stiff_current_limits_[i] = limits.stiff_limit;
      compliant_current_limits_[i] = limits.compliant_limit;
    }
  }
  // Recompute manipulability once per status message; fall back to 1 Hz in event-only
  // mode, where events force a refresh anyway.
  const double status_rate = params_.status_publish_rate > 0.0 ? params_.status_publish_rate : 1.0;
  manipulability_period_ =
      std::max( 1, static_cast<int>( std::lround( get_update_rate() / status_rate ) ) );
  manipulability_countdown_ = 0;

  // Hand the resolved values over BEFORE the status timer can fire: publishStatus()
  // reads them from executor threads, so they must not be written while it can run.
  diagnostics_->configure( params_.joints, stiff_current_limits_, compliant_current_limits_ );

  RCLCPP_INFO( get_node()->get_logger(),
               "SafetyPositionController config: joints=%zu, collisions=%s, broadphase=%s, "
               "padding=%.4f, safety_zone=%.4f, cache_eps=%.1e, debug_viz=%s, "
               "publish_distances=%s, hold_unrequested=%s",
               params_.joints.size(), params_.check_self_collisions ? "ON" : "OFF",
               params_.use_broadphase ? "ON" : "OFF", params_.collision_padding,
               params_.collision_safety_zone, params_.collision_cache_epsilon,
               params_.debug_visualize_collisions ? "ON" : "OFF",
               params_.publish_collision_distances ? "ON" : "OFF",
               params_.hold_unrequested_joints ? "ON" : "OFF" );

  if ( !setup_pipeline_on_activate() ) {
    return controller_interface::CallbackReturn::ERROR;
  }

  if ( state_interfaces_.size() != all_joint_names_.size() ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Expected %zu state interfaces, got %zu.",
                  all_joint_names_.size(), state_interfaces_.size() );
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

  // drop commands received while inactive
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>( nullptr );
  state_read_failure_time_ = 0.0;
  park_on_recovery_pending_ = false;
  last_manipulability_ = 0.0;
  // A pulse received while inactive must not park a freshly activated controller; a
  // held E-stop is carried by estop_active_ and re-engages on the first cycle.
  estop_engage_pending_.store( false, std::memory_order_relaxed );

  // Last, so a failed activation does not leave a timer publishing status for a
  // controller that never became active.
  status_timer_.reset();
  if ( params_.status_publish_rate > 0.0 ) {
    const auto period = std::chrono::duration<double>( 1.0 / params_.status_publish_rate );
    status_timer_ = get_node()->create_wall_timer( period, [this]() { publish_status(); } );
  }

  publish_status();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SafetyPositionController::on_deactivate( const rclcpp_lifecycle::State & )
{
  status_timer_.reset();
  estop_engaged_.store( false, std::memory_order_relaxed );
  // A bypass is granted for as long as someone is watching this controller run. It must
  // not outlive the controller and be inherited by whatever activates next.
  clear_bypass();

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

  // All of the message or none of it: a prefix would leave the joints it does not name
  // on the targets of an earlier command, which is a pose nobody asked for, and the
  // sender cannot tell that from success.
  if ( data.size() != n_expected ) {
    RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), throttle_logging_msg,
                          "Ignoring a command for %zu joints; this controller has %zu.",
                          data.size(), n_expected );
    return controller_interface::return_type::OK;
  }

  for ( size_t i = 0; i < n_expected; ++i ) { reference_interfaces_[i] = data[i]; }

  return controller_interface::return_type::OK;
}

controller_interface::return_type
SafetyPositionController::update_and_write_commands( const rclcpp::Time &,
                                                     const rclcpp::Duration &period )
{
  // Debug: incoming joint states
  diagnostics_->publishJointStateIn( reference_interfaces_ );

  // The bypass lapses on its own so an unattended one cannot last: enforced here rather
  // than by a timer, which would mean an unsynchronised handle shared with the control
  // thread. Checked before anything reads the flag, so the cycle that ends it is already
  // guarded.
  bool status_event = expire_bypass();

  // E-stop first: it must act even while the joint states are unreadable, otherwise an
  // engage (or a whole engage+release pulse) during a read outage would be lost.
  // Consumed unconditionally (not short-circuited): a held E-stop is carried by the
  // level, and the latch must not survive into the release cycle.
  const bool engage_pending = estop_engage_pending_.exchange( false, std::memory_order_relaxed );
  const bool estop_active = estop_active_.load( std::memory_order_relaxed ) || engage_pending;

  if ( estop_active != estop_engaged_.load( std::memory_order_relaxed ) ) {
    if ( estop_active ) {
      // Mark the hold as unrecorded; it is taken from the freshest readable states
      // below. Non-finite entries are never written, so until then nothing moves.
      hold_positions_.assign( params_.joints.size(), std::numeric_limits<double>::quiet_NaN() );
      RCLCPP_WARN( get_node()->get_logger(), "E-STOP engaged: holding positions for %zu joints",
                   params_.joints.size() );
    } else {
      // release E-stop; the pipeline is parked, so the arm holds until a new reference
      RCLCPP_WARN( get_node()->get_logger(),
                   "E-STOP released: holding until a new reference arrives" );
    }
    estop_engaged_.store( estop_active, std::memory_order_relaxed );
    status_event = true;
  }

  if ( estop_active ) {
    // Record the hold from the first read that succeeds — normally the engage cycle
    // itself; during a read outage (the engage must not wait for one) the first valid
    // read after it. Once recorded, no state read is needed to keep holding.
    if ( std::any_of( hold_positions_.begin(), hold_positions_.end(),
                      []( const double p ) { return !std::isfinite( p ); } ) &&
         read_current_positions() ) {
      hold_positions_ = current_positions_;
    }
    pipeline_->invalidate();
    write_position_commands( hold_positions_ );
  } else if ( !read_current_positions() ) {
    // A handle is locked by another thread (async hardware). Skipping the cycle leaves
    // the previous position command in place, which is what re-writing it would do.
    // The safety state is unobservable, so this counts toward the same watchdog as a
    // failed collision-state read below.
    note_state_unobservable( period );
  } else {
    status_event |= run_safety_pipeline( period );
    if ( params_.set_current_limits ) {
      write_current_limits();
    }
    // Manipulability is only ever read by ~/status, so recompute it at that rate (and
    // on any event that publishes) instead of every cycle. It describes the last
    // checked configuration, so it goes stale while checks are bypassed.
    if ( collision_checker_ && ( --manipulability_countdown_ <= 0 || status_event ) ) {
      last_manipulability_ = collision_checker_->computeManipulability();
      manipulability_countdown_ = manipulability_period_;
    }
  }

  // Events are published only once the snapshot has caught up with this cycle, otherwise
  // the message still reports the previous cycle's stall/park state.
  diagnostics_->updateSnapshot( pipeline_.get(), collision_observer_->lastMinDistance(),
                                collision_observer_->lastSafetyZonePairs().size(),
                                last_manipulability_ );
  if ( status_event ) {
    publish_status();
  }
  return controller_interface::return_type::OK;
}

// ===== Helpers =====

void SafetyPositionController::clear_bypass()
{
  safety_bypass_deadline_.store( 0, std::memory_order_relaxed );
  safety_bypass_active_.store( false, std::memory_order_relaxed );
}

bool SafetyPositionController::expire_bypass()
{
  const auto deadline = safety_bypass_deadline_.load( std::memory_order_relaxed );
  if ( deadline == 0 || !safety_bypass_active_.load( std::memory_order_relaxed ) ) {
    return false;
  }
  if ( std::chrono::steady_clock::now().time_since_epoch().count() < deadline ) {
    return false;
  }
  clear_bypass();
  RCLCPP_WARN( get_node()->get_logger(), "Safety bypass expired; safety checks are back on." );
  return true;
}

void SafetyPositionController::note_estop_request( const bool active )
{
  const bool prev = estop_active_.exchange( active, std::memory_order_relaxed );
  if ( active ) {
    estop_engage_pending_.store( true, std::memory_order_relaxed );
  }
  if ( active != prev ) {
    RCLCPP_WARN( get_node()->get_logger(), "E-STOP %s", active ? "ENGAGED" : "DISENGAGED" );
  }
}

void SafetyPositionController::note_state_unobservable( const rclcpp::Duration &period )
{
  // Nominal dt where the reported period is unusable (0 on the first cycle, paused sim
  // clock); get_update_rate() is the same rate the pipeline integrates with.
  const double rate = get_update_rate();
  const double dt = period.seconds() > 0.0 ? period.seconds() : ( rate > 0.0 ? 1.0 / rate : 0.0 );
  state_read_failure_time_ += dt;
  // Only on the edge: invalidate() rebases the command onto the measured position, so
  // repeating it every cycle would turn the hold into "follow the measurement" and let
  // a loaded limb sag away under gravity.
  if ( state_read_failure_time_ >= params_.state_read_timeout && !park_on_recovery_pending_ ) {
    RCLCPP_ERROR( get_node()->get_logger(),
                  "Safety state unobservable for %.2f s; holding. The pipeline is rebased "
                  "and parks until a new reference arrives.",
                  state_read_failure_time_ );
    pipeline_->invalidate();
    park_on_recovery_pending_ = true;
  }
}

bool SafetyPositionController::take_recovery_event()
{
  state_read_failure_time_ = 0.0;
  return std::exchange( park_on_recovery_pending_, false );
}

bool SafetyPositionController::read_current_positions()
{
  // One read per cycle for every joint: the controlled ones drive the pipeline, the
  // rest complete the collision-check configuration. Reading them separately could
  // hand the two consumers different snapshots of the same robot.
  all_positions_.assign( all_joint_names_.size(), std::numeric_limits<double>::quiet_NaN() );
  all_state_valid_ = state_interfaces_.size() >= all_joint_names_.size();
  for ( size_t i = 0; i < all_joint_names_.size() && i < state_interfaces_.size(); ++i ) {
    const auto opt = state_interfaces_[i].get_optional();
    if ( opt.has_value() && std::isfinite( *opt ) ) {
      all_positions_[i] = *opt;
    } else {
      all_state_valid_ = false;
      RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), throttle_logging_msg,
                            "Joint state of '%s' is unreadable (busy or non-finite).",
                            all_joint_names_[i].c_str() );
    }
  }

  // Only the controlled joints gate the cycle; the rest merely make the collision state
  // observable (handled via all_state_valid_).
  for ( size_t i = 0; i < params_.joints.size(); ++i ) {
    const double position = all_positions_[static_cast<size_t>( joint_index_[i] )];
    if ( !std::isfinite( position ) ) {
      return false;
    }
    current_positions_[i] = position;
  }
  return true;
}

void SafetyPositionController::write_position_commands( const std::vector<double> &commands )
{
  for ( size_t i = 0; i < params_.joints.size(); ++i ) {
    if ( std::isfinite( commands[i] ) && !command_interfaces_[i].set_value( commands[i] ) ) {
      RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), throttle_logging_msg,
                            "Position command interface of '%s' is busy; command not written.",
                            params_.joints[i].c_str() );
    }
  }
  diagnostics_->publishJointStateOut( commands );
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
  cfg.hold_unrequested = params_.hold_unrequested_joints;
  cfg.hold_velocity_threshold = params_.hold_unrequested_velocity_threshold;
  cfg.park_resume_threshold = params_.park_resume_reference_threshold;
  cfg.stall_park.stall_timeout = params_.qp_stall_timeout;
  cfg.stall_park.park_timeout = params_.stall_park_timeout;

  // Tunneling check: if one full-speed step can cross the whole braking zone, the damper
  // may be skipped over between two collision checks. Comparing the joint step [rad]
  // against the zone width [m] assumes a ~1 m lever arm, so this is a conservative
  // heuristic, not an exact criterion — hence a warning rather than a hard failure.
  const double zone_width = params_.collision_safety_zone - params_.collision_padding;
  const double max_step = cfg.qp.v_max.maxCoeff() * dt;
  if ( params_.check_self_collisions && max_step >= zone_width ) {
    RCLCPP_WARN( get_node()->get_logger(),
                 "Max per-cycle step (%.4f rad) >= safety zone width (%.4f m): a fast joint on a "
                 "long lever could cross the damper zone between two checks. Increase "
                 "collision_safety_zone (currently %.4f) or the update rate (currently %.0f Hz).",
                 max_step, zone_width, params_.collision_safety_zone, 1.0 / dt );
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

bool SafetyPositionController::run_safety_pipeline( const rclcpp::Duration &period )
{
  const size_t n = params_.joints.size();
  const bool bypass_active = safety_bypass_active_.load( std::memory_order_relaxed );
  const bool collision_checks_active =
      !bypass_active && params_.check_self_collisions && collision_checker_ != nullptr;

  bool status_event = false;
  if ( pipeline_->prepare( reference_interfaces_, current_positions_, bypass_active ) ) {
    RCLCPP_INFO( get_node()->get_logger(), "New reference received — resuming from parked state." );
    status_event = true;
  }

  // ---- Collision observation at the commanded configuration ----
  const auto snapshot =
      collision_observer_->observe( collision_checks_active, all_positions_, all_state_valid_,
                                    pipeline_->commandedPositions(), params_.collision_safety_zone );
  if ( snapshot.collision_started ) {
    const std::string pairs_str = diagnostics_->formatCollisionPairs(
        collision_observer_->lastSafetyZonePairs(), params_.collision_padding,
        collision_observer_->lastMinDistancePairIndex() );
    RCLCPP_WARN( get_node()->get_logger(),
                 "Collision detected (min_dist=%.4f m). Pairs in collision: %s. "
                 "QP holding/pushing out.",
                 collision_observer_->lastMinDistance(), pairs_str.c_str() );
    status_event = true;
  }
  if ( snapshot.observation.checks_active && !snapshot.observation.state_valid ) {
    // The collision state is as unobservable as a failed controlled-joint read: feed
    // the same watchdog so a dead encoder on an uncontrolled joint also parks on
    // recovery instead of jump-starting toward the still-live reference.
    RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), throttle_logging_msg,
                          "Failed to setup collision checking. Braking to a stop." );
    note_state_unobservable( period );
  } else if ( pipeline_->measurementDiverged() ) {
    // A joint is further from its command than the tracking leash can explain, so it
    // left on its own (backdriven, slipping, a re-homed encoder). The collision check
    // runs at the commanded configuration, which therefore no longer describes the
    // robot: same watchdog, so a lasting divergence rebases onto the measured state and
    // parks instead of steering a model the robot has left.
    RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), throttle_logging_msg,
                          "A joint is far from its command; the checked configuration no "
                          "longer describes the robot. Braking to a stop." );
    note_state_unobservable( period );
  } else {
    // Recovered: the pipeline parks on this cycle, which is an event worth publishing.
    status_event |= take_recovery_event();
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
    status_event = true;
  }
  if ( events.stall.parked ) {
    RCLCPP_WARN( get_node()->get_logger(),
                 "Stalled for %.1f s — parking: the current reference is abandoned and the "
                 "limb will hold position (even if the blockage clears) until a new command "
                 "arrives.",
                 pipeline_->stallTime() );
    status_event = true;
  }
  if ( events.stall.resumed ) {
    RCLCPP_INFO( get_node()->get_logger(), "Motion resumed after stall." );
    status_event = true;
  }

  // ---- Write ----
  const auto &qp_cmd = pipeline_->commandedPositions();
  for ( size_t i = 0; i < n; ++i ) { qp_cmd_std_[i] = qp_cmd[static_cast<Eigen::Index>( i )]; }
  write_position_commands( qp_cmd_std_ );

  // RViz markers, colored by whether the motion moves each pair apart. Only after a
  // real check: the visualizer draws the checker's latched state, which would otherwise
  // be redrawn with fresh stamps while the arm moves on (bypass, unobservable state).
  if ( collision_visualizer_ && collision_checker_ && snapshot.observation.checks_active &&
       snapshot.observation.state_valid ) {
    collision_visualizer_->publish( *collision_checker_,
                                    params_.debug_visualize_collisions
                                        ? CollisionVisualizer::Level::FullGeometry
                                        : CollisionVisualizer::Level::LinesOnly,
                                    collision_observer_->directionalInfo( pipeline_->velocity() ),
                                    params_.collision_safety_zone );
  }

  diagnostics_->maybePublishQpDebug( *pipeline_, reference_interfaces_ );
  return status_event;
}

void SafetyPositionController::write_current_limits()
{
  if ( command_interfaces_.size() <= params_.joints.size() ) {
    return;
  }
  const auto &limits = in_compliant_mode_.load( std::memory_order_relaxed )
                           ? compliant_current_limits_
                           : stiff_current_limits_;
  for ( size_t i = 0; i < params_.joints.size(); ++i ) {
    if ( !command_interfaces_[i + params_.joints.size()].set_value( limits[i] ) ) {
      RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), throttle_logging_msg,
                            "Current limit interface of '%s' is busy; limit not written.",
                            params_.joints[i].c_str() );
    }
  }
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
