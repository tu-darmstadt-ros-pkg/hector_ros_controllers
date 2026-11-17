#include "safety_position_controller/safety_diagnostics.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace safety_position_controller
{

SafetyDiagnostics::SafetyDiagnostics( rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                                      const Params &params, const CollisionChecker *checker )
    : node_( std::move( node ) ), params_( params ), checker_( checker )
{
  config_box_.set(
      StatusConfig{ {}, {}, {}, params_.set_current_limits, params_.check_self_collisions } );

  // Status publisher (latched)
  auto qos_latched = rclcpp::QoS( 1 ).transient_local().reliable();
  status_pub_ =
      node_->create_publisher<hector_ros_controllers_msgs::msg::SafetyPositionControllerStatus>(
          "~/status", qos_latched );

  // Debug joint state publishers (dynamically reconfigurable). Like ~/qp_debug below,
  // the publishers are created once and only the gate is toggled: destroying them from
  // the parameter callback would free them under the control thread's publish call.
  param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>( node_ );
  js_in_.pub = node_->create_publisher<sensor_msgs::msg::JointState>( "~/debug_in_joint_states", 10 );
  js_out_.pub =
      node_->create_publisher<sensor_msgs::msg::JointState>( "~/debug_out_joint_states", 10 );
  debug_js_enabled_.store( params_.publish_debug_joint_states, std::memory_order_relaxed );
  cb_handle_debug_pubs_ = param_subscriber_->add_parameter_callback(
      "publish_debug_joint_states",
      [this]( const rclcpp::Parameter &p ) {
        debug_js_enabled_.store( p.as_bool(), std::memory_order_relaxed );
        const std::lock_guard<std::mutex> lock( timer_mutex_ );
        updateDebugTimer();
        RCLCPP_INFO( node_->get_logger(), "Debug joint state publishing %s",
                     p.as_bool() ? "enabled" : "disabled" );
      },
      node_->get_name() );

  // QP debug introspection publisher (dynamically reconfigurable)
  qp_debug_pub_ =
      node_->create_publisher<hector_ros_controllers_msgs::msg::SafetyQpDebug>( "~/qp_debug", 10 );
  qp_debug_enabled_.store( params_.publish_qp_debug, std::memory_order_relaxed );
  cb_handle_qp_debug_ = param_subscriber_->add_parameter_callback(
      "publish_qp_debug",
      [this]( const rclcpp::Parameter &p ) {
        qp_debug_enabled_.store( p.as_bool(), std::memory_order_relaxed );
        const std::lock_guard<std::mutex> lock( timer_mutex_ );
        updateDebugTimer();
        RCLCPP_INFO( node_->get_logger(), "QP debug publishing %s",
                     p.as_bool() ? "enabled" : "disabled" );
      },
      node_->get_name() );
}

void SafetyDiagnostics::updateSnapshot( const SafetyPipeline *pipeline, const double min_distance,
                                        const std::size_t num_pairs_in_safety_zone,
                                        const double manipulability )
{
  StatusSnapshot snap;
  snap.min_distance = min_distance;
  snap.manipulability = manipulability;
  snap.num_pairs_in_safety_zone = static_cast<uint32_t>( num_pairs_in_safety_zone );
  if ( pipeline ) {
    const SafetyQpResult &qp_result = pipeline->lastResult();
    snap.qp_solved = qp_result.solved;
    snap.qp_braking = qp_result.braking;
    snap.qp_push_out_relaxed = qp_result.push_out_relaxed;
    snap.qp_num_collision_constraints = static_cast<uint32_t>( qp_result.num_collision_constraints );
    snap.qp_solve_time_us = qp_result.solve_time_us;
    snap.stalled = pipeline->stalled();
    snap.parked = pipeline->parked();
  }
  rt_status_box_.set( snap );
}

void SafetyDiagnostics::configure( std::vector<std::string> joint_names,
                                   std::vector<double> stiff_current_limits,
                                   std::vector<double> compliant_current_limits,
                                   const std::size_t max_constraints )
{
  const auto n = static_cast<Eigen::Index>( joint_names.size() );
  const auto m = static_cast<Eigen::Index>( max_constraints );
  {
    // A new activation starts without a capture: what the previous one left must
    // neither be published again nor be read against arrays sized for another limit.
    const std::lock_guard<std::mutex> lock( debug_mutex_ );
    qp_handoff_.reset( [&]( QpDebugSnapshot &qp ) {
      qp = QpDebugSnapshot{};
      for ( auto *v : { &qp.v_des, &qp.v_cmd, &qp.box_lb, &qp.box_ub, &qp.q_cmd } ) {
        v->setZero( n );
      }
      qp.q_ref.assign( joint_names.size(), 0.0 );
      qp.pair_indices.assign( max_constraints, 0 );
      for ( auto *v : { &qp.pair_distances, &qp.pair_rhs, &qp.pair_velocities } ) {
        v->setZero( m );
      }
    } );
    for ( auto *channel : { &js_in_, &js_out_ } ) {
      channel->handoff.reset(
          [&]( JointStateSnapshot &js ) { js.position.assign( joint_names.size(), 0.0 ); } );
    }

    debug_joint_names_ = joint_names;
    pair_names_.clear();
    const std::size_t num_pairs = checker_ ? checker_->getNumCollisionPairs() : 0;
    pair_names_.reserve( num_pairs );
    for ( std::size_t k = 0; k < num_pairs; ++k ) {
      const auto [name_a, name_b] = checker_->getPairNames( k );
      pair_names_.push_back( name_a + "<->" + name_b );
    }
  }

  config_box_.set( StatusConfig{ std::move( joint_names ), std::move( stiff_current_limits ),
                                 std::move( compliant_current_limits ), params_.set_current_limits,
                                 params_.check_self_collisions } );
  // The snapshot outlives a deactivation, so drop it here: the first status of a new
  // activation must not report the previous one's park/stall state.
  rt_status_box_.set( StatusSnapshot{} );
}

void SafetyDiagnostics::publishStatus( const StatusFlags &flags )
{
  if ( !status_pub_ ) {
    return;
  }
  const StatusSnapshot snap = rt_status_box_.get();
  const StatusConfig config = config_box_.get();
  hector_ros_controllers_msgs::msg::SafetyPositionControllerStatus msg;
  msg.header.stamp = node_->now();
  msg.safety_bypass_active = flags.bypass_active;
  msg.compliant_mode = flags.compliant_mode;
  msg.current_limits_enabled = config.current_limits_enabled;
  msg.collision_check_enabled = config.collision_check_enabled;
  msg.estop_engaged = flags.estop_engaged;
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
  if ( config.current_limits_enabled ) {
    msg.joint_names = config.joint_names;
    msg.current_limits =
        flags.compliant_mode ? config.compliant_current_limits : config.stiff_current_limits;
  }

  status_pub_->publish( msg );
}

void SafetyDiagnostics::captureQpDebug( const rclcpp::Time &stamp, const SafetyPipeline &pipeline,
                                        const std::vector<double> &processed_reference )
{
  QpDebugSnapshot &snap = qp_handoff_.staging();
  const auto n = static_cast<std::size_t>( snap.v_des.size() );
  if ( !qp_debug_enabled_.load( std::memory_order_relaxed ) || !qp_debug_pub_ ||
       pipeline.velocity().size() != snap.v_des.size() || processed_reference.size() < n ) {
    return;
  }
  const SafetyQpResult &qp_result = pipeline.lastResult();

  snap.stamp = stamp;
  snap.v_des = pipeline.qpInput().v_des;
  snap.v_cmd = pipeline.velocity();
  snap.box_lb = pipeline.limiter().lastBoxLower();
  snap.box_ub = pipeline.limiter().lastBoxUpper();
  snap.q_cmd = pipeline.commandedPositions();
  std::copy_n( processed_reference.begin(), n, snap.q_ref.begin() );

  const auto &pair_indices = pipeline.constraintPairIndices();
  snap.num_pairs = std::min( { static_cast<std::size_t>( qp_result.num_collision_constraints ),
                               pair_indices.size(), snap.pair_indices.size() } );
  const auto num_pairs = static_cast<Eigen::Index>( snap.num_pairs );
  std::copy_n( pair_indices.begin(), snap.num_pairs, snap.pair_indices.begin() );
  snap.pair_distances.head( num_pairs ) = pipeline.qpInput().collisions.distances().head( num_pairs );
  snap.pair_rhs.head( num_pairs ) = qp_result.collision_rhs.head( num_pairs );
  snap.pair_velocities.head( num_pairs ) = qp_result.collision_velocity.head( num_pairs );

  snap.solved = qp_result.solved;
  snap.braking = qp_result.braking;
  snap.push_out_relaxed = qp_result.push_out_relaxed;
  snap.bounds_conflict = qp_result.bounds_conflict;
  snap.solve_time_us = qp_result.solve_time_us;
  snap.iterations = qp_result.iterations;
  qp_handoff_.commit();
}

void SafetyDiagnostics::captureJointStateIn( const rclcpp::Time &stamp,
                                             const std::vector<double> &positions )
{ captureJointState( js_in_, stamp, positions ); }

void SafetyDiagnostics::captureJointStateOut( const rclcpp::Time &stamp,
                                              const std::vector<double> &positions )
{ captureJointState( js_out_, stamp, positions ); }

void SafetyDiagnostics::captureJointState( JointStateChannel &channel, const rclcpp::Time &stamp,
                                           const std::vector<double> &positions )
{
  JointStateSnapshot &snap = channel.handoff.staging();
  if ( !debug_js_enabled_.load( std::memory_order_relaxed ) ||
       positions.size() != snap.position.size() ) {
    return;
  }
  snap.stamp = stamp;
  std::copy( positions.begin(), positions.end(), snap.position.begin() );
  channel.handoff.commit();
}

void SafetyDiagnostics::startDebugPublishing( const double period,
                                              std::shared_ptr<CollisionVisualizer> visualizer )
{
  const std::lock_guard<std::mutex> lock( timer_mutex_ );
  debug_visualizer_ = std::move( visualizer );
  debug_timer_ = node_->create_wall_timer( std::chrono::duration<double>( period ),
                                           [this]() { publishDebug(); } );
  updateDebugTimer();
}

void SafetyDiagnostics::stopDebugPublishing()
{
  const std::lock_guard<std::mutex> lock( timer_mutex_ );
  debug_timer_.reset();
  debug_visualizer_.reset();
}

void SafetyDiagnostics::updateDebugTimer()
{
  if ( !debug_timer_ ) {
    return;
  }
  const bool enabled = debug_visualizer_ || qp_debug_enabled_.load( std::memory_order_relaxed ) ||
                       debug_js_enabled_.load( std::memory_order_relaxed );
  if ( !enabled ) {
    debug_timer_->cancel();
  } else if ( debug_timer_->is_canceled() ) {
    debug_timer_->reset();
  }
}

void SafetyDiagnostics::publishDebug()
{
  std::shared_ptr<CollisionVisualizer> visualizer;
  {
    const std::lock_guard<std::mutex> lock( timer_mutex_ );
    visualizer = debug_visualizer_;
  }
  if ( visualizer ) {
    visualizer->publishPending();
  }

  // Built under the lock, published outside it: a slow subscriber must not hold up an
  // activation waiting in configure().
  std::optional<sensor_msgs::msg::JointState> js_in, js_out;
  std::optional<hector_ros_controllers_msgs::msg::SafetyQpDebug> qp_debug;
  {
    const std::lock_guard<std::mutex> lock( debug_mutex_ );
    if ( debug_js_enabled_.load( std::memory_order_relaxed ) ) {
      js_in = buildJointState( js_in_ );
      js_out = buildJointState( js_out_ );
    }
    if ( qp_debug_enabled_.load( std::memory_order_relaxed ) ) {
      qp_debug = buildQpDebug();
    }
  }
  if ( js_in && js_in_.pub ) {
    js_in_.pub->publish( *js_in );
  }
  if ( js_out && js_out_.pub ) {
    js_out_.pub->publish( *js_out );
  }
  if ( qp_debug && qp_debug_pub_ ) {
    qp_debug_pub_->publish( *qp_debug );
  }
}

std::optional<hector_ros_controllers_msgs::msg::SafetyQpDebug> SafetyDiagnostics::buildQpDebug()
{
  const QpDebugSnapshot *pending = qp_handoff_.take();
  if ( !pending ) {
    return std::nullopt;
  }
  const QpDebugSnapshot &snap = *pending;

  hector_ros_controllers_msgs::msg::SafetyQpDebug msg;
  msg.header.stamp = snap.stamp;
  msg.joint_names = debug_joint_names_;
  msg.v_des.assign( snap.v_des.begin(), snap.v_des.end() );
  msg.v_cmd.assign( snap.v_cmd.begin(), snap.v_cmd.end() );
  msg.box_lb.assign( snap.box_lb.begin(), snap.box_lb.end() );
  msg.box_ub.assign( snap.box_ub.begin(), snap.box_ub.end() );
  msg.q_cmd.assign( snap.q_cmd.begin(), snap.q_cmd.end() );
  msg.q_ref = snap.q_ref;

  const auto num_pairs = static_cast<Eigen::Index>( snap.num_pairs );
  msg.pair_names.reserve( snap.num_pairs );
  for ( std::size_t k = 0; k < snap.num_pairs; ++k ) {
    const std::size_t pair_index = snap.pair_indices[k];
    msg.pair_names.push_back( pair_index < pair_names_.size() ? pair_names_[pair_index] : "<->" );
  }
  msg.pair_distances.assign( snap.pair_distances.begin(), snap.pair_distances.begin() + num_pairs );
  msg.pair_rhs.assign( snap.pair_rhs.begin(), snap.pair_rhs.begin() + num_pairs );
  msg.pair_velocities.assign( snap.pair_velocities.begin(), snap.pair_velocities.begin() + num_pairs );

  msg.solved = snap.solved;
  msg.braking = snap.braking;
  msg.push_out_relaxed = snap.push_out_relaxed;
  msg.bounds_conflict = snap.bounds_conflict;
  msg.solve_time_us = snap.solve_time_us;
  msg.iterations = snap.iterations;
  return msg;
}

std::optional<sensor_msgs::msg::JointState>
SafetyDiagnostics::buildJointState( JointStateChannel &channel )
{
  const JointStateSnapshot *snap = channel.handoff.take();
  if ( !snap ) {
    return std::nullopt;
  }
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = snap->stamp;
  msg.name = debug_joint_names_;
  msg.position = snap->position;
  return msg;
}

std::string SafetyDiagnostics::formatBlockedDirections( const SafetyPipeline &pipeline ) const
{
  // For each joint the reference wants to move but that is not moving, name the
  // constraint that most strongly opposes the desired direction (normal component
  // against the motion). Gives an immediate answer to "why is joint X stuck".
  std::ostringstream oss;
  const SafetyQpInput &qp_input = pipeline.qpInput();
  const auto &pair_indices = pipeline.constraintPairIndices();
  const double thr = params_.qp_stall_velocity_threshold;
  for ( size_t i = 0; i < params_.joints.size(); ++i ) {
    const auto idx = static_cast<Eigen::Index>( i );
    const double v_des = qp_input.v_des[idx];
    if ( std::abs( v_des ) <= thr || std::abs( pipeline.velocity()[idx] ) > thr ) {
      continue;
    }
    const double dir = v_des > 0.0 ? 1.0 : -1.0;
    // Most opposing constraint: largest -(normal_i * dir)
    double worst_opposition = 0.0;
    const auto num_constraints = static_cast<size_t>( qp_input.collisions.size() );
    size_t worst_k = num_constraints;
    for ( size_t k = 0; k < num_constraints; ++k ) {
      const double opposition =
          -qp_input.collisions.normal( static_cast<Eigen::Index>( k ) )[idx] * dir;
      if ( opposition > worst_opposition ) {
        worst_opposition = opposition;
        worst_k = k;
      }
    }
    oss << " " << params_.joints[i] << "[" << ( dir > 0.0 ? "+" : "-" ) << "]: ";
    if ( worst_k < num_constraints && worst_k < pair_indices.size() && checker_ ) {
      const auto [name_a, name_b] = checker_->getPairNames( pair_indices[worst_k] );
      oss << "blocked by '" << name_a << "<->" << name_b << "' (d=" << std::fixed
          << std::setprecision( 4 )
          << qp_input.collisions.distance( static_cast<Eigen::Index>( worst_k ) )
          << " m, g_i=" << std::setprecision( 3 ) << -worst_opposition * dir << ");";
    } else {
      oss << "no opposing collision constraint (velocity/position bounds?);";
    }
  }
  const std::string s = oss.str();
  return s.empty() ? std::string( " No blocked joints identified." )
                   : std::string( " Blocked directions:" ) + s;
}

std::string SafetyDiagnostics::formatCollisionPairs( const std::vector<CollisionResult::PairInfo> &pairs,
                                                     const double max_distance,
                                                     const std::size_t fallback_pair_index ) const
{
  if ( !checker_ )
    return "unknown";

  std::ostringstream oss;
  bool first = true;
  for ( const auto &pi : pairs ) {
    if ( pi.distance > max_distance )
      continue;
    const auto [name_a, name_b] = checker_->getPairNames( pi.pair_index );
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

  // Fallback: no pair below max_distance — name the closest pair instead.
  const auto [name_a, name_b] = checker_->getPairNames( fallback_pair_index );
  if ( name_a.empty() || name_b.empty() )
    return "unknown";
  std::ostringstream fallback;
  fallback << "'" << name_a << "' <-> '" << name_b << "'";
  return fallback.str();
}

} // namespace safety_position_controller
