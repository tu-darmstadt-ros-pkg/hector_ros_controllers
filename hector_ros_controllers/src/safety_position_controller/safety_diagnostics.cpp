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
  // Status publisher (latched)
  auto qos_latched = rclcpp::QoS( 1 ).transient_local().reliable();
  status_pub_ =
      node_->create_publisher<hector_ros_controllers_msgs::msg::SafetyPositionControllerStatus>(
          "~/status", qos_latched );

  // Debug joint state publishers (dynamically reconfigurable). Like ~/qp_debug below,
  // the publishers are created once and only the gate is toggled: destroying them from
  // the parameter callback would free them under the control thread's publish call.
  param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>( node_ );
  debug_in_js_pub_ =
      node_->create_publisher<sensor_msgs::msg::JointState>( "~/debug_in_joint_states", 10 );
  debug_out_js_pub_ =
      node_->create_publisher<sensor_msgs::msg::JointState>( "~/debug_out_joint_states", 10 );
  debug_js_enabled_.store( params_.publish_debug_joint_states, std::memory_order_relaxed );
  cb_handle_debug_pubs_ = param_subscriber_->add_parameter_callback(
      "publish_debug_joint_states",
      [this]( const rclcpp::Parameter &p ) {
        debug_js_enabled_.store( p.as_bool(), std::memory_order_relaxed );
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
                                   std::vector<double> compliant_current_limits )
{
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

void SafetyDiagnostics::maybePublishQpDebug( const SafetyPipeline &pipeline,
                                             const std::vector<double> &processed_reference )
{
  if ( !qp_debug_enabled_.load( std::memory_order_relaxed ) || !qp_debug_pub_ ) {
    return;
  }
  const size_t n = params_.joints.size();
  const SafetyQpInput &qp_input = pipeline.qpInput();
  const SafetyQpResult &qp_result = pipeline.lastResult();

  hector_ros_controllers_msgs::msg::SafetyQpDebug msg;
  msg.header.stamp = node_->now();
  msg.joint_names = params_.joints;
  msg.v_des.resize( n );
  msg.v_cmd.resize( n );
  msg.box_lb.resize( n );
  msg.box_ub.resize( n );
  msg.q_cmd.resize( n );
  msg.q_ref.resize( n );
  for ( size_t i = 0; i < n; ++i ) {
    const auto idx = static_cast<Eigen::Index>( i );
    msg.v_des[i] = qp_input.v_des[idx];
    msg.v_cmd[i] = pipeline.velocity()[idx];
    msg.box_lb[i] = pipeline.limiter().lastBoxLower()[idx];
    msg.box_ub[i] = pipeline.limiter().lastBoxUpper()[idx];
    msg.q_cmd[i] = pipeline.commandedPositions()[idx];
    msg.q_ref[i] = processed_reference[i];
  }

  const auto num_cc = static_cast<size_t>( qp_result.num_collision_constraints );
  const auto &pair_indices = pipeline.constraintPairIndices();
  const size_t num_pairs = std::min( num_cc, pair_indices.size() );
  msg.pair_names.reserve( num_pairs );
  for ( size_t k = 0; k < num_pairs; ++k ) {
    const auto [name_a, name_b] = checker_ ? checker_->getPairNames( pair_indices[k] )
                                           : std::pair<std::string, std::string>{};
    msg.pair_names.push_back( name_a + "<->" + name_b );
  }
  msg.pair_distances.resize( num_pairs );
  msg.pair_rhs.resize( num_pairs );
  msg.pair_velocities.resize( num_pairs );
  for ( size_t k = 0; k < num_pairs; ++k ) {
    const auto idx = static_cast<Eigen::Index>( k );
    msg.pair_distances[k] = qp_input.collisions[k].distance;
    msg.pair_rhs[k] = ( idx < qp_result.collision_rhs.size() )
                          ? qp_result.collision_rhs[idx]
                          : std::numeric_limits<double>::quiet_NaN();
    msg.pair_velocities[k] = ( idx < qp_result.collision_velocity.size() )
                                 ? qp_result.collision_velocity[idx]
                                 : std::numeric_limits<double>::quiet_NaN();
  }

  msg.solved = qp_result.solved;
  msg.braking = qp_result.braking;
  msg.push_out_relaxed = qp_result.push_out_relaxed;
  msg.bounds_conflict = qp_result.bounds_conflict;
  msg.solve_time_us = qp_result.solve_time_us;
  msg.iterations = qp_result.iterations;

  qp_debug_pub_->publish( msg );
}

void SafetyDiagnostics::publishJointStateIn( const std::vector<double> &positions )
{
  if ( !debug_js_enabled_.load( std::memory_order_relaxed ) ) {
    return;
  }
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_->now();
  msg.name = params_.joints;
  msg.position = positions;
  debug_in_js_pub_->publish( msg );
}

void SafetyDiagnostics::publishJointStateOut( const std::vector<double> &positions )
{
  if ( !debug_js_enabled_.load( std::memory_order_relaxed ) ) {
    return;
  }
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_->now();
  msg.name = params_.joints;
  msg.position = positions;
  debug_out_js_pub_->publish( msg );
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
    size_t worst_k = qp_input.collisions.size();
    for ( size_t k = 0; k < qp_input.collisions.size(); ++k ) {
      const double opposition = -qp_input.collisions[k].normal[idx] * dir;
      if ( opposition > worst_opposition ) {
        worst_opposition = opposition;
        worst_k = k;
      }
    }
    oss << " " << params_.joints[i] << "[" << ( dir > 0.0 ? "+" : "-" ) << "]: ";
    if ( worst_k < qp_input.collisions.size() && worst_k < pair_indices.size() && checker_ ) {
      const auto [name_a, name_b] = checker_->getPairNames( pair_indices[worst_k] );
      oss << "blocked by '" << name_a << "<->" << name_b << "' (d=" << std::fixed
          << std::setprecision( 4 ) << qp_input.collisions[worst_k].distance
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
