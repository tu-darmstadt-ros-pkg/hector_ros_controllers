//
// Created by aljoscha-schmidt on 10/20/25.
//
#include "safety_position_controller/collision_checker.hpp"

#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/skew.hpp>

#include "pinocchio/collision/distance.hpp"
#include <cmath>
#include <hpp/fcl/collision_data.h>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/multibody/model.hpp>

CollisionChecker::CollisionChecker( const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
                                    double collision_padding, double collision_cache_epsilon,
                                    bool pub_debug_geometry )
    : node_( node ), collision_padding_( collision_padding ),
      collision_cache_epsilon_( collision_cache_epsilon ), pub_debug_geometry_( pub_debug_geometry )
{
  updateDoDebugVisualization( pub_debug_geometry );
}

bool CollisionChecker::initFromXml( const std::string &urdf_xml, const std::string &srdf_xml,
                                    const std::vector<std::string> &controlled_joints,
                                    bool free_flyer )
{
  try {
    if ( free_flyer )
      pinocchio::urdf::buildModelFromXML( urdf_xml, pinocchio::JointModelFreeFlyer(), model_ );
    else
      pinocchio::urdf::buildModelFromXML( urdf_xml, model_ );

    data_ = pinocchio::Data( model_ );

    std::istringstream urdf_stream( urdf_xml );
    // Build collision geometry from the XML stream.
    pinocchio::urdf::buildGeom( model_, urdf_stream, pinocchio::COLLISION, geom_model_ );
    geom_model_.addAllCollisionPairs();
    if ( !srdf_xml.empty() )
      pinocchio::srdf::removeCollisionPairsFromXML( model_, geom_model_, srdf_xml );

    geom_data_ = pinocchio::GeometryData( geom_model_ );
    q_default_ = pinocchio::neutral( model_ );

    name_to_id_.clear();
    for ( pinocchio::JointIndex jid = 1; jid < model_.joints.size(); ++jid ) {
      name_to_id_[model_.names[jid]] = jid;
    }
    // Filter collision pairs based on controlled joints
    filterCollisionPairs( controlled_joints );

    // Pre-allocate Jacobian workspace matrices
    J1_workspace_ = Eigen::MatrixXd::Zero( 6, model_.nv );
    J2_workspace_ = Eigen::MatrixXd::Zero( 6, model_.nv );

    return true;
  } catch ( const std::exception &e ) {
    RCLCPP_ERROR( node_->get_logger(), "CollisionChecker init failed: %s", e.what() );
    return false;
  }
}

void CollisionChecker::filterCollisionPairs( const std::vector<std::string> &controlled_joints )
{
  // If nothing specified, keep all pairs
  if ( controlled_joints.empty() ) {
    RCLCPP_INFO( node_->get_logger(),
                 "[CollisionChecker] No controlled_joints -> keeping all %zu pairs",
                 geom_model_.collisionPairs.size() );
    return;
  }

  // Build the set of relevant joints: each controlled joint + all its ancestors up to root
  std::unordered_set<pinocchio::JointIndex> relevant;
  relevant.reserve( controlled_joints.size() * 4 );

  for ( const auto &name : controlled_joints ) {
    auto it = name_to_id_.find( name );
    if ( it == name_to_id_.end() ) {
      RCLCPP_WARN( node_->get_logger(),
                   "[CollisionChecker] controlled joint '%s' not found in model (ignored).",
                   name.c_str() );
      continue;
    }
    pinocchio::JointIndex j = it->second;
    // climb to root (universe is 0)
    while ( j != 0 ) {
      if ( relevant.insert( j ).second == false )
        break; // already inserted; ancestor chain above is already covered
      j = model_.parents[j];
    }
  }

  // Filter collision pairs: keep if either object's parentJoint is relevant
  const std::size_t before = geom_model_.collisionPairs.size();
  std::vector<pinocchio::CollisionPair> filtered;
  filtered.reserve( before );

  for ( const auto &cp : geom_model_.collisionPairs ) {
    const auto &go1 = geom_model_.geometryObjects[cp.first];
    const auto &go2 = geom_model_.geometryObjects[cp.second];
    const pinocchio::JointIndex j1 = go1.parentJoint;
    const pinocchio::JointIndex j2 = go2.parentJoint;
    if ( relevant.count( j1 ) || relevant.count( j2 ) ) {
      filtered.push_back( cp );
    }
  }

  geom_model_.collisionPairs.swap( filtered );

  //  re-create GeometryData so requests/results match new pair count
  geom_data_ = pinocchio::GeometryData( geom_model_ );

  const std::size_t after = geom_model_.collisionPairs.size();
  RCLCPP_INFO(
      node_->get_logger(), "[CollisionChecker] Filtered collision pairs: %zu -> %zu (controlled=%zu, relevant joints=%zu)",
      before, after, controlled_joints.size(), relevant.size() );
}

std::vector<std::string> CollisionChecker::getJointNames() const
{
  std::vector<std::string> out;
  out.reserve( !model_.joints.empty() ? model_.joints.size() - 1 : 0 );
  for ( pinocchio::JointIndex jid = 1; jid < model_.joints.size(); ++jid )
    out.push_back( model_.names[jid] );
  return out;
}

CollisionResult
CollisionChecker::checkCollision( const std::unordered_map<std::string, double> &joint_positions,
                                  double safety_zone_threshold )
{

  if ( model_.nq == 0 ) {
    RCLCPP_ERROR( node_->get_logger(), "Model not initialized." );
    return CollisionResult{ true, 0.0, {} };
  }
  // return collision if any position is Nan or Inf
  for ( const auto &[name, position] : joint_positions ) {
    if ( std::isnan( position ) || std::isinf( position ) ) {
      RCLCPP_ERROR(
          node_->get_logger(),
          "Joint position for joint '%s' is NaN or Inf (%.3f). Assuming the robot is in collision.",
          name.c_str(), position );
      return CollisionResult{ true, 0.0, {} };
    }
  }
  // transforms the joint positions into the pinocchio format
  Eigen::VectorXd q = q_default_;
  for ( const auto &[name, position] : joint_positions ) {
    const auto it = name_to_id_.find( name );
    if ( it == name_to_id_.end() ) {
      RCLCPP_WARN_THROTTLE( node_->get_logger(), *node_->get_clock(), 2000,
                            "Unknown joint '%s' (ignored).", name.c_str() );
      continue;
    }
    const pinocchio::JointIndex jid = it->second;
    const int nq_j = model_.joints[jid].nq(); // number of position DoF for this joint
    const int nv_j = model_.joints[jid].nv(); // number of velocity DoF for this joint
    const int iq = model_.idx_qs[jid];        // starting index in q vector
    const double alpha = position;

    if ( nq_j == 1 ) { // e.g prismatic or revolute with 1 DoF
      q[iq] = alpha;
    } else if ( nq_j == 2 && nv_j == 1 ) { // e.g. continuous Joint !!
      // Revolute Continuous Joints represented as unit complex [cos(α), sin(α)]
      const double c = std::cos( alpha );
      const double s = std::sin( alpha );
      q[iq] = c;
      q[iq + 1] = s;
    } else {
      RCLCPP_WARN_THROTTLE( node_->get_logger(), *node_->get_clock(), 2000,
                            "Joint '%s' (nq=%d,nv=%d) not supported; keeping default.",
                            model_.names[jid].c_str(), nq_j, nv_j );
    }
  }

  return checkCollisionQ( q, safety_zone_threshold );
}
CollisionResult CollisionChecker::checkCollisionQ( const Eigen::VectorXd &q,
                                                   double safety_zone_threshold )
{
#ifdef SAFETY_CC_ENABLE_TIMING
  using clock = std::chrono::steady_clock;
  const auto t0 = clock::now();
#endif

  if ( q.size() != model_.nq ) {
    RCLCPP_ERROR( node_->get_logger(), "q size (%ld) != model.nq (%d)", long( q.size() ), model_.nq );
    return CollisionResult{ true, 0.0, {} };
  }

  // check if robot moved since the last check
  if ( q.size() == q_last_.size() &&
       ( q - q_last_ ).cwiseAbs().maxCoeff() < collision_cache_epsilon_ ) {
    // no movement -> no need to recompute distances
    return last_collision_result_;
  }
  q_last_ = q;

  // Kinematics + placements
  pinocchio::forwardKinematics( model_, data_, q );
#ifdef SAFETY_CC_ENABLE_TIMING
  const auto t_fk = clock::now();
#endif
  pinocchio::updateGeometryPlacements( model_, data_, geom_model_, geom_data_ );
#ifdef SAFETY_CC_ENABLE_TIMING
  const auto t_placement = clock::now();
#endif

  // --- Pass 1: distances only (no nearest points) ---
  // When debug visualization is active, compute nearest points for ALL pairs in a single pass
  // instead of two-pass (debug mode is not performance-critical).
  const bool single_pass = pub_debug_geometry_;
  for ( auto &dreq : geom_data_.distanceRequests ) { dreq.enable_nearest_points = single_pass; }
  pinocchio::computeDistances( geom_model_, geom_data_ );

#ifdef SAFETY_CC_ENABLE_TIMING
  const auto t_distance = clock::now();
#endif

  // Find global minimum and identify safety-zone pairs
  double global_min_distance = std::numeric_limits<double>::max();
  bool logged_collision = false;
  bool has_safety_zone_pairs = false;
  std::vector<std::size_t> safety_zone_indices;

  for ( std::size_t k = 0; k < geom_model_.collisionPairs.size(); ++k ) {
    const auto &dres = geom_data_.distanceResults[k];

    if ( dres.min_distance < global_min_distance ) {
      global_min_distance = dres.min_distance;
    }

    if ( safety_zone_threshold > 0.0 && dres.min_distance < safety_zone_threshold ) {
      has_safety_zone_pairs = true;
      safety_zone_indices.push_back( k );
    }

    if ( dres.min_distance <= collision_padding_ && !logged_collision ) {
      logged_collision = true;
      const auto &cp = geom_model_.collisionPairs[k];
      const auto &o1 = geom_model_.geometryObjects[cp.first];
      const auto &o2 = geom_model_.geometryObjects[cp.second];
      RCLCPP_WARN_STREAM_THROTTLE( node_->get_logger(), *node_->get_clock(), 1000,
                                   "Collision (or contact) distance "
                                       << dres.min_distance << " between "
                                       << model_.frames[o1.parentFrame].name << " and "
                                       << model_.frames[o2.parentFrame].name );
    }
  }

  // --- Pass 2: recompute only safety-zone pairs with nearest points (for gradients) ---
  if ( !single_pass && has_safety_zone_pairs ) {
    for ( const std::size_t k : safety_zone_indices ) {
      geom_data_.distanceRequests[k].enable_nearest_points = true;
      geom_data_.distanceResults[k].clear();
      pinocchio::computeDistance( geom_model_, geom_data_, k );
    }
  }

  CollisionResult result;
  result.in_collision = ( global_min_distance <= collision_padding_ );
  result.min_distance = global_min_distance;

  // Compute per-pair distance gradients (lazy: only if pairs actually exist in safety zone)
  if ( has_safety_zone_pairs ) {
    pinocchio::computeJointJacobians( model_, data_ );

#ifdef SAFETY_CC_ENABLE_TIMING
    const auto t_jacobian = clock::now();
#endif

    for ( const std::size_t k : safety_zone_indices ) {
      CollisionResult::PairInfo info;
      info.pair_index = k;
      info.distance = geom_data_.distanceResults[k].min_distance;
      info.gradient = computePairGradient( k );
      result.safety_zone_pairs.push_back( std::move( info ) );
    }

#ifdef SAFETY_CC_ENABLE_TIMING
    const auto t_gradient = clock::now();
    timing_stats_.jacobian_us +=
        static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>( t_jacobian - t_distance ).count() ) /
        1000.0;
    timing_stats_.gradient_us +=
        static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>( t_gradient - t_jacobian ).count() ) /
        1000.0;
#endif
  }

#ifdef SAFETY_CC_ENABLE_TIMING
  const auto t_end = clock::now();
  timing_stats_.fk_us +=
      static_cast<double>( std::chrono::duration_cast<std::chrono::nanoseconds>( t_fk - t0 ).count() ) /
      1000.0;
  timing_stats_.placement_us +=
      static_cast<double>(
          std::chrono::duration_cast<std::chrono::nanoseconds>( t_placement - t_fk ).count() ) /
      1000.0;
  timing_stats_.distance_us +=
      static_cast<double>(
          std::chrono::duration_cast<std::chrono::nanoseconds>( t_distance - t_placement ).count() ) /
      1000.0;
  timing_stats_.total_us +=
      static_cast<double>(
          std::chrono::duration_cast<std::chrono::nanoseconds>( t_end - t0 ).count() ) /
      1000.0;
  timing_stats_.num_safety_zone_pairs += safety_zone_indices.size();
  timing_stats_.count++;
  RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "[CC timing] avg total=%.1f µs (FK=%.1f, placement=%.1f, dist=%.1f, "
      "jac=%.1f, grad=%.1f) pairs=%zu, zone_pairs=%.1f",
      timing_stats_.total_us / timing_stats_.count, timing_stats_.fk_us / timing_stats_.count,
      timing_stats_.placement_us / timing_stats_.count,
      timing_stats_.distance_us / timing_stats_.count,
      timing_stats_.jacobian_us / timing_stats_.count,
      timing_stats_.gradient_us / timing_stats_.count, geom_model_.collisionPairs.size(),
      static_cast<double>( timing_stats_.num_safety_zone_pairs ) / timing_stats_.count );
#endif

  last_collision_result_ = result;
  if ( pub_debug_geometry_ )
    publishMarkers();
  return result;
}

void CollisionChecker::updateDoDebugVisualization( const bool pub_debug_geometry )
{
  pub_debug_geometry_ = pub_debug_geometry;
  if ( pub_debug_geometry_ && !markers_pub_ ) {
    markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/debug_collision_geometry", 1 );
  }
}
void CollisionChecker::updateCollisionPadding( const double collision_padding )
{
  collision_padding_ = collision_padding;
}

void CollisionChecker::updateCollisionCacheEpsilon( const double epsilon )
{
  collision_cache_epsilon_ = epsilon;
}

Eigen::VectorXd CollisionChecker::computePairGradient( std::size_t pair_k )
{
  Eigen::VectorXd grad = Eigen::VectorXd::Zero( model_.nv );

  const auto &dres = geom_data_.distanceResults[pair_k];

  // Validate nearest points
  if ( dres.nearest_points[0].hasNaN() || dres.nearest_points[1].hasNaN() ||
       !dres.nearest_points[0].allFinite() || !dres.nearest_points[1].allFinite() ) {
    return grad; // zero gradient = no directional preference (safe fallback)
  }

  const auto &cp = geom_model_.collisionPairs[pair_k];
  const pinocchio::JointIndex j1 = geom_model_.geometryObjects[cp.first].parentJoint;
  const pinocchio::JointIndex j2 = geom_model_.geometryObjects[cp.second].parentJoint;

  // Get 6×nv Jacobians in LOCAL_WORLD_ALIGNED frame (reuse pre-allocated workspace)
  J1_workspace_.setZero();
  J2_workspace_.setZero();
  pinocchio::getJointJacobian( model_, data_, j1, pinocchio::LOCAL_WORLD_ALIGNED, J1_workspace_ );
  pinocchio::getJointJacobian( model_, data_, j2, pinocchio::LOCAL_WORLD_ALIGNED, J2_workspace_ );

  // Nearest points in world frame
  const Eigen::Vector3d p1 = dres.nearest_points[0];
  const Eigen::Vector3d p2 = dres.nearest_points[1];

  // Offsets from joint origins to nearest points
  const Eigen::Vector3d r1 = p1 - data_.oMi[j1].translation();
  const Eigen::Vector3d r2 = p2 - data_.oMi[j2].translation();

  // Direction vector: from p1 to p2 (positive distance direction)
  const Eigen::Vector3d diff = p2 - p1;
  const double dist_norm = diff.norm();
  if ( dist_norm < 1e-12 ) {
    return grad; // points coincide, gradient undefined
  }
  const Eigen::Vector3d n = diff / dist_norm;

  // Point Jacobians and gradient: dd/dv = n^T * (Jp2 - Jp1)
  // where Jp = J_linear - skew(r) * J_angular
  const Eigen::MatrixXd Jp1 =
      J1_workspace_.topRows( 3 ) - pinocchio::skew( r1 ) * J1_workspace_.bottomRows( 3 );
  const Eigen::MatrixXd Jp2 =
      J2_workspace_.topRows( 3 ) - pinocchio::skew( r2 ) * J2_workspace_.bottomRows( 3 );

  grad = ( n.transpose() * ( Jp2 - Jp1 ) ).transpose();
  return grad;
}

int CollisionChecker::getJointVelocityIndex( const std::string &joint_name ) const
{
  const auto it = name_to_id_.find( joint_name );
  if ( it == name_to_id_.end() )
    return -1;
  return model_.idx_vs[it->second];
}

int CollisionChecker::getNv() const { return model_.nv; }

std::size_t CollisionChecker::getNumCollisionPairs() const
{
  return geom_model_.collisionPairs.size();
}

void CollisionChecker::setDirectionalInfo( const std::vector<double> &derivatives,
                                           double safety_zone_threshold )
{
  viz_directional_derivatives_ = derivatives;
  viz_safety_zone_threshold_ = safety_zone_threshold;
}

void CollisionChecker::publishMarkers() const
{
  if ( !markers_pub_ )
    return;

  visualization_msgs::msg::MarkerArray arr;
  arr.markers.reserve( geom_model_.geometryObjects.size() + geom_model_.collisionPairs.size() + 4 );
  const rclcpp::Time now = node_->now();

  // Build a quick lookup of objects involved in "distance<=0" for coloring
  std::vector<size_t> objects_in_collision;
  auto add_unique = [&]( size_t idx ) {
    if ( std::find( objects_in_collision.begin(), objects_in_collision.end(), idx ) ==
         objects_in_collision.end() )
      objects_in_collision.push_back( idx );
  };
  for ( std::size_t k = 0; k < geom_model_.collisionPairs.size(); ++k ) {
    const auto &cp = geom_model_.collisionPairs[k];
    const auto &dres = geom_data_.distanceResults[k];
    if ( dres.min_distance <= 0.0 ) {
      add_unique( cp.first );
      add_unique( cp.second );
      RCLCPP_WARN(
          node_->get_logger(),
          "Collision detected between objects %zu and %zu (distance=%.6f), names '%s' - '%s'",
          cp.first, cp.second, dres.min_distance, geom_model_.geometryObjects[cp.first].name.c_str(),
          geom_model_.geometryObjects[cp.second].name.c_str() );
    }
  }

  // 1) Geometry markers
  for ( std::size_t i = 0; i < geom_model_.geometryObjects.size(); ++i ) {
    const auto &go = geom_model_.geometryObjects[i];
    const auto &M = geom_data_.oMg[i];

    visualization_msgs::msg::Marker m;
    m.header.frame_id = "base_link";
    m.header.stamp = now;
    m.ns = "collision_geometry";
    m.id = static_cast<int>( i );
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.position.x = M.translation().x();
    m.pose.position.y = M.translation().y();
    m.pose.position.z = M.translation().z();
    Eigen::Quaterniond q( M.rotation() );
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();

    using namespace hpp::fcl;
    const auto *s = go.geometry.get();
    if ( auto sp = dynamic_cast<const Sphere *>( s ) ) {
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.scale.x = m.scale.y = m.scale.z = 2.0 * sp->radius;
    } else if ( auto bx = dynamic_cast<const Box *>( s ) ) {
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.scale.x = bx->halfSide[0] * 2.0;
      m.scale.y = bx->halfSide[1] * 2.0;
      m.scale.z = bx->halfSide[2] * 2.0;
    } else if ( auto cy = dynamic_cast<const Cylinder *>( s ) ) {
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.scale.x = m.scale.y = 2.0 * cy->radius;
      m.scale.z = cy->halfLength * 2.0;
    } else {
      if ( !go.meshPath.empty() ) {
        m.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        m.mesh_resource = go.meshPath;
        m.mesh_use_embedded_materials = true;
        m.scale.x = go.meshScale[0];
        m.scale.y = go.meshScale[1];
        m.scale.z = go.meshScale[2];
      } else {
        m.type = visualization_msgs::msg::Marker::ARROW; // fallback
        m.scale.x = 0.05;
        m.scale.y = 0.01;
        m.scale.z = 0.01;
      }
    }

    const bool coll = std::find( objects_in_collision.begin(), objects_in_collision.end(), i ) !=
                      objects_in_collision.end();
    if ( coll ) {
      m.color.r = 1.0f;
      m.color.g = 0.0f;
      m.color.b = 0.0f;
      m.color.a = 1.0f;
    } else {
      m.color.r = 0.7f;
      m.color.g = 0.7f;
      m.color.b = 0.7f;
      m.color.a = 0.6f;
    }

    m.lifetime = rclcpp::Duration::from_seconds( 0.0 );
    arr.markers.push_back( std::move( m ) );
  }

  // Helper to check if nearest points are valid
  auto valid_nearest_points = []( const hpp::fcl::DistanceResult &dres ) -> bool {
    return !dres.nearest_points[0].hasNaN() && !dres.nearest_points[1].hasNaN() &&
           dres.nearest_points[0].allFinite() && dres.nearest_points[1].allFinite();
  };

  // Helper to create a line marker between nearest points of a pair
  auto make_line_points = []( const hpp::fcl::DistanceResult &dres )
      -> std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point> {
    geometry_msgs::msg::Point pA, pB;
    pA.x = dres.nearest_points[0][0];
    pA.y = dres.nearest_points[0][1];
    pA.z = dres.nearest_points[0][2];
    pB.x = dres.nearest_points[1][0];
    pB.y = dres.nearest_points[1][1];
    pB.z = dres.nearest_points[1][2];
    return { pA, pB };
  };

  // Check if directional info is available for a given pair
  const bool have_dir_info = !viz_directional_derivatives_.empty() &&
                             viz_directional_derivatives_.size() == geom_model_.collisionPairs.size();

  // 2) Distance lines — separated into namespaces by category
  // Initialize LINE_LIST markers for each category
  auto make_line_marker = [&]( const std::string &ns, int id, double thickness ) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "base_link";
    m.header.stamp = now;
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = thickness;
    m.lifetime = rclcpp::Duration::from_seconds( 0.0 );
    // pose defaults to identity
    m.pose.orientation.w = 1.0;
    return m;
  };

  // Lines for pairs outside safety zone (gray, thin)
  visualization_msgs::msg::Marker lines_safe = make_line_marker( "distance_lines_safe", 0, 0.002 );
  // Lines for pairs in safety zone — per-point colors used
  visualization_msgs::msg::Marker lines_zone =
      make_line_marker( "distance_lines_safety_zone", 0, 0.005 );
  // Lines for pairs at/below collision padding (bright red, thick)
  visualization_msgs::msg::Marker lines_coll =
      make_line_marker( "distance_lines_collision", 0, 0.006 );

  lines_safe.points.reserve( geom_model_.collisionPairs.size() * 2 );
  lines_zone.points.reserve( geom_model_.collisionPairs.size() * 2 );
  lines_zone.colors.reserve( geom_model_.collisionPairs.size() * 2 );
  lines_coll.points.reserve( geom_model_.collisionPairs.size() * 2 );

  // Default colors
  std_msgs::msg::ColorRGBA gray;
  gray.r = 0.5f;
  gray.g = 0.5f;
  gray.b = 0.5f;
  gray.a = 0.5f;
  std_msgs::msg::ColorRGBA bright_red;
  bright_red.r = 1.0f;
  bright_red.g = 0.0f;
  bright_red.b = 0.0f;
  bright_red.a = 1.0f;

  lines_safe.color = gray;
  lines_coll.color = bright_red;

  for ( std::size_t k = 0; k < geom_model_.collisionPairs.size(); ++k ) {
    const auto &dres = geom_data_.distanceResults[k];
    if ( !valid_nearest_points( dres ) )
      continue;

    auto [pA, pB] = make_line_points( dres );

    if ( dres.min_distance <= collision_padding_ ) {
      // Collision pair
      lines_coll.points.push_back( pA );
      lines_coll.points.push_back( pB );
    } else if ( viz_safety_zone_threshold_ > 0.0 && dres.min_distance < viz_safety_zone_threshold_ ) {
      // Safety zone pair — color by directional derivative
      std_msgs::msg::ColorRGBA color;
      if ( have_dir_info && !std::isnan( viz_directional_derivatives_[k] ) ) {
        if ( viz_directional_derivatives_[k] >= 0.0 ) {
          // Moving away: green
          color.r = 0.0f;
          color.g = 1.0f;
          color.b = 0.0f;
          color.a = 1.0f;
        } else {
          // Moving closer: red
          color.r = 1.0f;
          color.g = 0.0f;
          color.b = 0.0f;
          color.a = 1.0f;
        }
      } else {
        // No directional info: yellow
        color.r = 1.0f;
        color.g = 0.8f;
        color.b = 0.0f;
        color.a = 0.9f;
      }
      lines_zone.points.push_back( pA );
      lines_zone.colors.push_back( color );
      lines_zone.points.push_back( pB );
      lines_zone.colors.push_back( color );
    } else {
      // Safe pair (outside safety zone)
      lines_safe.points.push_back( pA );
      lines_safe.points.push_back( pB );
    }
  }

  arr.markers.push_back( std::move( lines_safe ) );
  arr.markers.push_back( std::move( lines_zone ) );
  arr.markers.push_back( std::move( lines_coll ) );

  markers_pub_->publish( arr );
}
