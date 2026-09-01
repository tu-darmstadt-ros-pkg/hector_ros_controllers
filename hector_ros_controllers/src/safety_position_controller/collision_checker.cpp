//
// Created by aljoscha-schmidt on 10/20/25.
//
#include "safety_position_controller/collision_checker.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/skew.hpp>

#include "pinocchio/collision/distance.hpp"
#include <cmath>
#include <coal/collision_data.h>
#include <coal/distance.h>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/multibody/model.hpp>

namespace
{
/// Custom coal broadphase distance callback that collects all safety-zone pairs
/// and tracks the global minimum distance. Follows the same pattern as
/// pinocchio::CollisionCallBackDefault (broadphase-callbacks.hpp:90-96).
struct SafetyZoneDistanceCallback : coal::DistanceCallBackBase {
  // Inputs (set before each broadphase scan)
  const pinocchio::GeometryModel *geom_model_ptr{ nullptr };
  pinocchio::GeometryData *geom_data_ptr{ nullptr };
  double safety_zone_threshold{ 0.0 };
  bool compute_nearest_points{ false }; ///< true in single-pass (debug_viz) mode

  // Outputs (filled during traversal)
  double global_min_distance{ std::numeric_limits<double>::max() };
  std::size_t min_distance_pair{ 0 };
  std::vector<std::size_t> safety_zone_indices;
  std::vector<std::size_t> visited_indices; ///< pairs whose distance (and nearest points) were computed

  void init() override
  {
    global_min_distance = std::numeric_limits<double>::max();
    min_distance_pair = 0;
    safety_zone_indices.clear();
    visited_indices.clear();
  }

  bool distance( coal::CollisionObject *o1, coal::CollisionObject *o2, coal::CoalScalar &dist ) override
  {
    // Cast to pinocchio::CollisionObject to get geometry indices
    // (safe: pinocchio creates these objects in BroadPhaseManagerTpl::init)
    auto &co1 = reinterpret_cast<pinocchio::CollisionObject &>( *o1 );
    auto &co2 = reinterpret_cast<pinocchio::CollisionObject &>( *o2 );

    auto go1 = static_cast<Eigen::DenseIndex>( co1.geometryObjectIndex );
    auto go2 = static_cast<Eigen::DenseIndex>( co2.geometryObjectIndex );

    // collisionPairMapping is upper triangular: needs go1 < go2. The broadphase hands
    // objects in TREE TRAVERSAL order, which for ~half the pairs is reversed relative to
    // the stored collision pair (first, second).
    bool swapped = false;
    if ( go1 > go2 ) {
      std::swap( go1, go2 );
      swapped = true;
    }

    // Look up collision pair index (-1 if not a tracked pair)
    const int pair_index = geom_model_ptr->collisionPairMapping( go1, go2 );
    if ( pair_index < 0 )
      return false; // not a tracked pair, skip

    const auto k = static_cast<std::size_t>( pair_index );

    // Check if pair is active
    if ( !geom_data_ptr->activeCollisionPairs[k] )
      return false;

    // Run narrow-phase distance via coal
    auto &dreq = geom_data_ptr->distanceRequests[k];
    auto &dres = geom_data_ptr->distanceResults[k];
    dreq.enable_nearest_points = compute_nearest_points;
    dres.clear();

    coal::distance( o1, o2, dreq, dres );
    if ( compute_nearest_points && swapped ) {
      // Restore pair-canonical order: nearest_points[0] must belong to cp.first,
      // otherwise gradients computed from the witness points are exactly negated.
      std::swap( dres.nearest_points[0], dres.nearest_points[1] );
      dres.normal = -dres.normal;
    }
    const double d = dres.min_distance;
    if ( compute_nearest_points )
      visited_indices.push_back( k );

    // Update outputs
    if ( d < global_min_distance ) {
      global_min_distance = d;
      min_distance_pair = k;
    }

    if ( safety_zone_threshold > 0.0 && d < safety_zone_threshold ) {
      safety_zone_indices.push_back( k );
    }

    // Set the AABB pruning bound: the tree skips subtrees whose AABB lower
    // bound exceeds dist. We need both the global minimum AND all safety zone
    // pairs, so the bound must be max(global_min, safety_zone_threshold).
    dist = ( safety_zone_threshold > 0.0 ) ? std::max( global_min_distance, safety_zone_threshold )
                                           : global_min_distance;

    return false; // never stop early — we need ALL safety zone pairs
  }
};
} // namespace

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

    // Root link = first BODY frame that is not "universe"; used as the marker frame_id.
    // Keeps the "base_link" default if none is found.
    for ( const auto &frame : model_.frames ) {
      if ( frame.type == pinocchio::FrameType::BODY && frame.name != "universe" ) {
        root_frame_ = frame.name;
        break;
      }
    }
    RCLCPP_INFO( node_->get_logger(), "[CollisionChecker] Using root frame '%s' for markers.",
                 root_frame_.c_str() );

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
    nearest_points_fresh_.assign( geom_model_.collisionPairs.size(), false );

    // Pre-allocate Jacobian workspace matrices
    J1_workspace_ = Eigen::MatrixXd::Zero( 6, model_.nv );
    J2_workspace_ = Eigen::MatrixXd::Zero( 6, model_.nv );

    // Always create broadphase manager (cheap); use_broadphase_ controls which path is taken
    broadphase_manager_ = std::make_unique<BroadPhaseManager>( &model_, &geom_model_, &geom_data_ );

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

  // Keep a pair iff the tree path between its two parent joints crosses a controlled
  // joint — only then can this controller change the pair's distance. Equivalent test
  // (chains nest): the DEEPEST CONTROLLED ANCESTOR (0 = none) of the two joints differs.
  // Keeps geometry behind passive joints on the controlled chain (gripper fingers ride
  // on the arm); drops zero-gradient pairs (finger<->finger, chassis<->flipper).
  std::unordered_set<pinocchio::JointIndex> controlled_ids;
  controlled_ids.reserve( controlled_joints.size() );
  for ( const auto &name : controlled_joints ) {
    auto it = name_to_id_.find( name );
    if ( it == name_to_id_.end() ) {
      RCLCPP_WARN( node_->get_logger(),
                   "[CollisionChecker] controlled joint '%s' not found in model (ignored).",
                   name.c_str() );
      continue;
    }
    controlled_ids.insert( it->second );
  }

  // Deepest controlled ancestor per joint (0 = none / universe)
  std::vector<pinocchio::JointIndex> deepest_controlled( model_.joints.size(), 0 );
  for ( pinocchio::JointIndex jid = 1; jid < model_.joints.size(); ++jid ) {
    for ( pinocchio::JointIndex j = jid; j != 0; j = model_.parents[j] ) {
      if ( controlled_ids.count( j ) ) {
        deepest_controlled[jid] = j;
        break;
      }
    }
  }

  const std::size_t before = geom_model_.collisionPairs.size();
  std::vector<pinocchio::CollisionPair> filtered;
  filtered.reserve( before );

  for ( const auto &cp : geom_model_.collisionPairs ) {
    const pinocchio::JointIndex j1 = geom_model_.geometryObjects[cp.first].parentJoint;
    const pinocchio::JointIndex j2 = geom_model_.geometryObjects[cp.second].parentJoint;
    if ( deepest_controlled[j1] != deepest_controlled[j2] ) {
      filtered.push_back( cp );
    }
  }

  geom_model_.collisionPairs.swap( filtered );

  // Rebuild collisionPairMapping matrix to match new pair indices
  geom_model_.collisionPairMapping.setConstant( -1 );
  for ( std::size_t k = 0; k < geom_model_.collisionPairs.size(); ++k ) {
    const auto &cp = geom_model_.collisionPairs[k];
    geom_model_.collisionPairMapping( static_cast<Eigen::DenseIndex>( cp.first ),
                                      static_cast<Eigen::DenseIndex>( cp.second ) ) =
        static_cast<int>( k );
  }

  //  re-create GeometryData so requests/results match new pair count
  geom_data_ = pinocchio::GeometryData( geom_model_ );

  const std::size_t after = geom_model_.collisionPairs.size();
  RCLCPP_INFO(
      node_->get_logger(),
      "[CollisionChecker] Filtered collision pairs: %zu -> %zu (kept = pairs whose distance the "
      "%zu controlled joints can influence)",
      before, after, controlled_joints.size() );
}

std::vector<std::string> CollisionChecker::getJointNames() const
{
  std::vector<std::string> out;
  out.reserve( !model_.joints.empty() ? model_.joints.size() - 1 : 0 );
  for ( pinocchio::JointIndex jid = 1; jid < model_.joints.size(); ++jid )
    out.push_back( model_.names[jid] );
  return out;
}

const CollisionResult &CollisionChecker::unsafeResult()
{
  last_collision_result_ = CollisionResult{};
  last_collision_result_.in_collision = true;
  last_collision_result_.min_distance = 0.0;
  return last_collision_result_;
}

const CollisionResult &
CollisionChecker::checkCollision( const std::unordered_map<std::string, double> &joint_positions )
{
  if ( model_.nq == 0 ) {
    RCLCPP_ERROR( node_->get_logger(), "Model not initialized." );
    return unsafeResult();
  }
  for ( const auto &[name, position] : joint_positions ) {
    if ( !std::isfinite( position ) ) {
      RCLCPP_ERROR( node_->get_logger(),
                    "Joint position for joint '%s' is not finite (%.3f). Assuming the robot is "
                    "in collision.",
                    name.c_str(), position );
      return unsafeResult();
    }
  }
  return checkCollisionQ( buildConfiguration( joint_positions ) );
}

Eigen::VectorXd
CollisionChecker::buildConfiguration( const std::unordered_map<std::string, double> &joint_positions )
{
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
  return q;
}
const CollisionResult &CollisionChecker::checkCollisionQ( const Eigen::VectorXd &q )
{
  const double safety_zone_threshold = safety_zone_threshold_;
#ifdef SAFETY_CC_ENABLE_TIMING
  using clock = std::chrono::steady_clock;
  const auto t0 = clock::now();
#endif

  if ( q.size() != model_.nq ) {
    RCLCPP_ERROR( node_->get_logger(), "q size (%ld) != model.nq (%d)", long( q.size() ), model_.nq );
    return unsafeResult();
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

  const bool single_pass = pub_debug_geometry_;

  if ( single_pass ) {
    // Only the debug-viz path uses freshness; reset it just there.
    std::fill( nearest_points_fresh_.begin(), nearest_points_fresh_.end(), false );
  }

  double global_min_distance = std::numeric_limits<double>::max();
  std::size_t min_distance_pair = 0;
  bool has_safety_zone_pairs = false;
  std::vector<std::size_t> safety_zone_indices;

  if ( use_broadphase_ && broadphase_manager_ ) {
    // --- Broadphase path: AABB-tree pruned distance scan ---
    broadphase_manager_->update( false ); // sync transforms from oMg, rebuild AABB tree

    SafetyZoneDistanceCallback callback;
    callback.geom_model_ptr = &geom_model_;
    callback.geom_data_ptr = &geom_data_;
    callback.safety_zone_threshold = safety_zone_threshold;
    callback.compute_nearest_points = single_pass;
    callback.init();

    broadphase_manager_->getManager().distance( &callback );

    global_min_distance = callback.global_min_distance;
    min_distance_pair = callback.min_distance_pair;
    safety_zone_indices = std::move( callback.safety_zone_indices );
    has_safety_zone_pairs = !safety_zone_indices.empty();

    if ( single_pass ) {
      // Only un-pruned (visited) pairs have fresh nearest points.
      for ( const std::size_t k : callback.visited_indices ) { nearest_points_fresh_[k] = true; }
    } else if ( has_safety_zone_pairs ) {
      // Pass 2: recompute safety-zone pairs with nearest points (for gradients)
      for ( const std::size_t k : safety_zone_indices ) {
        geom_data_.distanceRequests[k].enable_nearest_points = true;
        geom_data_.distanceResults[k].clear();
        pinocchio::computeDistance( geom_model_, geom_data_, k );
      }
    }
  } else {
    // --- Brute-force path: compute distances for ALL pairs ---
    for ( auto &dreq : geom_data_.distanceRequests ) { dreq.enable_nearest_points = single_pass; }
    pinocchio::computeDistances( geom_model_, geom_data_ );

    for ( std::size_t k = 0; k < geom_model_.collisionPairs.size(); ++k ) {
      const auto &dres = geom_data_.distanceResults[k];

      if ( dres.min_distance < global_min_distance ) {
        global_min_distance = dres.min_distance;
        min_distance_pair = k;
      }

      if ( safety_zone_threshold > 0.0 && dres.min_distance < safety_zone_threshold ) {
        has_safety_zone_pairs = true;
        safety_zone_indices.push_back( k );
      }
    }

    if ( single_pass ) {
      // Brute force recomputes every pair, so all are fresh.
      std::fill( nearest_points_fresh_.begin(), nearest_points_fresh_.end(), true );
    } else if ( has_safety_zone_pairs ) {
      // Pass 2: recompute only safety-zone pairs with nearest points (for gradients)
      for ( const std::size_t k : safety_zone_indices ) {
        geom_data_.distanceRequests[k].enable_nearest_points = true;
        geom_data_.distanceResults[k].clear();
        pinocchio::computeDistance( geom_model_, geom_data_, k );
      }
    }
  }

#ifdef SAFETY_CC_ENABLE_TIMING
  const auto t_distance = clock::now();
#endif

  CollisionResult result;
  result.in_collision = ( global_min_distance <= collision_padding_ );
  result.min_distance = global_min_distance;
  if ( global_min_distance != std::numeric_limits<double>::max() ) {
    result.min_distance_pair_index = min_distance_pair;
  }

  // Compute per-pair distance gradients (lazy: only if pairs actually exist in safety zone)
  if ( has_safety_zone_pairs ) {
    // Sort by distance ascending; cap to the closest max_safety_zone_pairs_ (0 = unlimited)
    // so downstream constraint building and gradient computation stay bounded.
    std::sort( safety_zone_indices.begin(), safety_zone_indices.end(),
               [this]( const std::size_t a, const std::size_t b ) {
                 return geom_data_.distanceResults[a].min_distance <
                        geom_data_.distanceResults[b].min_distance;
               } );
    if ( max_safety_zone_pairs_ > 0 && safety_zone_indices.size() > max_safety_zone_pairs_ ) {
      // Over budget: keep the closest pair of each DISTINCT link pair first, then refill
      // with the closest duplicates — near-duplicates of one contact must not evict a
      // different (e.g. approaching) contact.
      std::vector<std::size_t> primaries, duplicates;
      primaries.reserve( safety_zone_indices.size() );
      std::vector<std::pair<pinocchio::FrameIndex, pinocchio::FrameIndex>> seen_links;
      seen_links.reserve( safety_zone_indices.size() );
      for ( const std::size_t k : safety_zone_indices ) {
        const auto &cp = geom_model_.collisionPairs[k];
        const auto fa = geom_model_.geometryObjects[cp.first].parentFrame;
        const auto fb = geom_model_.geometryObjects[cp.second].parentFrame;
        const std::pair<pinocchio::FrameIndex, pinocchio::FrameIndex> key{ std::min( fa, fb ),
                                                                           std::max( fa, fb ) };
        if ( std::find( seen_links.begin(), seen_links.end(), key ) == seen_links.end() ) {
          seen_links.push_back( key );
          primaries.push_back( k );
        } else {
          duplicates.push_back( k );
        }
      }
      const std::size_t num_link_pairs = primaries.size();
      for ( const std::size_t k : duplicates ) {
        if ( primaries.size() >= max_safety_zone_pairs_ ) {
          break;
        }
        primaries.push_back( k );
      }
      if ( primaries.size() > max_safety_zone_pairs_ ) {
        primaries.resize( max_safety_zone_pairs_ );
      }
      // Restore the sorted-by-distance contract after the primary/duplicate split
      std::sort( primaries.begin(), primaries.end(),
                 [this]( const std::size_t a, const std::size_t b ) {
                   return geom_data_.distanceResults[a].min_distance <
                          geom_data_.distanceResults[b].min_distance;
                 } );
      RCLCPP_WARN_THROTTLE( node_->get_logger(), *node_->get_clock(), 10000,
                            "%zu pairs in safety zone (%zu distinct link pairs), keeping %zu.",
                            safety_zone_indices.size(), num_link_pairs, primaries.size() );

      // Dropped pairs are always farther than every kept pair; one inside the
      // braking-critical band means the budget is genuinely too small — make it loud.
      constexpr double kCriticalBand = 0.02; // [m] beyond the padding
      const double critical_distance = collision_padding_ + kCriticalBand;
      for ( const std::size_t k : safety_zone_indices ) {
        const double d = geom_data_.distanceResults[k].min_distance;
        if ( d >= critical_distance ) {
          break; // sorted ascending — nothing critical beyond this point
        }
        if ( std::find( primaries.begin(), primaries.end(), k ) == primaries.end() ) {
          const auto [name_a, name_b] = getPairNames( k );
          RCLCPP_ERROR_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 5000,
              "Collision pair budget too small: dropped pair '%s'<->'%s' at d=%.4f m is "
              "inside the braking-critical band (< %.4f m). Increase the pair budget!",
              name_a.c_str(), name_b.c_str(), d, critical_distance );
          break;
        }
      }

      safety_zone_indices = std::move( primaries );
    }

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

  last_collision_result_ = std::move( result );
  if ( pub_debug_geometry_ )
    publishMarkers();
  else if ( pub_collision_distances_ )
    publishMinimalMarkers();
  return last_collision_result_;
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

void CollisionChecker::setSafetyZoneThreshold( double threshold )
{
  if ( threshold > safety_zone_threshold_ ) {
    // Cached result was computed with a smaller threshold and may be missing
    // pairs that now fall inside the wider safety zone — invalidate it.
    q_last_.resize( 0 );
  }
  safety_zone_threshold_ = threshold;
}

double CollisionChecker::getSafetyZoneThreshold() const { return safety_zone_threshold_; }

void CollisionChecker::setMaxSafetyZonePairs( const std::size_t max_pairs )
{
  if ( max_pairs != max_safety_zone_pairs_ ) {
    // Cached result was truncated with a different cap — invalidate it.
    q_last_.resize( 0 );
    max_safety_zone_pairs_ = max_pairs;
  }
}

void CollisionChecker::setBroadphase( bool enable ) { use_broadphase_ = enable; }

bool CollisionChecker::isBroadphaseEnabled() const { return use_broadphase_; }

void CollisionChecker::updatePublishCollisionDistances( bool enable )
{
  pub_collision_distances_ = enable;
  if ( enable && !rt_markers_pub_ ) {
    auto pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/debug_collision_geometry", 1 );
    rt_markers_pub_ =
        std::make_shared<realtime_tools::RealtimePublisher<visualization_msgs::msg::MarkerArray>>(
            pub );
  }
}

void CollisionChecker::publishMinimalMarkers()
{
  if ( !rt_markers_pub_ || !rt_markers_pub_->trylock() )
    return;

  auto &arr = rt_markers_pub_->msg_;
  arr.markers.clear();

  const rclcpp::Time now = node_->now();

  // Delete all previous markers
  visualization_msgs::msg::Marker delete_all;
  delete_all.header.frame_id = root_frame_;
  delete_all.header.stamp = now;
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back( std::move( delete_all ) );

  const auto &result = last_collision_result_;
  if ( result.safety_zone_pairs.empty() && !result.in_collision ) {
    rt_markers_pub_->unlockAndPublish();
    return;
  }

  // Build LINE_LIST markers for safety zone and collision pairs
  auto make_line_marker = [&]( const std::string &ns, int id, double thickness ) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = root_frame_;
    m.header.stamp = now;
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = thickness;
    m.lifetime = rclcpp::Duration::from_seconds( 0.0 );
    m.pose.orientation.w = 1.0;
    return m;
  };

  visualization_msgs::msg::Marker lines_zone =
      make_line_marker( "distance_lines_safety_zone", 0, 0.005 );
  visualization_msgs::msg::Marker lines_coll =
      make_line_marker( "distance_lines_collision", 0, 0.006 );

  // Collision-category lines use per-vertex colors:
  //   orange  = inside the padding but not touching (0 < d <= padding)
  //   magenta = actually penetrating (d <= 0)
  std_msgs::msg::ColorRGBA orange;
  orange.r = 1.0f;
  orange.g = 0.55f;
  orange.b = 0.0f;
  orange.a = 1.0f;
  std_msgs::msg::ColorRGBA magenta;
  magenta.r = 1.0f;
  magenta.g = 0.0f;
  magenta.b = 1.0f;
  magenta.a = 1.0f;

  auto push_collision_line = [&]( const geometry_msgs::msg::Point &pA,
                                  const geometry_msgs::msg::Point &pB, const double distance ) {
    const auto &color = ( distance <= 0.0 ) ? magenta : orange;
    lines_coll.points.push_back( pA );
    lines_coll.colors.push_back( color );
    lines_coll.points.push_back( pB );
    lines_coll.colors.push_back( color );
  };

  // Fallback: when the controller did not request gradient computation (threshold=0),
  // safety_zone_pairs is empty even on collision. Draw the colliding pair from the
  // global-min index so RViz still shows the collision line.
  if ( result.safety_zone_pairs.empty() && result.in_collision &&
       result.min_distance_pair_index < geom_data_.distanceResults.size() ) {
    const auto &dres = geom_data_.distanceResults[result.min_distance_pair_index];
    if ( !dres.nearest_points[0].hasNaN() && !dres.nearest_points[1].hasNaN() &&
         dres.nearest_points[0].allFinite() && dres.nearest_points[1].allFinite() ) {
      geometry_msgs::msg::Point pA, pB;
      pA.x = dres.nearest_points[0][0];
      pA.y = dres.nearest_points[0][1];
      pA.z = dres.nearest_points[0][2];
      pB.x = dres.nearest_points[1][0];
      pB.y = dres.nearest_points[1][1];
      pB.z = dres.nearest_points[1][2];
      push_collision_line( pA, pB, dres.min_distance );
    }
  }

  for ( const auto &pair : result.safety_zone_pairs ) {
    const auto &dres = geom_data_.distanceResults[pair.pair_index];
    if ( dres.nearest_points[0].hasNaN() || dres.nearest_points[1].hasNaN() ||
         !dres.nearest_points[0].allFinite() || !dres.nearest_points[1].allFinite() )
      continue;

    geometry_msgs::msg::Point pA, pB;
    pA.x = dres.nearest_points[0][0];
    pA.y = dres.nearest_points[0][1];
    pA.z = dres.nearest_points[0][2];
    pB.x = dres.nearest_points[1][0];
    pB.y = dres.nearest_points[1][1];
    pB.z = dres.nearest_points[1][2];

    if ( pair.distance <= collision_padding_ ) {
      push_collision_line( pA, pB, pair.distance );
    } else {
      std_msgs::msg::ColorRGBA color;
      const double dir = pairDirection( viz_directional_derivatives_,
                                        geom_model_.collisionPairs.size(), pair.pair_index );
      // Neutral band: |g^T v| below this is tangential motion / standstill — without it
      // numerical noise around zero makes the color flicker red/green.
      constexpr double kDirNeutralBand = 1e-3; // [m/s]
      if ( !std::isnan( dir ) && std::abs( dir ) > kDirNeutralBand ) {
        if ( dir > 0.0 ) {
          color.r = 0.0f;
          color.g = 1.0f;
          color.b = 0.0f;
          color.a = 1.0f;
        } else {
          color.r = 1.0f;
          color.g = 0.0f;
          color.b = 0.0f;
          color.a = 1.0f;
        }
      } else {
        color.r = 1.0f;
        color.g = 0.8f;
        color.b = 0.0f;
        color.a = 0.9f;
      }
      lines_zone.points.push_back( pA );
      lines_zone.colors.push_back( color );
      lines_zone.points.push_back( pB );
      lines_zone.colors.push_back( color );
    }
  }

  // RViz ignores an empty-points LINE_LIST update (old lines persist); DELETE to clear instead.
  auto push_line_marker = [&]( visualization_msgs::msg::Marker &&m ) {
    if ( m.points.empty() ) {
      m.action = visualization_msgs::msg::Marker::DELETE;
    }
    arr.markers.push_back( std::move( m ) );
  };
  push_line_marker( std::move( lines_zone ) );
  push_line_marker( std::move( lines_coll ) );

  rt_markers_pub_->unlockAndPublish();
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

  // Direction vector: from p1 to p2 (positive distance direction).
  // coal's witness points satisfy p2 - p1 = min_distance * normal, so for PENETRATING
  // pairs (min_distance < 0) the vector p2 - p1 is ANTI-parallel to the separation
  // normal and must be flipped, otherwise the gradient points into the collision.
  const Eigen::Vector3d diff = ( dres.min_distance < 0.0 ) ? ( p1 - p2 ).eval() : ( p2 - p1 ).eval();
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

std::pair<std::string, std::string> CollisionChecker::getPairNames( std::size_t pair_index ) const
{
  if ( pair_index >= geom_model_.collisionPairs.size() )
    return { "", "" };
  const auto &cp = geom_model_.collisionPairs[pair_index];
  return { geom_model_.geometryObjects[cp.first].name, geom_model_.geometryObjects[cp.second].name };
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

  // Build lookups for geometry coloring: penetrating (d <= 0) and inside padding
  // (0 < d <= padding).
  std::vector<size_t> objects_penetrating;
  std::vector<size_t> objects_in_padding;
  auto add_unique = [&]( std::vector<size_t> &vec, size_t idx ) {
    if ( std::find( vec.begin(), vec.end(), idx ) == vec.end() )
      vec.push_back( idx );
  };
  for ( std::size_t k = 0; k < geom_model_.collisionPairs.size(); ++k ) {
    if ( !nearest_points_fresh_[k] )
      continue; // skip stale (pruned) pairs
    const auto &cp = geom_model_.collisionPairs[k];
    const auto &dres = geom_data_.distanceResults[k];
    if ( dres.min_distance <= 0.0 ) {
      add_unique( objects_penetrating, cp.first );
      add_unique( objects_penetrating, cp.second );
    } else if ( dres.min_distance <= collision_padding_ ) {
      add_unique( objects_in_padding, cp.first );
      add_unique( objects_in_padding, cp.second );
    }
  }

  // 1) Geometry markers
  for ( std::size_t i = 0; i < geom_model_.geometryObjects.size(); ++i ) {
    const auto &go = geom_model_.geometryObjects[i];
    const auto &M = geom_data_.oMg[i];

    visualization_msgs::msg::Marker m;
    m.header.frame_id = root_frame_;
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

    using namespace coal;
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

    if ( std::find( objects_penetrating.begin(), objects_penetrating.end(), i ) !=
         objects_penetrating.end() ) {
      // Penetrating: red
      m.color.r = 1.0f;
      m.color.g = 0.0f;
      m.color.b = 0.0f;
      m.color.a = 1.0f;
    } else if ( std::find( objects_in_padding.begin(), objects_in_padding.end(), i ) !=
                objects_in_padding.end() ) {
      // Inside the padding (counts as collision for the controller, but not touching): orange
      m.color.r = 1.0f;
      m.color.g = 0.55f;
      m.color.b = 0.0f;
      m.color.a = 0.9f;
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
  auto valid_nearest_points = []( const coal::DistanceResult &dres ) -> bool {
    return !dres.nearest_points[0].hasNaN() && !dres.nearest_points[1].hasNaN() &&
           dres.nearest_points[0].allFinite() && dres.nearest_points[1].allFinite();
  };

  // Helper to create a line marker between nearest points of a pair
  auto make_line_points = []( const coal::DistanceResult &dres )
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

  // 2) Distance lines — separated into namespaces by category
  // Initialize LINE_LIST markers for each category
  auto make_line_marker = [&]( const std::string &ns, int id, double thickness ) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = root_frame_;
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
  // Collision-category lines use per-vertex colors:
  //   orange  = inside the padding but not touching (0 < d <= padding)
  //   magenta = actually penetrating (d <= 0)
  std_msgs::msg::ColorRGBA orange;
  orange.r = 1.0f;
  orange.g = 0.55f;
  orange.b = 0.0f;
  orange.a = 1.0f;
  std_msgs::msg::ColorRGBA magenta;
  magenta.r = 1.0f;
  magenta.g = 0.0f;
  magenta.b = 1.0f;
  magenta.a = 1.0f;

  lines_safe.color = gray;

  for ( std::size_t k = 0; k < geom_model_.collisionPairs.size(); ++k ) {
    // Skip stale (pruned) pairs; their nearest points are from an earlier cycle.
    if ( !nearest_points_fresh_[k] )
      continue;

    const auto &dres = geom_data_.distanceResults[k];
    if ( !valid_nearest_points( dres ) )
      continue;

    auto [pA, pB] = make_line_points( dres );

    if ( dres.min_distance <= collision_padding_ ) {
      // Collision pair (per-vertex color: magenta = penetrating, orange = in padding)
      const auto &color = ( dres.min_distance <= 0.0 ) ? magenta : orange;
      lines_coll.points.push_back( pA );
      lines_coll.colors.push_back( color );
      lines_coll.points.push_back( pB );
      lines_coll.colors.push_back( color );
    } else if ( viz_safety_zone_threshold_ > 0.0 && dres.min_distance < viz_safety_zone_threshold_ ) {
      // Safety zone pair — color by directional derivative. Neutral band avoids
      // red/green flicker from numerical noise around zero (standstill/tangential).
      std_msgs::msg::ColorRGBA color;
      const double dir =
          pairDirection( viz_directional_derivatives_, geom_model_.collisionPairs.size(), k );
      constexpr double kDirNeutralBand = 1e-3; // [m/s]
      if ( !std::isnan( dir ) && std::abs( dir ) > kDirNeutralBand ) {
        if ( dir > 0.0 ) {
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
        // No directional info / neutral: yellow
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

  // RViz ignores an empty-points LINE_LIST update (old lines persist); DELETE to clear instead.
  auto push_line_marker = [&]( visualization_msgs::msg::Marker &&m ) {
    if ( m.points.empty() ) {
      m.action = visualization_msgs::msg::Marker::DELETE;
    }
    arr.markers.push_back( std::move( m ) );
  };
  push_line_marker( std::move( lines_safe ) );
  push_line_marker( std::move( lines_zone ) );
  push_line_marker( std::move( lines_coll ) );

  markers_pub_->publish( arr );
}

bool CollisionChecker::setManipulabilityFrame( const std::string &ee_frame_name )
{
  manipulability_frame_ = kNoFrame;
  if ( ee_frame_name.empty() ) {
    return true;
  }
  if ( !model_.existFrame( ee_frame_name ) ) {
    return false;
  }
  manipulability_frame_ = model_.getFrameId( ee_frame_name );
  manipulability_jacobian_.setZero( 6, model_.nv );
  return true;
}

double CollisionChecker::computeManipulability()
{
  if ( manipulability_frame_ == kNoFrame || q_last_.size() != model_.nq ) {
    return 0.0; // disabled, or no collision check has run yet
  }

  // computeFrameJacobian internally refreshes the kinematics it needs, so the result
  // is correct independent of which pinocchio passes ran during the previous collision
  // check (computeJointJacobians is only called when safety-zone pairs exist).
  manipulability_jacobian_.setZero();
  pinocchio::computeFrameJacobian( model_, data_, q_last_, manipulability_frame_,
                                   pinocchio::LOCAL_WORLD_ALIGNED, manipulability_jacobian_ );

  // Yoshikawa manipulability: w = sqrt(det(J * J^T)); the 6x6 stays on the stack
  const Eigen::Matrix<double, 6, 6> jjt =
      manipulability_jacobian_ * manipulability_jacobian_.transpose();
  const double det = jjt.determinant();
  return det > 0.0 ? std::sqrt( det ) : 0.0;
}
