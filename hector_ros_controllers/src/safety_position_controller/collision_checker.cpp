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

  // Outputs (filled during traversal)
  double global_min_distance{ std::numeric_limits<double>::max() };
  std::size_t min_distance_pair{ 0 };
  std::vector<std::size_t> safety_zone_indices;
  std::vector<std::size_t> visited_indices; ///< pairs the traversal actually evaluated

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
    dreq.enable_nearest_points = true;
    dres.clear();

    coal::distance( o1, o2, dreq, dres );
    if ( swapped ) {
      // Restore pair-canonical order: nearest_points[0] must belong to cp.first,
      // otherwise gradients computed from the witness points are exactly negated.
      std::swap( dres.nearest_points[0], dres.nearest_points[1] );
      dres.normal = -dres.normal;
    }
    const double d = dres.min_distance;
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
                                    double collision_padding, double collision_cache_epsilon )
    : node_( node ), collision_padding_( collision_padding ),
      collision_cache_epsilon_( collision_cache_epsilon )
{
}

bool CollisionChecker::initFromXml( const std::string &urdf_xml, const std::string &srdf_xml,
                                    const std::vector<std::string> &controlled_joints,
                                    bool free_flyer )
{
  try {
    // The parsers append to whatever the model already holds, so a second init (a
    // reconfigure, or a retry after a failed activation) would duplicate every joint
    // and leave the filter with no pairs it can influence — a check that silently
    // passes everything.
    model_ = pinocchio::Model();
    geom_model_ = pinocchio::GeometryModel();

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

    // A cached result from a previous model must not survive re-initialization.
    invalidateCache();
    last_collision_result_ = CollisionResult{};

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
  // Static: the latch and the movement cache stay untouched, so after the input
  // recovers a stationary robot is served the still-valid cached result instead of a
  // poisoned "assume collision" latch.
  static const CollisionResult kUnsafe = [] {
    CollisionResult r;
    r.in_collision = true;
    r.min_distance = 0.0;
    return r;
  }();
  return kUnsafe;
}

const CollisionResult &
CollisionChecker::checkCollision( const std::unordered_map<std::string, double> &joint_positions )
{
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

CollisionChecker::JointQSlot CollisionChecker::getJointQSlot( const std::string &joint_name ) const
{
  const auto it = name_to_id_.find( joint_name );
  if ( it == name_to_id_.end() ) {
    return {};
  }
  const pinocchio::JointIndex jid = it->second;
  const int nq_j = model_.joints[jid].nq();
  const int nv_j = model_.joints[jid].nv();
  if ( nq_j == 1 ) { // revolute / prismatic
    return { model_.idx_qs[jid], false };
  }
  if ( nq_j == 2 && nv_j == 1 ) { // continuous: unit complex [cos, sin]
    return { model_.idx_qs[jid], true };
  }
  return {};
}

void CollisionChecker::writeJointPosition( Eigen::VectorXd &q, const JointQSlot &slot,
                                           const double position )
{
  if ( slot.index < 0 ) {
    return;
  }
  if ( slot.continuous ) {
    q[slot.index] = std::cos( position );
    q[slot.index + 1] = std::sin( position );
  } else {
    q[slot.index] = position;
  }
}

Eigen::VectorXd
CollisionChecker::buildConfiguration( const std::unordered_map<std::string, double> &joint_positions )
{
  // transforms the joint positions into the pinocchio format
  Eigen::VectorXd q = q_default_;
  for ( const auto &[name, position] : joint_positions ) {
    const JointQSlot slot = getJointQSlot( name );
    if ( slot.index < 0 ) {
      RCLCPP_WARN_THROTTLE( node_->get_logger(), *node_->get_clock(), 2000,
                            "Joint '%s' is unknown or has an unsupported DoF layout (ignored).",
                            name.c_str() );
      continue;
    }
    writeJointPosition( q, slot, position );
  }
  return q;
}
const CollisionResult &CollisionChecker::checkCollisionQ( const Eigen::VectorXd &q )
{
  const double safety_zone_threshold = safety_zone_threshold_;

  if ( model_.nq == 0 ) {
    RCLCPP_ERROR( node_->get_logger(), "Model not initialized." );
    return unsafeResult();
  }
  if ( q.size() != model_.nq ) {
    RCLCPP_ERROR( node_->get_logger(), "q size (%ld) != model.nq (%d)", long( q.size() ), model_.nq );
    return unsafeResult();
  }
  // Every distance comparison against a NaN is false, which would report "no collision,
  // min_distance = DBL_MAX" — the check has to fail safe, not open.
  if ( !q.allFinite() ) {
    RCLCPP_ERROR_THROTTLE( node_->get_logger(), *node_->get_clock(), 2000,
                           "Configuration is not finite. Assuming the robot is in collision." );
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
  pinocchio::updateGeometryPlacements( model_, data_, geom_model_, geom_data_ );

  // Nearest points come out of the same narrow-phase call as the distance, so asking
  // for them up front is cheaper than re-running the query for the safety-zone pairs
  // (benchmarked: 84 us vs 116 us for the broadphase path).
  std::fill( nearest_points_fresh_.begin(), nearest_points_fresh_.end(), false );

  double global_min_distance = std::numeric_limits<double>::max();
  std::size_t min_distance_pair = 0;
  bool has_safety_zone_pairs = false;
  std::vector<std::size_t> &safety_zone_indices = safety_zone_indices_; // capacity reused
  safety_zone_indices.clear();

  if ( use_broadphase_ && broadphase_manager_ ) {
    // --- Broadphase path: AABB-tree pruned distance scan ---
    broadphase_manager_->update( false ); // sync transforms from oMg, rebuild AABB tree

    SafetyZoneDistanceCallback callback;
    callback.geom_model_ptr = &geom_model_;
    callback.geom_data_ptr = &geom_data_;
    callback.safety_zone_threshold = safety_zone_threshold;
    callback.init();

    broadphase_manager_->getManager().distance( &callback );

    global_min_distance = callback.global_min_distance;
    min_distance_pair = callback.min_distance_pair;
    safety_zone_indices.swap( callback.safety_zone_indices );
    has_safety_zone_pairs = !safety_zone_indices.empty();

    // Pruned pairs keep last cycle's nearest points; only the visited ones are fresh.
    // Every safety-zone pair is visited: the pruning bound never drops below the zone.
    for ( const std::size_t k : callback.visited_indices ) { nearest_points_fresh_[k] = true; }
  } else {
    // --- Brute-force path: compute distances for ALL pairs ---
    for ( auto &dreq : geom_data_.distanceRequests ) { dreq.enable_nearest_points = true; }
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

    // Brute force visits every pair, so all are fresh.
    std::fill( nearest_points_fresh_.begin(), nearest_points_fresh_.end(), true );
  }

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
      std::vector<std::size_t> &primaries = primaries_;
      std::vector<std::size_t> &duplicates = duplicates_;
      auto &seen_links = seen_links_;
      primaries.clear();
      duplicates.clear();
      seen_links.clear();
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

      safety_zone_indices.swap( primaries );
    }

    pinocchio::computeJointJacobians( model_, data_ );

    for ( const std::size_t k : safety_zone_indices ) {
      CollisionResult::PairInfo info;
      info.pair_index = k;
      info.distance = geom_data_.distanceResults[k].min_distance;
      info.gradient = computePairGradient( k );
      result.safety_zone_pairs.push_back( std::move( info ) );
    }
  }

  last_collision_result_ = std::move( result );
  return last_collision_result_;
}

void CollisionChecker::updateCollisionPadding( const double collision_padding )
{
  if ( collision_padding != collision_padding_ ) {
    // The cached result classified in_collision under the old padding.
    invalidateCache();
    collision_padding_ = collision_padding;
  }
}

void CollisionChecker::updateCollisionCacheEpsilon( const double epsilon )
{
  collision_cache_epsilon_ = epsilon;
}

void CollisionChecker::setSafetyZoneThreshold( double threshold )
{
  if ( threshold != safety_zone_threshold_ ) {
    // The cached result's safety-zone pair set was selected under the old threshold.
    invalidateCache();
    safety_zone_threshold_ = threshold;
  }
}

double CollisionChecker::getSafetyZoneThreshold() const { return safety_zone_threshold_; }

void CollisionChecker::setMaxSafetyZonePairs( const std::size_t max_pairs )
{
  if ( max_pairs != max_safety_zone_pairs_ ) {
    // Cached result was truncated with a different cap.
    invalidateCache();
    max_safety_zone_pairs_ = max_pairs;
  }
}

void CollisionChecker::setBroadphase( bool enable )
{
  if ( enable != use_broadphase_ ) {
    // The two paths can produce different results for far configurations.
    invalidateCache();
    use_broadphase_ = enable;
  }
}

bool CollisionChecker::isBroadphaseEnabled() const { return use_broadphase_; }

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
