#include "safety_position_controller/collision_observer.hpp"

#include <cmath>

namespace safety_position_controller
{

CollisionObserver::CollisionObserver( CollisionChecker *checker,
                                      std::vector<std::string> all_joint_names,
                                      std::vector<std::string> controlled_joints,
                                      std::vector<int> joint_v_index )
    : checker_( checker ), all_joint_names_( std::move( all_joint_names ) ),
      controlled_joints_( std::move( controlled_joints ) ),
      joint_v_index_( std::move( joint_v_index ) )
{
}

const std::vector<CollisionResult::PairInfo> &CollisionObserver::lastSafetyZonePairs() const
{
  static const std::vector<CollisionResult::PairInfo> kNone;
  return last_safety_zone_pairs_ ? *last_safety_zone_pairs_ : kNone;
}

void CollisionObserver::clearObservation()
{
  last_min_distance_ = std::numeric_limits<double>::max();
  last_min_distance_pair_index_ = std::numeric_limits<std::size_t>::max();
  last_safety_zone_pairs_ = nullptr;
  was_in_collision_ = false;
}

void CollisionObserver::reset()
{
  clearObservation();
  pair_candidates_.clear();
  observed_ = false;
}

CollisionObserver::Snapshot
CollisionObserver::observe( const bool checks_active,
                            std::vector<hardware_interface::LoanedStateInterface> &state_interfaces,
                            const Eigen::VectorXd &commanded_positions,
                            const double safety_zone_threshold )
{
  Snapshot snapshot;
  snapshot.observation.checks_active = checks_active && checker_;
  pair_candidates_.clear();
  observed_ = false;

  if ( !snapshot.observation.checks_active ) {
    clearObservation();
    return snapshot;
  }

  // Fewer interfaces than joints would silently check stale positions — fail safe.
  bool state_valid = state_interfaces.size() >= all_joint_names_.size();
  for ( size_t i = 0; i < all_joint_names_.size() && i < state_interfaces.size(); ++i ) {
    const auto opt = state_interfaces[i].get_optional();
    // A non-finite position makes the safety state just as unobservable as a busy
    // handle; it must brake here instead of reaching the checker as a fake collision.
    if ( opt.has_value() && std::isfinite( *opt ) ) {
      cc_positions_[all_joint_names_[i]] = *opt;
    } else {
      state_valid = false;
    }
  }
  for ( size_t i = 0; i < controlled_joints_.size(); ++i ) {
    cc_positions_[controlled_joints_[i]] = commanded_positions[static_cast<Eigen::Index>( i )];
  }

  snapshot.observation.state_valid = state_valid;
  if ( !state_valid ) {
    // Nothing was observed this cycle: the caches must not keep reporting the
    // pre-fault distances as current.
    clearObservation();
    return snapshot;
  }

  observed_ = true;
  // Always request gradients for the full zone: they ARE the constraints.
  checker_->setSafetyZoneThreshold( safety_zone_threshold );
  const auto &cc_result = checker_->checkCollision( cc_positions_ );
  last_min_distance_ = cc_result.min_distance;
  last_min_distance_pair_index_ = cc_result.min_distance_pair_index;
  last_safety_zone_pairs_ = &cc_result.safety_zone_pairs;
  snapshot.observation.in_collision = cc_result.in_collision;

  if ( cc_result.in_collision ) {
    snapshot.collision_started = !was_in_collision_;
    was_in_collision_ = true;
  } else {
    was_in_collision_ = false;
  }

  pair_candidates_.reserve( cc_result.safety_zone_pairs.size() );
  for ( const auto &pair_info : cc_result.safety_zone_pairs ) {
    pair_candidates_.push_back( { pair_info.distance, &pair_info.gradient, pair_info.pair_index } );
  }
  snapshot.observation.pairs = &pair_candidates_;
  return snapshot;
}

void CollisionObserver::publishDirectionalInfo( const Eigen::VectorXd &velocity,
                                                const double safety_zone )
{
  if ( !observed_ || !checker_ ) {
    return;
  }
  const std::size_t num_pairs = checker_->getNumCollisionPairs();
  std::vector<double> per_pair_dir( num_pairs, std::numeric_limits<double>::quiet_NaN() );
  for ( const auto &pi : lastSafetyZonePairs() ) {
    if ( pi.pair_index < num_pairs ) {
      double dot = 0.0;
      for ( size_t i = 0; i < joint_v_index_.size(); ++i ) {
        if ( joint_v_index_[i] >= 0 && joint_v_index_[i] < pi.gradient.size() ) {
          dot += pi.gradient[joint_v_index_[i]] * velocity[static_cast<Eigen::Index>( i )];
        }
      }
      per_pair_dir[pi.pair_index] = dot;
    }
  }
  checker_->setDirectionalInfo( per_pair_dir, safety_zone );
}

} // namespace safety_position_controller
