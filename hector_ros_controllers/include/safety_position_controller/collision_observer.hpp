#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <hardware_interface/loaned_state_interface.hpp>

#include <safety_position_controller/collision_checker.hpp>
#include <safety_position_controller/safety_pipeline.hpp>

namespace safety_position_controller
{

/**
 * @brief Per-cycle collision observation for the safety pipeline.
 *
 * Wraps the CollisionChecker interaction: assembles the check configuration (measured
 * positions for uncontrolled joints, commanded for controlled ones), runs the distance
 * check, caches the results and converts safety-zone pairs into pipeline candidates.
 * Tracks the not-in-collision → in-collision edge so a steady collision is reported
 * once per episode (the edge state resets silently whenever the collision state stops
 * being observed). Reports flags instead of logging.
 */
class CollisionObserver
{
public:
  struct Snapshot {
    /// Observation for SafetyPipeline::step(); pairs point into this observer.
    SafetyPipeline::CollisionObservation observation;
    bool collision_started{ false }; ///< edge: entered collision this cycle (warn once)
  };

  /**
   * @param checker non-owning; may be null (observations are then never active)
   * @param all_joint_names all non-fixed joints, in state-interface order
   * @param controlled_joints the controlled joints (commanded positions overlay)
   * @param joint_v_index controlled joint index → collision-model velocity-space index
   */
  CollisionObserver( CollisionChecker *checker, std::vector<std::string> all_joint_names,
                     std::vector<std::string> controlled_joints, std::vector<int> joint_v_index );

  /// Reset caches and the collision edge state (on activation).
  void reset();

  /**
   * @brief Run the collision check at the commanded configuration.
   * When @p checks_active is false (bypass / checks disabled), resets the edge state
   * and caches instead. observation.state_valid=false means the joint state reads
   * failed and the safety state is unobservable this cycle.
   * @param checks_active whether collision checking should run this cycle
   * @param state_interfaces position state interfaces in all_joint_names order
   * @param commanded_positions commanded configuration of the controlled joints
   * @param safety_zone_threshold outer zone distance for gradient requests [m]
   */
  Snapshot observe( bool checks_active,
                    std::vector<hardware_interface::LoanedStateInterface> &state_interfaces,
                    const Eigen::VectorXd &commanded_positions, double safety_zone_threshold );

  /// Push per-pair directional info (gradient · velocity) to the checker for RViz
  /// distance-line coloring (green = moving away). No-op without a valid observation.
  void publishDirectionalInfo( const Eigen::VectorXd &velocity, double safety_zone );

  double lastMinDistance() const { return last_min_distance_; }
  std::size_t lastMinDistancePairIndex() const { return last_min_distance_pair_index_; }
  /// Safety-zone pairs of the last observation, empty when there were none. Points into
  /// the checker's latched result and stays valid until the next observe().
  const std::vector<CollisionResult::PairInfo> &lastSafetyZonePairs() const;

private:
  /// Clear the per-observation caches (distances, pair pointer, collision edge).
  void clearObservation();

  CollisionChecker *checker_; ///< non-owning
  std::vector<std::string> all_joint_names_;
  std::vector<std::string> controlled_joints_;
  std::vector<int> joint_v_index_;

  std::unordered_map<std::string, double> cc_positions_; ///< name→position map for the check
  std::vector<SafetyPipeline::PairCandidate> pair_candidates_;
  const std::vector<CollisionResult::PairInfo> *last_safety_zone_pairs_{ nullptr };
  double last_min_distance_{ std::numeric_limits<double>::max() };
  std::size_t last_min_distance_pair_index_{ std::numeric_limits<std::size_t>::max() };
  bool was_in_collision_{ false };
  bool observed_{ false }; ///< last observe() produced a valid collision state
};

} // namespace safety_position_controller
