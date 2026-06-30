//
// Created by aljoscha-schmidt on 10/20/25.
//

#ifndef COLLISION_CHECKER_HPP
#define COLLISION_CHECKER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

#include <pinocchio/collision/broadphase-manager.hpp>

#include <Eigen/Core>
#include <coal/broadphase/broadphase_dynamic_AABB_tree.h>
#include <coal/collision.h>
#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

// #define SAFETY_CC_ENABLE_TIMING // TODO: remove when no longer needed for optimization

/// Result of a collision query: collision flag + minimum clearance.
struct CollisionResult {
  bool in_collision{ false }; ///< true if any pair distance <= padding
  double min_distance{ std::numeric_limits<double>::max() }; ///< global minimum pairwise distance [m]
  std::size_t min_distance_pair_index{
      std::numeric_limits<std::size_t>::max() }; ///< index of the pair attaining min_distance; SIZE_MAX if none

  /// Per-pair info for pairs within the safety zone (only populated when gradient computation requested).
  struct PairInfo {
    std::size_t pair_index;   ///< index into geom_model_.collisionPairs
    double distance;          ///< pairwise distance [m]
    Eigen::VectorXd gradient; ///< dd/dv (size model_.nv): positive = moving apart
  };
  std::vector<PairInfo> safety_zone_pairs; ///< pairs with distance < safety_zone_threshold
};

/// Self-collision checker using Pinocchio + coal; optional RViz debug markers.
class CollisionChecker
{
public:
  /**
   * @brief Ctor.
   * @param node lifecycle node (pub/log/time)
   * @param collision_padding min allowed distance [m]
   * @param collision_cache_epsilon cache threshold on max(q - q_last) [rad/m] -> reuse last distances
   * @param pub_debug_geometry publish MarkerArray on ~/debug_collision_geometry
   */
  explicit CollisionChecker( const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
                             double collision_padding = 0.0, double collision_cache_epsilon = 1e-6,
                             bool pub_debug_geometry = false );

  /**
   * @brief Init from URDF/SRDF XML.
   * - builds Model/Geometry(COLLISION)
   * - prunes pairs via SRDF + controlled_joints ancestry
   * @param urdf_xml URDF string
   * @param srdf_xml SRDF string (may be empty)
   * @param controlled_joints names used to keep relevant pairs (empty → keep all)
   * @param free_flyer root as free-flyer
   * @return true on success
   */
  bool initFromXml( const std::string &urdf_xml, const std::string &srdf_xml,
                    const std::vector<std::string> &controlled_joints, bool free_flyer = false );

  /**
   * @brief Joint names excluding "universe".
   * @return names in model order
   */
  std::vector<std::string> getJointNames() const;

  /**
   * @brief Collision check from name→position map.
   * Uses the safety-zone threshold last set via setSafetyZoneThreshold (default 0).
   * @param joint_positions rad (rev) / m (prismatic)
   * @return CollisionResult with collision flag, minimum clearance, and optional per-pair gradients
   */
  CollisionResult checkCollision( const std::unordered_map<std::string, double> &joint_positions );

  /**
   * @brief Collision check for full q.
   * Uses the safety-zone threshold last set via setSafetyZoneThreshold (default 0).
   * @param q size == model_.nq
   * @return CollisionResult with collision flag, minimum clearance, and optional per-pair gradients
   */
  CollisionResult checkCollisionQ( const Eigen::VectorXd &q );

  /**
   * @brief Set the safety-zone threshold used by subsequent collision queries.
   * When > 0, the next query computes per-pair distance gradients (dd/dv) for all
   * pairs with distance < threshold; otherwise gradient computation is skipped.
   * If the new threshold is larger than the previously cached one, the cache is
   * invalidated so the next call cannot reuse a result that omitted now-relevant pairs.
   * @param threshold new threshold in [m]; 0 disables gradient computation
   */
  void setSafetyZoneThreshold( double threshold );

  /**
   * @brief Currently configured safety-zone threshold [m].
   */
  double getSafetyZoneThreshold() const;

  /**
   * @brief Get the velocity-space index for a named joint.
   * @return starting index in model_.nv, or -1 if not found
   */
  int getJointVelocityIndex( const std::string &joint_name ) const;

  /**
   * @brief Get the total velocity-space dimension.
   */
  int getNv() const;

  /**
   * @brief Get the number of collision pairs.
   */
  std::size_t getNumCollisionPairs() const;

  /**
   * @brief Get the geometry-object names making up a collision pair.
   * @param pair_index index into geom_model_.collisionPairs
   * @return {first_name, second_name}; empty strings if pair_index is out of range
   */
  std::pair<std::string, std::string> getPairNames( std::size_t pair_index ) const;

  /**
   * @brief Set collision padding [m].
   * @param collision_padding new threshold
   */
  void updateCollisionPadding( double collision_padding );

  /**
   * @brief Toggle RViz debug publishing.
   * @param pub_debug_geometry on/off
   */
  void updateDoDebugVisualization( bool pub_debug_geometry );

  /**
   * @brief Set cache epsilon
   * @param epsilon new threshold
   */
  void updateCollisionCacheEpsilon( double epsilon );

  /**
   * @brief Set per-pair directional derivatives for visualization coloring.
   * Must be called before the next collision check if you want colors to reflect motion direction.
   * @param derivatives one value per collision pair; NaN = no info, >=0 = moving away, <0 = moving closer
   * @param safety_zone_threshold the threshold used to classify pairs into safety zone vs safe
   */
  void setDirectionalInfo( const std::vector<double> &derivatives, double safety_zone_threshold );

  /**
   * @brief Toggle lightweight collision distance visualization.
   * Publishes only safety-zone and collision distance lines via a realtime publisher.
   * Ignored when full debug visualization is active.
   * @param enable on/off
   */
  void updatePublishCollisionDistances( bool enable );

  /**
   * @brief Enable/disable broadphase AABB-tree acceleration for distance queries.
   * Call before initFromXml, or re-call initFromXml after changing.
   */
  void setBroadphase( bool enable );

  /**
   * @brief Whether broadphase acceleration is enabled.
   */
  bool isBroadphaseEnabled() const;

  /**
   * @brief Compute the Yoshikawa manipulability index for a given end-effector frame.
   * Evaluated at the configuration of the last collision check (q_last_).
   * @param ee_frame_name name of the end-effector frame in the URDF
   * @return w = sqrt(det(J * J^T)), 0 if singular or frame not found
   */
  double computeManipulability( const std::string &ee_frame_name );

private:
  /**
   * @brief Compute the distance gradient for a single collision pair.
   * Requires FK + computeJointJacobians to have been called already.
   * @param pair_k index into geom_model_.collisionPairs
   * @return gradient vector of size model_.nv
   */
  Eigen::VectorXd computePairGradient( std::size_t pair_k );

  /**
   * @brief Publish geometry and nearest-point markers with namespace-separated categories.
   */
  void publishMarkers() const;

  /**
   * @brief Publish lightweight distance-only markers for safety zone and collision pairs.
   * Uses realtime publisher (non-blocking). Skips geometry markers and safe-pair lines.
   */
  void publishMinimalMarkers();

  /**
   * @brief Keep only pairs attached to controlled joints (and ancestors).
   * @param controlled_joints names defining relevance
   */
  void filterCollisionPairs( const std::vector<std::string> &controlled_joints );

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::GeometryModel geom_model_;
  pinocchio::GeometryData geom_data_;

  Eigen::VectorXd q_default_;
  Eigen::VectorXd q_last_;
  CollisionResult last_collision_result_;

  double collision_padding_{ 0.0 };
  double collision_cache_epsilon_{ 1e-6 };
  double safety_zone_threshold_{ 0.0 }; ///< 0 = no gradient computation
  bool pub_debug_geometry_{ false };
  bool pub_collision_distances_{ false };
  std::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::msg::MarkerArray>> rt_markers_pub_;

  std::unordered_map<std::string, pinocchio::JointIndex> name_to_id_;

  /// Per-pair: true if nearest_points were recomputed this cycle. Broadphase pruning leaves pruned
  /// pairs holding stale data, so publishMarkers() must skip pairs that are not fresh.
  std::vector<bool> nearest_points_fresh_;

  /// Model root link (URDF root); frame_id for markers since FK is relative to it. Defaults to "base_link".
  std::string root_frame_{ "base_link" };

  // Per-pair directional derivatives for visualization (set by controller via setDirectionalInfo)
  std::vector<double> viz_directional_derivatives_; ///< one per collision pair; NaN = no info
  double viz_safety_zone_threshold_{ 0.0 };

  // Pre-allocated Jacobian workspace (sized in initFromXml)
  Eigen::MatrixXd J1_workspace_; ///< 6 × nv
  Eigen::MatrixXd J2_workspace_; ///< 6 × nv

  // Broadphase acceleration
  bool use_broadphase_{ true };
  using BroadPhaseManager = pinocchio::BroadPhaseManagerTpl<coal::DynamicAABBTreeCollisionManager>;
  std::unique_ptr<BroadPhaseManager> broadphase_manager_;

#ifdef SAFETY_CC_ENABLE_TIMING
  struct TimingStats {
    double fk_us{ 0 };
    double placement_us{ 0 };
    double distance_us{ 0 };
    double jacobian_us{ 0 };
    double gradient_us{ 0 };
    double total_us{ 0 };
    int count{ 0 };
    std::size_t num_safety_zone_pairs{ 0 };
  };
  TimingStats timing_stats_;
#endif
};

#endif // COLLISION_CHECKER_HPP
