//
// Created by aljoscha-schmidt on 10/20/25.
//

#ifndef COLLISION_CHECKER_HPP
#define COLLISION_CHECKER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

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
   */
  explicit CollisionChecker( const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
                             double collision_padding = 0.0, double collision_cache_epsilon = 1e-6 );

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
   * @return the latched result (collision flag, minimum clearance, optional per-pair
   * gradients); valid until the next collision query
   */
  const CollisionResult &
  checkCollision( const std::unordered_map<std::string, double> &joint_positions );

  /**
   * @brief Collision check for full q.
   * Uses the safety-zone threshold last set via setSafetyZoneThreshold (default 0).
   * @param q size == model_.nq
   * @return the latched result (collision flag, minimum clearance, optional per-pair
   * gradients); valid until the next collision query
   */
  const CollisionResult &checkCollisionQ( const Eigen::VectorXd &q );

  /**
   * @brief Cap the number of safety-zone pairs returned (and gradient computations).
   * When > 0, checkCollision keeps only the @p max_pairs closest pairs (sorted by
   * distance ascending); 0 = unlimited. Pairs are always sorted by distance ascending.
   * @param max_pairs maximum number of pairs; 0 disables the cap
   */
  void setMaxSafetyZonePairs( std::size_t max_pairs );

  /// Where a joint's position lives in the configuration vector. Resolve once (name
  /// lookup) and reuse every cycle; index < 0 means the joint is not in the model.
  struct JointQSlot {
    int index{ -1 };
    bool continuous{ false }; ///< stored as the unit complex [cos, sin]
  };

  /**
   * @brief Resolve a joint's configuration-vector slot by name.
   * @return the slot; index < 0 if the joint is unknown or has an unsupported DoF layout
   */
  JointQSlot getJointQSlot( const std::string &joint_name ) const;

  /// Write one joint position into a configuration vector; no-op for an invalid slot.
  static void writeJointPosition( Eigen::VectorXd &q, const JointQSlot &slot, double position );

  /// Neutral configuration of the model (size model_.nq); the seed for building q.
  const Eigen::VectorXd &neutralConfiguration() const { return q_default_; }

  /**
   * @brief Build a full pinocchio configuration vector from a name→position map.
   * Unknown joints are ignored (warn-throttled); unset joints keep their neutral value.
   * Handles 1-DoF joints and continuous joints ([cos, sin]).
   * @param joint_positions rad (revolute) / m (prismatic)
   * @return q of size model_.nq
   */
  Eigen::VectorXd buildConfiguration( const std::unordered_map<std::string, double> &joint_positions );

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

  // ---- Read-only views of the last check, for visualization ----
  const pinocchio::GeometryModel &geometryModel() const { return geom_model_; }
  const pinocchio::GeometryData &geometryData() const { return geom_data_; }
  const CollisionResult &lastResult() const { return last_collision_result_; }
  /// Per pair: whether its distance (and nearest points) were recomputed this cycle.
  /// Broadphase pruning leaves pruned pairs holding stale data.
  const std::vector<bool> &nearestPointsFresh() const { return nearest_points_fresh_; }
  /// URDF root link; FK is relative to it, so markers are published in this frame.
  const std::string &rootFrame() const { return root_frame_; }
  double collisionPadding() const { return collision_padding_; }

  /**
   * @brief Set collision padding [m].
   * @param collision_padding new threshold
   */
  void updateCollisionPadding( double collision_padding );

  /**
   * @brief Set cache epsilon
   * @param epsilon new threshold
   */
  void updateCollisionCacheEpsilon( double epsilon );

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
   * @brief Select the end-effector frame for computeManipulability().
   * Resolves the frame once so the per-cycle call needs no name lookup.
   * @param ee_frame_name frame in the URDF; empty disables manipulability
   * @return false if the frame is not in the model
   */
  bool setManipulabilityFrame( const std::string &ee_frame_name );

  /**
   * @brief Yoshikawa manipulability at the configuration of the last collision check.
   * @return w = sqrt(det(J * J^T)); 0 if singular or no frame was selected
   */
  double computeManipulability();

private:
  /// Forget the cached configuration: the next query recomputes instead of serving
  /// last_collision_result_. Required whenever the latch and q_last_ stop agreeing
  /// (error results) or the classification parameters change.
  void invalidateCache() { q_last_.resize( 0 ); }

  /// "Assume in collision" answer for unusable input; never touches the latch/cache.
  static const CollisionResult &unsafeResult();

  /**
   * @brief Compute the distance gradient for a single collision pair.
   * Requires FK + computeJointJacobians to have been called already.
   * @param pair_k index into geom_model_.collisionPairs
   * @return gradient vector of size model_.nv
   */
  Eigen::VectorXd computePairGradient( std::size_t pair_k );

  /**
   * @brief Keep only pairs attached to controlled joints (and ancestors).
   * @param controlled_joints names defining relevance
   */
  void filterCollisionPairs( const std::vector<std::string> &controlled_joints );

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::GeometryModel geom_model_;
  pinocchio::GeometryData geom_data_;

  Eigen::VectorXd q_default_;
  Eigen::VectorXd q_last_;

  // Per-cycle scratch, kept as members so the control loop does not reallocate.
  std::vector<std::size_t> safety_zone_indices_;
  std::vector<std::size_t> primaries_;
  std::vector<std::size_t> duplicates_;
  std::vector<std::pair<pinocchio::FrameIndex, pinocchio::FrameIndex>> seen_links_;
  CollisionResult last_collision_result_;

  double collision_padding_{ 0.0 };
  double collision_cache_epsilon_{ 1e-6 };
  double safety_zone_threshold_{ 0.0 };    ///< 0 = no gradient computation
  std::size_t max_safety_zone_pairs_{ 0 }; ///< cap on returned pairs; 0 = unlimited

  std::unordered_map<std::string, pinocchio::JointIndex> name_to_id_;

  /// Per-pair: true if nearest_points were recomputed this cycle. Broadphase pruning
  /// leaves pruned pairs holding stale data (see nearestPointsFresh()).
  std::vector<bool> nearest_points_fresh_;

  /// Model root link (URDF root); frame_id for markers since FK is relative to it. Defaults to "base_link".
  std::string root_frame_{ "base_link" };

  // Pre-allocated Jacobian workspace (sized in initFromXml)
  Eigen::MatrixXd J1_workspace_; ///< 6 × nv
  Eigen::MatrixXd J2_workspace_; ///< 6 × nv

  static constexpr pinocchio::FrameIndex kNoFrame = std::numeric_limits<pinocchio::FrameIndex>::max();
  pinocchio::FrameIndex manipulability_frame_{ kNoFrame };
  Eigen::MatrixXd manipulability_jacobian_; ///< 6 × nv workspace

  // Broadphase acceleration
  bool use_broadphase_{ true };
  using BroadPhaseManager = pinocchio::BroadPhaseManagerTpl<coal::DynamicAABBTreeCollisionManager>;
  std::unique_ptr<BroadPhaseManager> broadphase_manager_;
};

#endif // COLLISION_CHECKER_HPP
