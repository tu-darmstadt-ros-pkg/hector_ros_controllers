//
// Created by aljoscha-schmidt on 10/20/25.
//

#ifndef COLLISION_CHECKER_HPP
#define COLLISION_CHECKER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

#include <Eigen/Core>
#include <hpp/fcl/collision.h>
#include <limits>
#include <unordered_map>

// #define SAFETY_CC_ENABLE_TIMING

/// Result of a collision query: collision flag + minimum clearance.
struct CollisionResult {
  bool in_collision{ false }; ///< true if any pair distance <= padding
  double min_distance{ std::numeric_limits<double>::max() }; ///< global minimum pairwise distance [m]

  /// Per-pair info for pairs within the safety zone (only populated when gradient computation requested).
  struct PairInfo {
    std::size_t pair_index;   ///< index into geom_model_.collisionPairs
    double distance;          ///< pairwise distance [m]
    Eigen::VectorXd gradient; ///< dd/dv (size model_.nv): positive = moving apart
  };
  std::vector<PairInfo> safety_zone_pairs; ///< pairs with distance < safety_zone_threshold
};

/// Self-collision checker using Pinocchio + hpp-fcl; optional RViz debug markers.
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
                             double collision_padding = 0.0, double collision_cache_epsilon = 1e-4,
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
   * @param joint_positions rad (rev) / m (prismatic)
   * @param safety_zone_threshold when > 0, computes per-pair distance gradients (dd/dv)
   *        for all pairs with distance < threshold. Set to 0 to skip gradient computation.
   * @return CollisionResult with collision flag, minimum clearance, and optional per-pair gradients
   */
  CollisionResult checkCollision( const std::unordered_map<std::string, double> &joint_positions,
                                  double safety_zone_threshold = 0.0 );

  /**
   * @brief Collision check for full q.
   * @param q size == model_.nq
   * @param safety_zone_threshold when > 0, computes per-pair distance gradients (dd/dv)
   *        for all pairs with distance < threshold. Set to 0 to skip gradient computation.
   * @return CollisionResult with collision flag, minimum clearance, and optional per-pair gradients
   */
  CollisionResult checkCollisionQ( const Eigen::VectorXd &q, double safety_zone_threshold = 0.0 );

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

private:
  /**
   * @brief Compute the distance gradient for a single collision pair.
   * Requires FK + computeJointJacobians to have been called already.
   * @param pair_k index into geom_model_.collisionPairs
   * @return gradient vector of size model_.nv
   */
  Eigen::VectorXd computePairGradient( std::size_t pair_k ) const;

  /**
   * @brief Publish geometry and nearest-point markers with namespace-separated categories.
   */
  void publishMarkers() const;

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
  double collision_cache_epsilon_{ 1e-4 };
  bool pub_debug_geometry_{ false };

  std::unordered_map<std::string, pinocchio::JointIndex> name_to_id_;

  // Per-pair directional derivatives for visualization (set by controller via setDirectionalInfo)
  std::vector<double> viz_directional_derivatives_; ///< one per collision pair; NaN = no info
  double viz_safety_zone_threshold_{ 0.0 };

#ifdef SAFETY_CC_ENABLE_TIMING
  double sum_timings_{ 0.0 };
  int n_timings_{ 0 };
#endif
};

#endif // COLLISION_CHECKER_HPP
