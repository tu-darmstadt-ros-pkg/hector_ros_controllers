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

#include <hpp/fcl/collision.h>
#include <unordered_map>

// #define SAFETY_CC_ENABLE_TIMING

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
   * - nq==1: assign directly
   * - continuous revolute (nq=2,nv=1): [cos(θ), sin(θ)]
   * - others: left at neutral with warning
   * @param joint_positions rad (rev) / m (prismatic)
   * @return true if any distance ≤ padding
   */
  bool checkCollision( const std::unordered_map<std::string, double> &joint_positions );

  /**
   * @brief Collision check for full q.
   * - FK + update placements
   * - computeDistances() with nearest points + cached GJK
   * - cache: reuse if last collision result if q change ≤ epsilon
   * @param q size == model_.nq
   * @return true if any distance ≤ padding
   */
  bool checkCollisionQ( const Eigen::VectorXd &q );

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

private:
  /**
   * @brief Publish geometry and nearest-point LINE_LIST markers.
   * - red if part of a pair with distance ≤ 0
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
  bool last_collision_state_{ false };

  double collision_padding_{ 0.0 };
  double collision_cache_epsilon_{ 1e-4 };
  bool pub_debug_geometry_{ false };

  std::unordered_map<std::string, pinocchio::JointIndex> name_to_id_;

#ifdef SAFETY_CC_ENABLE_TIMING
  double sum_timings_{ 0.0 };
  int n_timings_{ 0 };
#endif
};

#endif // COLLISION_CHECKER_HPP
