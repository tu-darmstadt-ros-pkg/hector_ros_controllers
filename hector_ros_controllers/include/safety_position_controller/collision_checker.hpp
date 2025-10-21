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

#define SAFETY_CC_ENABLE_TIMING

/// @brief CollisionChecker builds a Pinocchio + hpp-fcl collision model and performs self-collision tests.
///        - Builds from URDF + SRDF XML strings
///        - Handles revolute joints encoded as [sin(theta), cos(theta)]
///        - Publishes optional MarkerArray for debug visualization
class CollisionChecker
{
public:
  explicit CollisionChecker( const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
                             double collision_padding_, bool pub_debug_geometry = false );

  /// Initialize from URDF and SRDF XML strings
  bool initFromXml( const std::string &urdf_xml, const std::string &srdf_xml,
                    const std::vector<std::string> &controlled_joints, bool free_flyer = false );

  /// Return joint names (excluding "universe")
  std::vector<std::string> getJointNames() const;

  /// Check collision using joint names + joint positions (rad)
  bool checkCollision( const std::unordered_map<std::string, double> &joint_positions );

  /// Check collision for a given q vector
  bool checkCollisionQ( const Eigen::VectorXd &q );

  void setCollisionPadding( const double collision_padding )
  {
    collision_padding_ = collision_padding;
  }

private:
  void publishMarkers() const;
  void filterCollisionPairs( const std::vector<std::string> &controlled_joints );

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::GeometryModel geom_model_;
  pinocchio::GeometryData geom_data_;
  Eigen::VectorXd q_default_;

  double collision_padding_{ 0.0 };
  bool pub_debug_geometry_{ false };

  std::unordered_map<std::string, pinocchio::JointIndex> name_to_id_;

#ifdef SAFETY_CC_ENABLE_TIMING
  double sum_timings_{ 0.0 };
  int n_timings_{ 0 };
#endif
};

#endif // COLLISION_CHECKER_HPP
