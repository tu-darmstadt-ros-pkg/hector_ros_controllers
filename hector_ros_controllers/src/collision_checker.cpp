//
// Created by aljoscha-schmidt on 10/20/25.
//
#include "safety_position_controller/collision_checker.hpp"

#include <hpp/fcl/narrowphase/narrowphase.h>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "pinocchio/collision/collision.hpp"
#include "pinocchio/collision/distance.hpp"
#include <cmath>
#include <hpp/fcl/collision_data.h>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

CollisionChecker::CollisionChecker( const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
                                    bool pub_debug_geometry )
    : node_( node ), pub_debug_geometry_( pub_debug_geometry )
{
  if ( pub_debug_geometry_ ) {
    markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/debug_collision_geometry", 1 );
  }
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
      RCLCPP_INFO_STREAM( node_->get_logger(),
                          "Joint Index: " << jid << " Name: " << model_.names[jid] );
    }
    // Filter collision pairs based on controlled joints
    filterCollisionPairs( controlled_joints );
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

bool CollisionChecker::checkCollision( const std::unordered_map<std::string, double> &joint_positions )
{

  if ( model_.nq == 0 ) {
    RCLCPP_ERROR( node_->get_logger(), "Model not initialized." );
    return true;
  }

  Eigen::VectorXd q = q_default_;
  for ( const auto &[name, position] : joint_positions ) {
    const auto it = name_to_id_.find( name );
    if ( it == name_to_id_.end() ) {
      RCLCPP_WARN_THROTTLE( node_->get_logger(), *node_->get_clock(), 2000,
                            "Unknown joint '%s' (ignored).", name.c_str() );
      continue;
    }
    const pinocchio::JointIndex jid = it->second;
    const int nq_j = model_.joints[jid].nq();
    const int nv_j = model_.joints[jid].nv();
    const int iq = model_.idx_qs[jid];
    const double alpha = position;

    if ( nq_j == 1 ) {
      q[iq] = alpha;
    } else if ( nq_j == 2 && nv_j == 1 ) {
      // Revolute represented as unit complex [sin(α), cos(α)]
      const double s = std::cos( alpha );
      const double c = std::sin( alpha );
      q[iq] = s;
      q[iq + 1] = c;
      /*const double n = std::hypot( s, c );
      if ( n > 1e-12 ) {
        q[iq] /= n;
        q[iq + 1] /= n;
      }*/
    } else {
      RCLCPP_WARN_THROTTLE( node_->get_logger(), *node_->get_clock(), 2000,
                            "Joint '%s' (nq=%d,nv=%d) not supported; keeping default.",
                            model_.names[jid].c_str(), nq_j, nv_j );
    }
  }

  return checkCollisionQ( q );
}

bool CollisionChecker::checkCollisionQ( const Eigen::VectorXd &q )
{
  if ( q.size() != model_.nq ) {
    RCLCPP_ERROR( node_->get_logger(), "q size (%ld) != model.nq (%d)", long( q.size() ), model_.nq );
    return true;
  }

  pinocchio::forwardKinematics( model_, data_, q );
  pinocchio::updateGeometryPlacements( model_, data_, geom_model_, geom_data_ );
  // pinocchio::computeDistances(geom_model_, geom_data_);
  //  TODO: maybe use computeDistance -> allows to add margin

  pinocchio::computeCollisions( geom_model_, geom_data_ );

  // RCLCPP_INFO_STREAM( node_->get_logger(),
  //                     "Collision chcking " << geom_model_.collisionPairs.size() << " pairs." );
  bool in_collision = false;
  for ( std::size_t k = 0; k < geom_model_.collisionPairs.size(); ++k ) {
    const auto &cp = geom_model_.collisionPairs[k];
    const auto &o1 = geom_model_.geometryObjects[cp.first];
    const auto &o2 = geom_model_.geometryObjects[cp.second];
    const auto &result = geom_data_.collisionResults[k];
    if ( result.isCollision() ) {
      in_collision = true;
      RCLCPP_WARN_STREAM_THROTTLE( node_->get_logger(), *node_->get_clock(), 1000,
                                   "Found collision  "
                                       << " distance " << result.distance_lower_bound << " between "
                                       << o1.parentFrame << " and " << o2.parentFrame );
      break;
    }
  }

  if ( pub_debug_geometry_ )
    publishMarkers();

  return in_collision;
}

void CollisionChecker::publishMarkers() const
{
  if ( !markers_pub_ )
    return;

  visualization_msgs::msg::MarkerArray arr;
  arr.markers.reserve( geom_model_.geometryObjects.size() );
  const rclcpp::Time now = node_->now();

  std::vector<size_t> objects_in_collision;

  auto add_unique = [&]( size_t idx ) {
    if ( std::find( objects_in_collision.begin(), objects_in_collision.end(), idx ) ==
         objects_in_collision.end() )
      objects_in_collision.push_back( idx );
  };

  for ( std::size_t k = 0; k < geom_model_.collisionPairs.size(); ++k ) {
    const auto &cp = geom_model_.collisionPairs[k];
    const auto &result = geom_data_.collisionResults[k];
    if ( result.isCollision() ) {
      add_unique( cp.first );
      add_unique( cp.second );
    }
  }

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
      }
    }

    const bool in_collision = std::find( objects_in_collision.begin(), objects_in_collision.end(),
                                         i ) != objects_in_collision.end();
    if ( in_collision ) {
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
    m.lifetime = rclcpp::Duration::from_seconds( 0.2 );
    arr.markers.push_back( std::move( m ) );
  }

  markers_pub_->publish( arr );
}
