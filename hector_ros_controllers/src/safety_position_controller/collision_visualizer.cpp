#include "safety_position_controller/collision_visualizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <coal/collision_object.h>
#include <coal/shape/geometric_shapes.h>

namespace safety_position_controller
{

namespace
{
/// |g^T v| below this is tangential motion or standstill; without the band, numerical
/// noise around zero makes the line color flicker red/green.
constexpr double kDirNeutralBand = 1e-3; // [m/s]

std_msgs::msg::ColorRGBA makeColor( const float r, const float g, const float b, const float a )
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

bool nearestPointsValid( const coal::DistanceResult &dres )
{
  return !dres.nearest_points[0].hasNaN() && !dres.nearest_points[1].hasNaN() &&
         dres.nearest_points[0].allFinite() && dres.nearest_points[1].allFinite();
}

geometry_msgs::msg::Point toPoint( const Eigen::Vector3d &v )
{
  geometry_msgs::msg::Point p;
  p.x = v[0];
  p.y = v[1];
  p.z = v[2];
  return p;
}

void pushLine( visualization_msgs::msg::Marker &marker, const Eigen::Vector3d &p1,
               const Eigen::Vector3d &p2, const std_msgs::msg::ColorRGBA &color )
{
  marker.points.push_back( toPoint( p1 ) );
  marker.colors.push_back( color );
  marker.points.push_back( toPoint( p2 ) );
  marker.colors.push_back( color );
}

/// A mesh resource RViz can actually retrieve, or "" when @p path is not one. pinocchio
/// fills meshPath with the primitive's NAME ("BOX", "CYLINDER", "SPHERE", "CAPSULE") for
/// non-mesh geometry, and with a resolved absolute filesystem path for real meshes —
/// resource_retriever needs a scheme, so a bare path has to become a file:// URI.
std::string meshResourceUri( const std::string &path )
{
  if ( path.empty() || path.find( '/' ) == std::string::npos ) {
    return {}; // primitive placeholder name, not a path
  }
  if ( path.find( "://" ) != std::string::npos ) {
    return path; // already a URI (package://, file://, http://)
  }
  return path.front() == '/' ? "file://" + path : std::string{};
}

/// RViz ignores a LINE_LIST update with no points (the old lines persist), so an empty
/// marker has to DELETE instead.
void pushLineMarker( visualization_msgs::msg::MarkerArray &array,
                     visualization_msgs::msg::Marker &&marker )
{
  if ( marker.points.empty() ) {
    marker.action = visualization_msgs::msg::Marker::DELETE;
  }
  array.markers.push_back( std::move( marker ) );
}
} // namespace

CollisionVisualizer::CollisionVisualizer( rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                                          const double publish_rate )
    : node_( std::move( node ) ), min_period_( publish_rate > 0.0 ? 1.0 / publish_rate : 0.0 ),
      last_publish_( 0, 0, RCL_ROS_TIME )
{
  pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/debug_collision_geometry", 1 );
}

void CollisionVisualizer::setModel( const CollisionChecker &checker )
{
  const auto &geom_model = checker.geometryModel();
  num_geometries_ = geom_model.geometryObjects.size();
  num_pairs_ = geom_model.collisionPairs.size();
  // A new model starts without a capture: the last one belongs to the previous
  // activation and counts as published.
  const std::lock_guard<std::mutex> lock( model_mutex_ );
  const auto reset = [this]( Snapshot &snap ) {
    snap.stamp_ns = 0;
    snap.geometry.clear();
    snap.lines.clear();
    snap.geometry.reserve( num_geometries_ );
    snap.lines.reserve( num_pairs_ + 1 ); // + the closest pair drawn on its own
  };
  reset( staging_ );
  box_.set( [&]( Snapshot &snap ) {
    reset( snap );
    snap.seq = staging_.seq;
  } );
  published_seq_ = staging_.seq;

  root_frame_ = checker.rootFrame();
  geometry_templates_.clear();
  for ( std::size_t i = 0; i < num_geometries_; ++i ) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = root_frame_;
    m.ns = "collision_geometry";
    m.id = static_cast<int>( i );
    m.action = visualization_msgs::msg::Marker::ADD;
    m.lifetime = rclcpp::Duration::from_seconds( 0.0 );
    m.pose.orientation.w = 1.0;
    describeShape( geom_model.geometryObjects[i], m );
    geometry_templates_.push_back( std::move( m ) );
  }
}

double CollisionVisualizer::pairDirection( const std::vector<double> &directional,
                                           const std::size_t num_pairs, const std::size_t pair_index )
{
  return directional.size() == num_pairs && pair_index < num_pairs
             ? directional[pair_index]
             : std::numeric_limits<double>::quiet_NaN();
}

std_msgs::msg::ColorRGBA CollisionVisualizer::directionColor( const double direction )
{
  if ( std::isnan( direction ) || std::abs( direction ) <= kDirNeutralBand ) {
    return makeColor( 1.0f, 0.8f, 0.0f, 0.9f ); // no info / tangential: yellow
  }
  return direction > 0.0 ? makeColor( 0.0f, 1.0f, 0.0f, 1.0f )  // moving apart: green
                         : makeColor( 1.0f, 0.0f, 0.0f, 1.0f ); // approaching: red
}

std_msgs::msg::ColorRGBA CollisionVisualizer::collisionColor( const double distance )
{
  return distance <= 0.0 ? makeColor( 1.0f, 0.0f, 1.0f, 1.0f )   // penetrating: magenta
                         : makeColor( 1.0f, 0.55f, 0.0f, 1.0f ); // inside padding: orange
}

visualization_msgs::msg::Marker
CollisionVisualizer::makeLineMarker( const std::string &ns, const double thickness,
                                     const rclcpp::Time &stamp ) const
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = root_frame_;
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::LINE_LIST;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = thickness;
  m.lifetime = rclcpp::Duration::from_seconds( 0.0 );
  m.pose.orientation.w = 1.0;
  return m;
}

void CollisionVisualizer::describeShape( const pinocchio::GeometryObject &go,
                                         visualization_msgs::msg::Marker &m )
{
  // Switch on the node type rather than dynamic_cast: it is a plain virtual call, so it
  // cannot be defeated by RTTI that does not unify across the coal/pinocchio/plugin
  // library boundary, and it also covers the shapes that have no C++ class here.
  const auto *shape = go.geometry.get();
  switch ( shape != nullptr ? shape->getNodeType() : coal::BV_UNKNOWN ) {
  case coal::GEOM_SPHERE: {
    const auto &sphere = static_cast<const coal::Sphere &>( *shape );
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.scale.x = m.scale.y = m.scale.z = 2.0 * sphere.radius;
    return;
  }
  case coal::GEOM_BOX: {
    const auto &box = static_cast<const coal::Box &>( *shape );
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.scale.x = 2.0 * box.halfSide[0];
    m.scale.y = 2.0 * box.halfSide[1];
    m.scale.z = 2.0 * box.halfSide[2];
    return;
  }
  case coal::GEOM_CYLINDER: {
    const auto &cylinder = static_cast<const coal::Cylinder &>( *shape );
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.scale.x = m.scale.y = 2.0 * cylinder.radius;
    m.scale.z = 2.0 * cylinder.halfLength;
    return;
  }
  case coal::GEOM_CAPSULE: {
    // No capsule marker exists; the enclosing cylinder is the honest approximation
    // (it over-covers the caps, so nothing looks safer than it is).
    const auto &capsule = static_cast<const coal::Capsule &>( *shape );
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.scale.x = m.scale.y = 2.0 * capsule.radius;
    m.scale.z = 2.0 * ( capsule.halfLength + capsule.radius );
    return;
  }
  case coal::GEOM_ELLIPSOID: {
    const auto &ellipsoid = static_cast<const coal::Ellipsoid &>( *shape );
    m.type = visualization_msgs::msg::Marker::SPHERE; // scaled sphere == ellipsoid
    m.scale.x = 2.0 * ellipsoid.radii[0];
    m.scale.y = 2.0 * ellipsoid.radii[1];
    m.scale.z = 2.0 * ellipsoid.radii[2];
    return;
  }
  case coal::BV_AABB:
  case coal::BV_OBB:
  case coal::BV_RSS:
  case coal::BV_kIOS:
  case coal::BV_OBBRSS:
  case coal::BV_KDOP16:
  case coal::BV_KDOP18:
  case coal::BV_KDOP24:
  case coal::GEOM_CONVEX: {
    // A real mesh. meshPath is a resolved FILESYSTEM path, which RViz cannot retrieve
    // without a scheme, so give it a file:// URI. Anything else falls through to the
    // bounding box below rather than being drawn wrong.
    if ( const std::string uri = meshResourceUri( go.meshPath ); !uri.empty() ) {
      m.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      m.mesh_resource = uri;
      m.mesh_use_embedded_materials = false; // the state color must stay visible
      m.scale.x = go.meshScale[0];
      m.scale.y = go.meshScale[1];
      m.scale.z = go.meshScale[2];
      return;
    }
    break;
  }
  default:
    break;
  }

  // Unrepresentable (height field, octree, halfspace, ...) or a mesh with no usable
  // path: draw its local bounding box. NEVER fall back to meshPath here — pinocchio
  // stores the literal strings "BOX"/"CYLINDER"/"SPHERE" with meshScale (1,1,1) for
  // primitives, so treating it as a mesh resource asks for a 1 m unit box.
  coal::AABB aabb;
  if ( shape != nullptr ) {
    aabb = shape->aabb_local;
  }
  const Eigen::Vector3d size = aabb.max_ - aabb.min_;
  const Eigen::Vector3d center = 0.5 * ( aabb.max_ + aabb.min_ );
  m.type = visualization_msgs::msg::Marker::CUBE;
  m.scale.x = std::max( size[0], 1e-4 );
  m.scale.y = std::max( size[1], 1e-4 );
  m.scale.z = std::max( size[2], 1e-4 );
  // The AABB is in the object's local frame; shift the marker onto its center.
  const Eigen::Vector3d offset = Eigen::Quaterniond( m.pose.orientation.w, m.pose.orientation.x,
                                                     m.pose.orientation.y, m.pose.orientation.z ) *
                                 center;
  m.pose.position.x += offset[0];
  m.pose.position.y += offset[1];
  m.pose.position.z += offset[2];
}

void CollisionVisualizer::captureGeometry( const CollisionChecker &checker, Snapshot &snap )
{
  const auto &geom_model = checker.geometryModel();
  const auto &geom_data = checker.geometryData();
  const auto &fresh = checker.nearestPointsFresh();
  const double padding = checker.collisionPadding();

  snap.geometry.resize( geom_model.geometryObjects.size() );
  for ( std::size_t i = 0; i < snap.geometry.size(); ++i ) {
    snap.geometry[i].translation = geom_data.oMg[i].translation();
    snap.geometry[i].rotation = geom_data.oMg[i].rotation();
    snap.geometry[i].state = 0;
  }
  // An object is red if any fresh pair it belongs to penetrates, orange if one is inside
  // the padding.
  for ( std::size_t k = 0; k < geom_model.collisionPairs.size() && k < fresh.size(); ++k ) {
    if ( !fresh[k] ) {
      continue; // pruned by the broadphase: its distance is from an earlier cycle
    }
    const double distance = geom_data.distanceResults[k].min_distance;
    const uint8_t level = distance <= 0.0 ? 2 : ( distance <= padding ? 1 : 0 );
    if ( level == 0 ) {
      continue;
    }
    const auto &cp = geom_model.collisionPairs[k];
    snap.geometry[cp.first].state = std::max( snap.geometry[cp.first].state, level );
    snap.geometry[cp.second].state = std::max( snap.geometry[cp.second].state, level );
  }
}

void CollisionVisualizer::captureLines( const CollisionChecker &checker, const Level level,
                                        const std::vector<double> &directional,
                                        const double safety_zone_threshold, Snapshot &snap )
{
  const auto &geom_data = checker.geometryData();
  const auto &result = checker.lastResult();
  const auto &fresh = checker.nearestPointsFresh();
  const double padding = checker.collisionPadding();
  const std::size_t num_pairs = checker.geometryModel().collisionPairs.size();
  const double kNoDirection = std::numeric_limits<double>::quiet_NaN();
  const auto push = [&snap]( const coal::DistanceResult &dres, const double distance,
                             const double direction, const LineKind kind ) {
    snap.lines.push_back(
        { dres.nearest_points[0], dres.nearest_points[1], distance, direction, kind } );
  };

  if ( level == Level::FullGeometry ) {
    // Every pair whose distance was computed this cycle, classified by distance.
    for ( std::size_t k = 0; k < num_pairs && k < fresh.size(); ++k ) {
      const auto &dres = geom_data.distanceResults[k];
      if ( !fresh[k] || !nearestPointsValid( dres ) ) {
        continue;
      }
      if ( dres.min_distance <= padding ) {
        push( dres, dres.min_distance, kNoDirection, LineKind::Collision );
      } else if ( safety_zone_threshold > 0.0 && dres.min_distance < safety_zone_threshold ) {
        push( dres, dres.min_distance, pairDirection( directional, num_pairs, k ), LineKind::Zone );
      } else {
        push( dres, dres.min_distance, kNoDirection, LineKind::Safe );
      }
    }
    return;
  }
  // Only the pairs the safety pipeline actually reasoned about.
  for ( const auto &pair : result.safety_zone_pairs ) {
    const auto &dres = geom_data.distanceResults[pair.pair_index];
    if ( !nearestPointsValid( dres ) ) {
      continue;
    }
    if ( pair.distance <= padding ) {
      push( dres, pair.distance, kNoDirection, LineKind::Collision );
    } else {
      push( dres, pair.distance, pairDirection( directional, num_pairs, pair.pair_index ),
            LineKind::Zone );
    }
  }
  // With gradients disabled there are no safety-zone pairs, so draw the closest pair
  // on its own to still show a detected collision.
  if ( result.safety_zone_pairs.empty() && result.in_collision &&
       result.min_distance_pair_index < geom_data.distanceResults.size() ) {
    const auto &dres = geom_data.distanceResults[result.min_distance_pair_index];
    if ( nearestPointsValid( dres ) ) {
      push( dres, dres.min_distance, kNoDirection, LineKind::Collision );
    }
  }
}

void CollisionVisualizer::appendGeometryMarkers( const Snapshot &snap,
                                                 visualization_msgs::msg::MarkerArray &array,
                                                 const rclcpp::Time &stamp ) const
{
  for ( std::size_t i = 0; i < snap.geometry.size() && i < geometry_templates_.size(); ++i ) {
    const GeometryPose &pose = snap.geometry[i];
    visualization_msgs::msg::Marker m = geometry_templates_[i];
    m.header.stamp = stamp;

    const Eigen::Quaterniond q( pose.rotation );
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();
    // The template's position is the shape's local offset (bounding-box fallback).
    const Eigen::Vector3d offset( m.pose.position.x, m.pose.position.y, m.pose.position.z );
    m.pose.position = toPoint( pose.translation + q * offset );

    m.color = pose.state == 2   ? makeColor( 1.0f, 0.0f, 0.0f, 1.0f )
              : pose.state == 1 ? makeColor( 1.0f, 0.55f, 0.0f, 0.9f )
                                : makeColor( 0.7f, 0.7f, 0.7f, 0.6f );
    array.markers.push_back( std::move( m ) );
  }
}

void CollisionVisualizer::appendDistanceLines( const Snapshot &snap,
                                               visualization_msgs::msg::MarkerArray &array,
                                               const rclcpp::Time &stamp ) const
{
  auto lines_safe = makeLineMarker( "distance_lines_safe", 0.002, stamp );
  auto lines_zone = makeLineMarker( "distance_lines_safety_zone", 0.005, stamp );
  auto lines_coll = makeLineMarker( "distance_lines_collision", 0.006, stamp );
  lines_safe.color = makeColor( 0.5f, 0.5f, 0.5f, 0.5f );

  for ( const Line &line : snap.lines ) {
    switch ( line.kind ) {
    case LineKind::Collision:
      pushLine( lines_coll, line.p1, line.p2, collisionColor( line.distance ) );
      break;
    case LineKind::Zone:
      pushLine( lines_zone, line.p1, line.p2, directionColor( line.direction ) );
      break;
    case LineKind::Safe:
      lines_safe.points.push_back( toPoint( line.p1 ) );
      lines_safe.points.push_back( toPoint( line.p2 ) );
      break;
    }
  }

  if ( snap.level == Level::FullGeometry ) {
    pushLineMarker( array, std::move( lines_safe ) );
  }
  pushLineMarker( array, std::move( lines_zone ) );
  pushLineMarker( array, std::move( lines_coll ) );
}

void CollisionVisualizer::capture( const CollisionChecker &checker, const Level level,
                                   const std::vector<double> &directional,
                                   const double safety_zone_threshold )
{
  if ( checker.geometryModel().geometryObjects.size() != num_geometries_ ||
       checker.geometryModel().collisionPairs.size() != num_pairs_ ) {
    return; // setModel() did not size the snapshots for this model
  }
  const rclcpp::Time stamp = node_->now();
  if ( min_period_ > 0.0 && last_publish_.nanoseconds() != 0 ) {
    const double elapsed = ( stamp - last_publish_ ).seconds();
    // A backwards jump (sim clock reset) publishes instead of stalling until it catches up.
    if ( elapsed >= 0.0 && elapsed < min_period_ ) {
      return;
    }
  }

  Snapshot &snap = staging_;
  snap.stamp_ns = stamp.nanoseconds();
  snap.level = level;
  snap.geometry.clear();
  snap.lines.clear();
  if ( level == Level::FullGeometry ) {
    captureGeometry( checker, snap );
  }
  captureLines( checker, level, directional, safety_zone_threshold, snap );
  ++snap.seq;
  if ( box_.try_set( snap ) ) { // busy: publishPending() is copying; skip this cycle
    last_publish_ = stamp;
  }
}

void CollisionVisualizer::publishPending()
{
  // Built under the lock, published outside it: a slow subscriber must not hold up an
  // activation waiting in setModel().
  visualization_msgs::msg::MarkerArray array;
  {
    const std::lock_guard<std::mutex> lock( model_mutex_ );
    if ( !buildPending( array ) ) {
      return;
    }
  }
  pub_->publish( array );
}

bool CollisionVisualizer::buildPending( visualization_msgs::msg::MarkerArray &array )
{
  box_.get( pending_ );
  if ( pending_.seq == published_seq_ ) {
    return false;
  }
  published_seq_ = pending_.seq;
  const rclcpp::Time stamp( pending_.stamp_ns, RCL_ROS_TIME );

  // DELETEALL only when the marker set can have shrunk. Sending it every cycle makes
  // RViz destroy and rebuild every scene node between two frames, which both flickers
  // and dwarfs the cost of the update it is meant to deliver. Stable ids otherwise let
  // the markers be updated in place; the line markers DELETE themselves when empty.
  const std::size_t geometry_count = pending_.geometry.size();
  if ( geometry_count != last_geometry_count_ ) {
    visualization_msgs::msg::Marker delete_all;
    delete_all.header.frame_id = root_frame_;
    delete_all.header.stamp = stamp;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back( std::move( delete_all ) );
    last_geometry_count_ = geometry_count;
  }

  appendGeometryMarkers( pending_, array, stamp );
  appendDistanceLines( pending_, array, stamp );
  return true;
}

} // namespace safety_position_controller
