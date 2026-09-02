#include "safety_position_controller/collision_visualizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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

void pushLine( visualization_msgs::msg::Marker &marker, const coal::DistanceResult &dres,
               const std_msgs::msg::ColorRGBA &color )
{
  marker.points.push_back( toPoint( dres.nearest_points[0] ) );
  marker.colors.push_back( color );
  marker.points.push_back( toPoint( dres.nearest_points[1] ) );
  marker.colors.push_back( color );
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

CollisionVisualizer::CollisionVisualizer( rclcpp_lifecycle::LifecycleNode::SharedPtr node )
    : node_( std::move( node ) )
{
  auto pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/debug_collision_geometry", 1 );
  pub_ = std::make_shared<realtime_tools::RealtimePublisher<visualization_msgs::msg::MarkerArray>>(
      pub );
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
                                     const std::string &frame_id, const rclcpp::Time &stamp ) const
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
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

void CollisionVisualizer::appendGeometryMarkers( const CollisionChecker &checker,
                                                 visualization_msgs::msg::MarkerArray &array,
                                                 const rclcpp::Time &stamp ) const
{
  const auto &geom_model = checker.geometryModel();
  const auto &geom_data = checker.geometryData();
  const auto &fresh = checker.nearestPointsFresh();
  const double padding = checker.collisionPadding();

  // Color lookup: an object is red if any fresh pair it belongs to penetrates, orange
  // if one is inside the padding.
  std::vector<uint8_t> state( geom_model.geometryObjects.size(), 0 ); // 0 safe, 1 padding, 2 hit
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
    state[cp.first] = std::max( state[cp.first], level );
    state[cp.second] = std::max( state[cp.second], level );
  }

  for ( std::size_t i = 0; i < geom_model.geometryObjects.size(); ++i ) {
    const auto &go = geom_model.geometryObjects[i];
    const auto &placement = geom_data.oMg[i];

    visualization_msgs::msg::Marker m;
    m.header.frame_id = checker.rootFrame();
    m.header.stamp = stamp;
    m.ns = "collision_geometry";
    m.id = static_cast<int>( i );
    m.action = visualization_msgs::msg::Marker::ADD;
    m.lifetime = rclcpp::Duration::from_seconds( 0.0 );

    m.pose.position.x = placement.translation().x();
    m.pose.position.y = placement.translation().y();
    m.pose.position.z = placement.translation().z();
    const Eigen::Quaterniond q( placement.rotation() );
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();

    const auto *shape = go.geometry.get();
    if ( const auto *sphere = dynamic_cast<const coal::Sphere *>( shape ) ) {
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.scale.x = m.scale.y = m.scale.z = 2.0 * sphere->radius;
    } else if ( const auto *box = dynamic_cast<const coal::Box *>( shape ) ) {
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.scale.x = 2.0 * box->halfSide[0];
      m.scale.y = 2.0 * box->halfSide[1];
      m.scale.z = 2.0 * box->halfSide[2];
    } else if ( const auto *cylinder = dynamic_cast<const coal::Cylinder *>( shape ) ) {
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.scale.x = m.scale.y = 2.0 * cylinder->radius;
      m.scale.z = 2.0 * cylinder->halfLength;
    } else if ( !go.meshPath.empty() ) {
      m.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      m.mesh_resource = go.meshPath;
      m.mesh_use_embedded_materials = true;
      m.scale.x = go.meshScale[0];
      m.scale.y = go.meshScale[1];
      m.scale.z = go.meshScale[2];
    } else {
      m.type = visualization_msgs::msg::Marker::ARROW; // unknown shape
      m.scale.x = 0.05;
      m.scale.y = 0.01;
      m.scale.z = 0.01;
    }

    m.color = state[i] == 2   ? makeColor( 1.0f, 0.0f, 0.0f, 1.0f )
              : state[i] == 1 ? makeColor( 1.0f, 0.55f, 0.0f, 0.9f )
                              : makeColor( 0.7f, 0.7f, 0.7f, 0.6f );
    array.markers.push_back( std::move( m ) );
  }
}

void CollisionVisualizer::appendDistanceLines( const CollisionChecker &checker, const Level level,
                                               const std::vector<double> &directional,
                                               const double safety_zone_threshold,
                                               visualization_msgs::msg::MarkerArray &array,
                                               const rclcpp::Time &stamp ) const
{
  const auto &geom_model = checker.geometryModel();
  const auto &geom_data = checker.geometryData();
  const auto &result = checker.lastResult();
  const auto &fresh = checker.nearestPointsFresh();
  const double padding = checker.collisionPadding();
  const std::string &frame = checker.rootFrame();
  const std::size_t num_pairs = geom_model.collisionPairs.size();

  auto lines_safe = makeLineMarker( "distance_lines_safe", 0.002, frame, stamp );
  auto lines_zone = makeLineMarker( "distance_lines_safety_zone", 0.005, frame, stamp );
  auto lines_coll = makeLineMarker( "distance_lines_collision", 0.006, frame, stamp );
  lines_safe.color = makeColor( 0.5f, 0.5f, 0.5f, 0.5f );

  if ( level == Level::FullGeometry ) {
    // Every pair whose distance was computed this cycle, classified by distance.
    for ( std::size_t k = 0; k < num_pairs && k < fresh.size(); ++k ) {
      const auto &dres = geom_data.distanceResults[k];
      if ( !fresh[k] || !nearestPointsValid( dres ) ) {
        continue;
      }
      if ( dres.min_distance <= padding ) {
        pushLine( lines_coll, dres, collisionColor( dres.min_distance ) );
      } else if ( safety_zone_threshold > 0.0 && dres.min_distance < safety_zone_threshold ) {
        pushLine( lines_zone, dres, directionColor( pairDirection( directional, num_pairs, k ) ) );
      } else {
        lines_safe.points.push_back( toPoint( dres.nearest_points[0] ) );
        lines_safe.points.push_back( toPoint( dres.nearest_points[1] ) );
      }
    }
  } else {
    // Only the pairs the safety pipeline actually reasoned about.
    for ( const auto &pair : result.safety_zone_pairs ) {
      const auto &dres = geom_data.distanceResults[pair.pair_index];
      if ( !nearestPointsValid( dres ) ) {
        continue;
      }
      if ( pair.distance <= padding ) {
        pushLine( lines_coll, dres, collisionColor( pair.distance ) );
      } else {
        pushLine( lines_zone, dres,
                  directionColor( pairDirection( directional, num_pairs, pair.pair_index ) ) );
      }
    }
    // With gradients disabled there are no safety-zone pairs, so draw the closest pair
    // on its own to still show a detected collision.
    if ( result.safety_zone_pairs.empty() && result.in_collision &&
         result.min_distance_pair_index < geom_data.distanceResults.size() ) {
      const auto &dres = geom_data.distanceResults[result.min_distance_pair_index];
      if ( nearestPointsValid( dres ) ) {
        pushLine( lines_coll, dres, collisionColor( dres.min_distance ) );
      }
    }
  }

  if ( level == Level::FullGeometry ) {
    pushLineMarker( array, std::move( lines_safe ) );
  }
  pushLineMarker( array, std::move( lines_zone ) );
  pushLineMarker( array, std::move( lines_coll ) );
}

void CollisionVisualizer::publish( const CollisionChecker &checker, const Level level,
                                   const std::vector<double> &directional,
                                   const double safety_zone_threshold )
{
  if ( !pub_ || !pub_->trylock() ) {
    return; // a subscriber is still reading the previous message; skip this cycle
  }
  auto &array = pub_->msg_;
  array.markers.clear();

  const rclcpp::Time stamp = node_->now();
  visualization_msgs::msg::Marker delete_all;
  delete_all.header.frame_id = checker.rootFrame();
  delete_all.header.stamp = stamp;
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back( std::move( delete_all ) );

  if ( level == Level::FullGeometry ) {
    appendGeometryMarkers( checker, array, stamp );
  }
  appendDistanceLines( checker, level, directional, safety_zone_threshold, array, stamp );

  pub_->unlockAndPublish();
}

} // namespace safety_position_controller
