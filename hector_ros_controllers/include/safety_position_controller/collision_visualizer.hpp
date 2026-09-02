#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <safety_position_controller/collision_checker.hpp>

namespace safety_position_controller
{

/**
 * @brief RViz markers for the collision state on "~/debug_collision_geometry".
 *
 * Rate-limited (see the constructor) and id-stable: DELETEALL only goes out when the
 * marker set changes, so RViz updates the markers in place instead of rebuilding every
 * scene node on every cycle.
 *
 * Two levels, selected per publish() call:
 * - distance lines only (cheap, realtime publisher): safety-zone pairs colored by
 *   whether the motion moves them apart, plus collision pairs.
 * - full geometry: the above with every collision shape and the safe-pair lines.
 *
 * Reads the checker's latched result; never mutates control state.
 */
class CollisionVisualizer
{
public:
  /// Where a pair sits relative to the padding / safety zone, for coloring.
  enum class Level { LinesOnly, FullGeometry };

  /**
   * @param node lifecycle node owning the marker publisher
   * @param publish_rate max marker rate [Hz]; 0 publishes on every publish() call. The
   * publisher is reliable, so a slow subscriber throttles the publisher itself — do not
   * feed it at the control rate.
   */
  explicit CollisionVisualizer( rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                                double publish_rate = 0.0 );

  /**
   * @brief Publish the markers for the checker's latest result.
   * @param checker source of geometry, distances and the latched result
   * @param level lines only, or lines plus geometry
   * @param directional per-pair gradient·velocity (one per collision pair, NaN = no
   * info): >= 0 colors a safety-zone line green (moving apart), < 0 red
   * @param safety_zone_threshold zone distance used to classify pairs [m]
   */
  void publish( const CollisionChecker &checker, Level level,
                const std::vector<double> &directional, double safety_zone_threshold );

  /// Change the marker rate [Hz] without dropping the publisher (0 = every call).
  void setPublishRate( double publish_rate )
  {
    min_period_ = publish_rate > 0.0 ? 1.0 / publish_rate : 0.0;
  }

  /// Fill in @p m's type and scale (and, for a bounding-box fallback, offset its pose)
  /// from @p go's collision geometry. Never emits a mesh resource for a primitive:
  /// pinocchio stores "BOX"/"CYLINDER"/"SPHERE" in meshPath with a unit meshScale, so
  /// doing so would draw a 1 m box in place of the real shape.
  static void describeShape( const pinocchio::GeometryObject &go, visualization_msgs::msg::Marker &m );

private:
  /// Directional derivative for one pair, or NaN when @p directional does not describe
  /// the current pair set.
  static double pairDirection( const std::vector<double> &directional, std::size_t num_pairs,
                               std::size_t pair_index );

  /// Color of a safety-zone line: green moving apart, red approaching, yellow neutral.
  static std_msgs::msg::ColorRGBA directionColor( double direction );

  /// Color of a collision line: magenta penetrating, orange inside the padding.
  static std_msgs::msg::ColorRGBA collisionColor( double distance );

  visualization_msgs::msg::Marker makeLineMarker( const std::string &ns, double thickness,
                                                  const std::string &frame_id,
                                                  const rclcpp::Time &stamp ) const;

  /// Append the geometry shape markers of every collision object.
  void appendGeometryMarkers( const CollisionChecker &checker,
                              visualization_msgs::msg::MarkerArray &array,
                              const rclcpp::Time &stamp ) const;

  /// Build the distance-line markers into @p array.
  void appendDistanceLines( const CollisionChecker &checker, Level level,
                            const std::vector<double> &directional, double safety_zone_threshold,
                            visualization_msgs::msg::MarkerArray &array,
                            const rclcpp::Time &stamp ) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::msg::MarkerArray>> pub_;
  double min_period_{ 0.0 };  ///< minimum time between publishes [s]; 0 = every call
  rclcpp::Time last_publish_; ///< stamp of the last published array
  /// Geometry markers of the last publish; a change re-sends DELETEALL so ids that no
  /// longer exist cannot linger.
  std::size_t last_geometry_count_{ 0 };
};

} // namespace safety_position_controller
