#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <safety_position_controller/collision_checker.hpp>
#include <safety_position_controller/debug_handoff.hpp>

namespace safety_position_controller
{

/**
 * @brief RViz markers for the collision state on "~/debug_collision_geometry".
 *
 * Rate-limited (see the constructor) and id-stable: DELETEALL only goes out when the
 * marker set changes, so RViz updates the markers in place instead of rebuilding every
 * scene node on every cycle.
 *
 * Two levels, selected per capture() call:
 * - distance lines only: safety-zone pairs colored by whether the motion moves them
 *   apart, plus collision pairs.
 * - full geometry: the above with every collision shape and the safe-pair lines.
 *
 * capture() copies the checker's state on the control thread; publishPending() builds
 * and publishes the markers on a non-realtime thread.
 */
class CollisionVisualizer
{
public:
  /// Marker detail: distance lines only, or lines plus every collision shape.
  enum class Level { LinesOnly, FullGeometry };

  /**
   * @param node lifecycle node owning the marker publisher
   * @param publish_rate max marker rate [Hz]; 0 captures on every capture() call. The
   * publisher is reliable, so a slow subscriber throttles the publisher itself — do not
   * feed it at the control rate.
   */
  explicit CollisionVisualizer( rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                                double publish_rate = 0.0 );

  /// Cache what the markers need from @p checker's model (frame, shapes, sizes) and
  /// size the snapshots. Call once the checker is initialized, before capture(), and
  /// not concurrently with it.
  void setModel( const CollisionChecker &checker );

  /**
   * @brief Capture the markers' content for the checker's latest result. Control
   * thread; does not allocate. Ignored for a checker whose model differs from setModel().
   * @param stamp time of the control cycle; also paces the marker rate
   * @param checker source of geometry, distances and the latched result
   * @param level lines only, or lines plus geometry
   * @param directional per-pair gradient·velocity (one per collision pair, NaN = no
   * info): >= 0 colors a safety-zone line green (moving apart), < 0 red
   * @param safety_zone_threshold zone distance used to classify pairs [m]
   */
  void capture( const rclcpp::Time &stamp, const CollisionChecker &checker, Level level,
                const std::vector<double> &directional, double safety_zone_threshold );

  /// Build and publish the markers of the last capture, once. Non-realtime thread only.
  void publishPending();

  /// Change the marker rate [Hz] without dropping the publisher (0 = every call).
  void setPublishRate( double publish_rate )
  { min_period_ = publish_rate > 0.0 ? 1.0 / publish_rate : 0.0; }

  /// Fill in @p m's type and scale (and, for a bounding-box fallback, offset its pose)
  /// from @p go's collision geometry. Never emits a mesh resource for a primitive:
  /// pinocchio stores "BOX"/"CYLINDER"/"SPHERE" in meshPath with a unit meshScale, so
  /// doing so would draw a 1 m box in place of the real shape.
  static void describeShape( const pinocchio::GeometryObject &go, visualization_msgs::msg::Marker &m );

private:
  enum class LineKind : uint8_t { Safe, Zone, Collision };

  struct Line {
    Eigen::Vector3d p1, p2; ///< nearest points of the pair
    double distance;
    double direction; ///< gradient·velocity, NaN = no info
    LineKind kind;
  };

  /// Closest contact of a geometry object over its pairs; ordered by severity.
  enum class ContactState : uint8_t { Safe, InPadding, Penetrating };

  struct GeometryPose {
    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    ContactState state;
  };

  /// One capture. Copying it into an instance with enough capacity does not allocate.
  struct Snapshot {
    rclcpp::Time stamp{ int64_t{ 0 }, RCL_ROS_TIME };
    Level level{ Level::LinesOnly };
    std::vector<GeometryPose> geometry; ///< per geometry object; empty for LinesOnly
    std::vector<Line> lines;
  };

  /// Directional derivative for one pair, or NaN when @p directional does not describe
  /// the current pair set.
  static double pairDirection( const std::vector<double> &directional, std::size_t num_pairs,
                               std::size_t pair_index );

  /// Color of a safety-zone line: green moving apart, red approaching, yellow neutral.
  static std_msgs::msg::ColorRGBA directionColor( double direction );

  /// Color of a collision line: magenta penetrating, orange inside the padding.
  static std_msgs::msg::ColorRGBA collisionColor( double distance );

  visualization_msgs::msg::Marker makeLineMarker( const std::string &ns, double thickness,
                                                  const rclcpp::Time &stamp ) const;

  static void captureGeometry( const CollisionChecker &checker, Snapshot &snap );

  static void captureLines( const CollisionChecker &checker, Level level,
                            const std::vector<double> &directional, double safety_zone_threshold,
                            Snapshot &snap );

  /// Append the geometry shape markers of every collision object.
  void appendGeometryMarkers( const Snapshot &snap, visualization_msgs::msg::MarkerArray &array,
                              const rclcpp::Time &stamp ) const;

  /// Build the markers of @p snap into @p array. Requires model_mutex_.
  void buildMarkers( const Snapshot &snap, visualization_msgs::msg::MarkerArray &array );

  /// Build the distance-line markers into @p array.
  void appendDistanceLines( const Snapshot &snap, visualization_msgs::msg::MarkerArray &array,
                            const rclcpp::Time &stamp ) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  double min_period_{ 0.0 };  ///< minimum time between captures [s]; 0 = every call
  rclcpp::Time last_publish_; ///< stamp of the last handed-over capture

  // Model sizes, read by capture(); written only by setModel().
  std::size_t num_geometries_{ 0 };
  std::size_t num_pairs_{ 0 };
  DebugHandoff<Snapshot> handoff_;

  /// Read by publishPending(); written by setModel(). Guarded by model_mutex_, which
  /// publishPending() holds while it builds.
  std::mutex model_mutex_;
  std::string root_frame_;
  /// Per geometry object: shape, scale and (for a bounding-box fallback) the local
  /// offset in pose.position, at the identity placement.
  std::vector<visualization_msgs::msg::Marker> geometry_templates_;
  /// Geometry markers of the last publish; a change re-sends DELETEALL so ids that no
  /// longer exist cannot linger.
  std::size_t last_geometry_count_{ 0 };
};

} // namespace safety_position_controller
