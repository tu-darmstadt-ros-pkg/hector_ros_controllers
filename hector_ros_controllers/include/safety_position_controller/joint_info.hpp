#pragma once

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace safety_position_controller
{

/// Joint type classification used for limit/unwrap logic.
enum class JointType { CONTINUOUS, REVOLUTE_BOUNDED, PRISMATIC_BOUNDED, FIXED, OTHER };

/// URDF-derived info for one controlled joint.
struct JointInfo {
  JointType type{ JointType::OTHER };
  bool has_position_limits{ false };
  double lower_limit{ std::numeric_limits<double>::lowest() };
  double upper_limit{ std::numeric_limits<double>::max() };
  double velocity_limit{ std::numeric_limits<double>::max() };
};

/// Result of parse_joint_infos(). Warnings are returned instead of logged (pure function).
struct JointInfoParseResult {
  bool ok{ false };
  std::vector<std::string> all_joint_names; ///< all non-fixed joints in the URDF
  std::vector<JointInfo> joints;            ///< in order of the requested joint names
  std::vector<std::string> warnings;
};

/**
 * @brief Parse the URDF and collect joint types/limits for the requested joints.
 * Joints with invalid position limits get has_position_limits=false; joints without a
 * usable velocity limit fall back to @p default_velocity_limit (both with a warning).
 * @param urdf_xml URDF as XML string
 * @param joint_names controlled joints, defines the order of the result
 * @param default_velocity_limit fallback velocity limit [rad/s]
 * @return see JointInfoParseResult; ok=false if the URDF could not be parsed
 */
JointInfoParseResult parse_joint_infos( const std::string &urdf_xml,
                                        const std::vector<std::string> &joint_names,
                                        double default_velocity_limit );

/**
 * @brief Unwrap target angle to the nearest equivalent around current.
 * @param current current angle [rad]
 * @param target target angle (wrapped) [rad]
 * @return unwrapped target near current
 */
inline double unwrap_to_nearest( const double current, const double target )
{
  const double k = std::round( ( current - target ) / ( 2.0 * M_PI ) );
  return target + k * ( 2.0 * M_PI );
}

/**
 * Compute the signed shortest distance between two revolute joint angles.
 *
 * @param value_a  Start angle (rad)
 * @param value_b  Target angle (rad)
 * @return Signed minimal angular difference in [-π, π].
 *         Positive → turn left (CCW), Negative → turn right (CW).
 */
inline double get_signed_distance( const double value_a, const double value_b )
{
  // Normalize into [-2π, 2π)
  double diff = std::fmod( value_b - value_a, 2.0 * M_PI );

  // Wrap into [-π, π]
  if ( diff > M_PI ) {
    diff -= 2.0 * M_PI;
  } else if ( diff < -M_PI ) {
    diff += 2.0 * M_PI;
  }

  return diff;
}

} // namespace safety_position_controller
