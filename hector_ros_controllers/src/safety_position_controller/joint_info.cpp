#include "safety_position_controller/joint_info.hpp"

#include <cstdio>

#include <urdf_parser/urdf_parser.h>

namespace safety_position_controller
{

namespace
{
std::string invalid_limits_warning( const std::string &joint_name, const double lower,
                                    const double upper )
{
  char buf[256];
  std::snprintf( buf, sizeof( buf ), "Joint '%s' has invalid limits [%.3f, %.3f]",
                 joint_name.c_str(), lower, upper );
  return buf;
}

std::string velocity_fallback_warning( const std::string &joint_name, const double fallback )
{
  char buf[256];
  std::snprintf( buf, sizeof( buf ),
                 "Joint '%s' has no usable URDF velocity limit; using default_velocity_limit=%.3f",
                 joint_name.c_str(), fallback );
  return buf;
}
} // namespace

JointInfoParseResult parse_joint_infos( const std::string &urdf_xml,
                                        const std::vector<std::string> &joint_names,
                                        const double default_velocity_limit )
{
  JointInfoParseResult result;
  const auto model = urdf::parseURDF( urdf_xml );
  if ( !model ) {
    return result;
  }
  for ( const auto &[name, joint] : model->joints_ ) {
    if ( joint->type != urdf::Joint::FIXED ) {
      result.all_joint_names.push_back( name );
    }
  }

  result.joints.assign( joint_names.size(), JointInfo{} );
  for ( size_t i = 0; i < joint_names.size(); ++i ) {
    const auto &jn = joint_names[i];
    JointInfo &joint = result.joints[i];
    if ( const auto urdf_joint = model->getJoint( jn ) ) {
      switch ( urdf_joint->type ) {
      case urdf::Joint::CONTINUOUS:
        joint.type = JointType::CONTINUOUS;
        if ( urdf_joint->limits ) {
          joint.velocity_limit = urdf_joint->limits->velocity;
        }
        break;
      case urdf::Joint::REVOLUTE:
      case urdf::Joint::PRISMATIC:
        joint.type = urdf_joint->type == urdf::Joint::REVOLUTE ? JointType::REVOLUTE_BOUNDED
                                                               : JointType::PRISMATIC_BOUNDED;
        if ( urdf_joint->limits ) {
          joint.has_position_limits = true;
          joint.lower_limit = urdf_joint->limits->lower;
          joint.upper_limit = urdf_joint->limits->upper;
          joint.velocity_limit = urdf_joint->limits->velocity;
        }
        break;
      case urdf::Joint::FIXED:
        joint.type = JointType::FIXED;
        break;
      default:
        joint.type = JointType::OTHER;
        break;
      }

      if ( joint.has_position_limits && !( joint.lower_limit < joint.upper_limit ) ) {
        result.warnings.push_back(
            invalid_limits_warning( jn, joint.lower_limit, joint.upper_limit ) );
        joint.has_position_limits = false;
      }
    }

    // Without a usable velocity limit the joint would be stepped unbounded and could
    // jump to a far-away target in a single cycle.
    if ( !std::isfinite( joint.velocity_limit ) || joint.velocity_limit <= 0.0 ) {
      result.warnings.push_back( velocity_fallback_warning( jn, default_velocity_limit ) );
      joint.velocity_limit = default_velocity_limit;
    }
  }

  result.ok = true;
  return result;
}

} // namespace safety_position_controller
