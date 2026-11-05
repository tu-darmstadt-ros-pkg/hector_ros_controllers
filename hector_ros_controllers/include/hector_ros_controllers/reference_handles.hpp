#ifndef HECTOR_ROS_CONTROLLERS__REFERENCE_HANDLES_HPP_
#define HECTOR_ROS_CONTROLLERS__REFERENCE_HANDLES_HPP_

#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/handle.hpp"

namespace hector_ros_controllers
{

using ReferenceHandles = std::vector<hardware_interface::CommandInterface::SharedPtr>;

/// Off the control loop only, because it waits for each handle's lock.
/// @return false if a handle refused the value
inline bool invalidate_references( const ReferenceHandles &references )
{
  bool written = true;
  for ( const auto &reference : references ) {
    written &= reference->set_value( std::numeric_limits<double>::quiet_NaN(), true );
  }
  return written;
}

/// Real-time safe: copies every reference into @p values, which must have the same size.
/// @return false if a handle was locked by another thread, @p values is then incomplete
inline bool read_references( const ReferenceHandles &references, std::vector<double> &values )
{
  for ( std::size_t i = 0; i < references.size(); ++i ) {
    const auto value = references[i]->get_optional();
    if ( !value ) {
      return false;
    }
    values[i] = *value;
  }
  return true;
}

} // namespace hector_ros_controllers

#endif // HECTOR_ROS_CONTROLLERS__REFERENCE_HANDLES_HPP_
