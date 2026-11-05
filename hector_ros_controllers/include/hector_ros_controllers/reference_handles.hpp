#ifndef HECTOR_ROS_CONTROLLERS__REFERENCE_HANDLES_HPP_
#define HECTOR_ROS_CONTROLLERS__REFERENCE_HANDLES_HPP_

#include <algorithm>
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

/// Real-time safe: copies every reference into @p values, which must have the same size. A handle
/// locked by another thread keeps its entry of @p values from the previous cycle.
/// @return the number of handles that were locked
inline std::size_t read_references( const ReferenceHandles &references, std::vector<double> &values )
{
  std::size_t busy = 0;
  for ( std::size_t i = 0; i < references.size(); ++i ) {
    const auto value = references[i]->get_optional();
    if ( value ) {
      values[i] = *value;
    } else {
      ++busy;
    }
  }
  return busy;
}

/// Real-time safe: writes @p values into the references, as far as both reach. A handle locked by
/// another thread keeps its old value.
/// @return the number of handles that were locked
inline std::size_t write_references( const ReferenceHandles &references,
                                     const std::vector<double> &values )
{
  std::size_t busy = 0;
  for ( std::size_t i = 0; i < std::min( references.size(), values.size() ); ++i ) {
    if ( !references[i]->set_value( values[i] ) ) {
      ++busy;
    }
  }
  return busy;
}

} // namespace hector_ros_controllers

#endif // HECTOR_ROS_CONTROLLERS__REFERENCE_HANDLES_HPP_
