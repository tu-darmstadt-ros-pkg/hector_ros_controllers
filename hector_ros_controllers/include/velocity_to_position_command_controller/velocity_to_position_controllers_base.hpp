// Copyright 2021 Stogl Robotics Consulting UG (haftungsbescrhänkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VELOCITY_TO_POSITION_COMMAND_CONTROLLER__VELOCITY_TO_POSITION_CONTROLLERS_BASE_HPP_
#define VELOCITY_TO_POSITION_COMMAND_CONTROLLER__VELOCITY_TO_POSITION_CONTROLLERS_BASE_HPP_

#include <boost/shared_ptr.hpp>
#include <float.h>
#include <memory>
#include <string>
#include <urdf_parser/urdf_parser.h>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace velocity_to_position_command_controller
{
using CmdType = std_msgs::msg::Float64MultiArray;

/**
 * \brief Forward command controller for a set of joints and interfaces.
 *
 * This class forwards the command signal down to a set of joints or interfaces.
 *
 * Subscribes to:
 * - \b commands (std_msgs::msg::Float64) : The commands to apply.
 */
class VelocityToPositionControllersBase : public controller_interface::ChainableControllerInterface
{
public:
  VelocityToPositionControllersBase();

  ~VelocityToPositionControllersBase() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn
  on_configure( const rclcpp_lifecycle::State &previous_state ) override;

  controller_interface::CallbackReturn
  on_activate( const rclcpp_lifecycle::State &previous_state ) override;

  controller_interface::CallbackReturn
  on_deactivate( const rclcpp_lifecycle::State &previous_state ) override;

  controller_interface::return_type
  update_and_write_commands( const rclcpp::Time &time, const rclcpp::Duration &period ) override;

  bool on_set_chained_mode( bool chained_mode ) override;

protected:
  /**
   * Derived controllers have to declare parameters in this method.
   * Error handling does not have to be done. It is done in `on_init`-method of this class.
   */
  virtual void declare_parameters() = 0;

  /**
   * Derived controllers have to read parameters in this method and set `command_interface_types_`
   * variable. The variable is then used to propagate the command interface configuration to
   * controller manager. The method is called from `on_configure`-method of this class.
   *
   * It is expected that error handling of exceptions is done.
   *
   * \returns controller_interface::CallbackReturn::SUCCESS if parameters are successfully read and
   * their values are allowed, controller_interface::CallbackReturn::ERROR otherwise.
   */
  virtual controller_interface::CallbackReturn read_parameters() = 0;

  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  controller_interface::return_type
  update_reference_from_subscribers( const rclcpp::Time &time,
                                     const rclcpp::Duration &period ) override;

  std::vector<std::string> joints_;
  std::vector<std::string> reference_interface_names_;
  std::vector<std::shared_ptr<urdf::JointLimits>> joint_limits_;

  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;

  std::vector<double> last_positions_;
  std::vector<bool> stopping_;

  std::string e_stop_topic_;
  bool e_stop_active_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_buffer_ptr_;
  // rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hard_estop_sub_;
};

} // namespace velocity_to_position_command_controller

#endif // VELOCITY_TO_POSITION_COMMAND_CONTROLLER__FORWARD_CONTROLLERS_BASE_HPP_