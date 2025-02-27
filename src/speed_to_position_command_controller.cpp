// Copyright 2020 PAL Robotics S.L.
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

#include "speed_to_position_command_controller/speed_to_position_command_controller.hpp"

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/logging.hpp"

namespace speed_to_position_command_controller
{
SpeedToPositionCommandController::SpeedToPositionCommandController() : SpeedToPositionControllersBase() {}

void SpeedToPositionCommandController::declare_parameters()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());
}

controller_interface::CallbackReturn SpeedToPositionCommandController::read_parameters()
{
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  if (params_.joints.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(this->get_robot_description());

  for (const auto & joint : params_.joints){
    command_interface_types_.push_back(joint + "/" + "position");
    state_interface_types_.push_back(joint + "/" + "position");
    //joint_limits_.push_back(*(urdf->getJoint(joint)->limits));

    //if(urdf->getJoint(joint)->limits){
    //  RCLCPP_INFO(get_node()->get_logger(), "Got limit for joint %s", joint.c_str());
    //  joint_limits_.insert({joint, urdf->getJoint(joint)->limits});
    joint_limits_.push_back(urdf->getJoint(joint)->limits);
    last_positions_.push_back(-1);
    }
   

  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace forward_command_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  speed_to_position_command_controller::SpeedToPositionCommandController, controller_interface::ControllerInterface)
