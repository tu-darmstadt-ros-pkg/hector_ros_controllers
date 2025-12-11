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

#include "velocity_to_position_command_controller/velocity_to_position_command_controller.hpp"

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/logging.hpp"

namespace velocity_to_position_command_controller
{
VelocityToPositionCommandController::VelocityToPositionCommandController()
    : VelocityToPositionControllersBase()
{
}

void VelocityToPositionCommandController::declare_parameters()
{
  param_listener_ = std::make_shared<ParamListener>( get_node() );
}

controller_interface::CallbackReturn VelocityToPositionCommandController::read_parameters()
{
  if ( !param_listener_ ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Error encountered during init" );
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  if ( params_.joints.empty() ) {
    RCLCPP_ERROR( get_node()->get_logger(), "'joints' parameter was empty" );
    return controller_interface::CallbackReturn::ERROR;
  }

  std::string interface_prefix = "";

  if ( !params_.passthrough_controller.empty() )
    interface_prefix = params_.passthrough_controller + "/";

  for ( const auto &joint : params_.joints ) {
    joints_.push_back( joint );
    command_interface_types_.push_back( interface_prefix + joint + "/position" );
    state_interface_types_.push_back( joint + "/" + "position" );
    state_interface_types_.push_back( joint + "/" + "velocity" );
    reference_interface_names_.push_back( joint + "/" + "velocity" );

    joint_position_states_.push_back( std::numeric_limits<double>::quiet_NaN() );
    joint_velocity_states_.push_back( std::numeric_limits<double>::quiet_NaN() );
    joint_prev_vel_states_.push_back( std::numeric_limits<double>::quiet_NaN() );
  }

  if ( params_.synchronous_groups.size() != 0 && params_.synchronous_groups.size() != joints_.size() ) {
    RCLCPP_ERROR( get_node()->get_logger(),
                  "Need to either specify a sync group for each joint or none at all" );
    return controller_interface::CallbackReturn::ERROR;
  }

  synced_joints_ = std::vector<std::vector<size_t>>( joints_.size() );
  sync_states_ = std::vector<bool>( joints_.size(), false );
  sync_offsets_ = std::vector<std::vector<double>>( joints_.size() );

  for ( size_t i = 0; i < params_.synchronous_groups.size(); i++ ) {
    joint_groups_.push_back( params_.synchronous_groups[i] );
    groups_[params_.synchronous_groups[i]].push_back( i );
  }

  for ( const auto &group : groups_ ) {
    for ( size_t i = 0; i < group.second.size(); i++ ) {
      size_t joint_idx = group.second[i];

      for ( size_t j = 0; j < group.second.size() - 1; j++ ) {
        size_t synced_joint_idx = group.second[( i + j + 1 ) % group.second.size()];

        synced_joints_[joint_idx].push_back( synced_joint_idx );
        sync_offsets_[joint_idx].push_back( std::numeric_limits<double>::quiet_NaN() );
        RCLCPP_INFO( get_node()->get_logger(),
                     "Adding synced joint %s for joint %s for vel to pos controller",
                     joints_[group.second[i]].c_str(),
                     joints_[group.second[( i + j + 1 ) % group.second.size()]].c_str() );
      }
    }
  }

  reference_interfaces_.resize( reference_interface_names_.size() );
  hold_positions_.resize( reference_interface_names_.size() );
  move_states_.resize( reference_interface_names_.size() );

  e_stop_topic_ = params_.e_stop_topic;

  kp_sync_ = params_.kp_sync;
  kp_ = params_.kp;
  kd_ = params_.kd;

  stopping_vel_threshold_ = params_.stopping_velocity_threshold;

  return controller_interface::CallbackReturn::SUCCESS;
}

} // namespace velocity_to_position_command_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS( velocity_to_position_command_controller::VelocityToPositionCommandController,
                        controller_interface::ChainableControllerInterface )
