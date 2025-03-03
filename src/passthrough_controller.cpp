// Copyright (c) 2023, PAL Robotics
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

#include "passthrough_controller/passthrough_controller.hpp"
#include "controller_interface/helpers.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace passthrough_controller
{

controller_interface::CallbackReturn PassthroughController::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>( get_node() );
    params_ = param_listener_->get_params();
  } catch ( const std::exception &e ) {
    fprintf( stderr, "Exception thrown during init stage with message: %s \n", e.what() );
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration PassthroughController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_names_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration PassthroughController::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE };
}

controller_interface::CallbackReturn
PassthroughController::on_configure( const rclcpp_lifecycle::State & /*previous_state*/ )
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

  joints_ = params_.joints;

  if ( params_.interface_types.empty() ) {
    RCLCPP_ERROR( get_node()->get_logger(), "'interface_types' parameter was empty" );
    return controller_interface::CallbackReturn::ERROR;
  }

  command_interface_names_.reserve( joints_.size() * params_.interface_types.size() );

  for ( auto i = 0ul; i < joints_.size(); i++ ) {
    for ( auto j = 0ul; j < params_.interface_types.size(); j++ ) {
      command_interface_names_.push_back( joints_[i] + "/" + params_.interface_types[j] );
    }
  }

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF( this->get_robot_description() );

  // pre-reserve command interfaces

  RCLCPP_INFO( this->get_node()->get_logger(), "configure successful" );

  // The names should be in the same order as for command interfaces for easier matching
  for ( auto i = 0ul; i < command_interface_names_.size(); i++ )
    reference_interface_names_.push_back( command_interface_names_[i] );
  // for any case make reference interfaces size of command interfaces
  reference_interfaces_.resize( reference_interface_names_.size(),
                                std::numeric_limits<double>::quiet_NaN() );

  auto node = get_node();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>( node->get_clock() );
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>( *tf_buffer_ );

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PassthroughController::on_activate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  // ATTENTION(destogl): Shouldn't we use ordered interface all the time?
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_interfaces;
  if ( !controller_interface::get_ordered_interfaces( command_interfaces_, command_interface_names_,
                                                      std::string( "" ), ordered_interfaces ) ||
       command_interface_names_.size() != ordered_interfaces.size() ) {
    RCLCPP_ERROR( this->get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
                  command_interface_names_.size(), ordered_interfaces.size() );
    return controller_interface::CallbackReturn::ERROR;
  }

  // reset command buffer if a command came through callback when controller was inactive
  rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>>( nullptr );

  RCLCPP_INFO( this->get_node()->get_logger(), "activate successful" );

  std::fill( reference_interfaces_.begin(), reference_interfaces_.end(),
             std::numeric_limits<double>::quiet_NaN() );

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PassthroughController::on_deactivate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  // reset command buffer
  rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>>( nullptr );
  return controller_interface::CallbackReturn::SUCCESS;
}

bool PassthroughController::on_set_chained_mode( bool /*chained_mode*/ ) { return true; }

controller_interface::return_type
PassthroughController::update_and_write_commands( const rclcpp::Time & /*time*/,
                                                  const rclcpp::Duration & /*period*/ )
{
  for ( size_t i = 0; i < command_interfaces_.size(); ++i ) {
    if ( !std::isnan( reference_interfaces_[i] ) ) {
      command_interfaces_[i].set_value( reference_interfaces_[i] );
    }
  }

  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface>
PassthroughController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;

  for ( size_t i = 0; i < reference_interface_names_.size(); ++i ) {
    reference_interfaces.push_back( hardware_interface::CommandInterface(
        get_node()->get_name(), reference_interface_names_[i], &reference_interfaces_[i] ) );
  }

  return reference_interfaces;
}

void PassthroughController::set_child_links( urdf::ModelInterfaceSharedPtr urdf )
{
  joint_child_link_names_.reserve( joints_.size() );

  for ( auto i = 0ul; i < joints_.size(); i++ ) {
    joint_child_link_names_.push_back( urdf->getJoint( joints_[i] )->child_link_name );
  }
}

void PassthroughController::aggregate_collision_primitives( urdf::ModelInterfaceSharedPtr urdf )
{
  for ( std::string link_name : joint_child_link_names_ ) {

    links_aggregated_collision_primitives_[link_name] =
        std::vector<std::shared_ptr<urdf::Collision>>();

    auto collision_elements = urdf->getLink( link_name )->collision_array;
    for ( auto it = collision_elements.begin(); it != collision_elements.end(); ++it ) {
      links_aggregated_collision_primitives_[link_name].push_back( *it );
    }
  }
}

std::shared_ptr<fcl::CollisionGeometry<double>>
PassthroughController::convert_urdf_geom_to_fcl_geom( std::shared_ptr<urdf::Geometry> urdf_geom )
{

  switch ( urdf_geom->type ) {
  case urdf::Geometry::SPHERE :
  {
    std::shared_ptr<urdf::Sphere> urdf_sphere = std::static_pointer_cast<urdf::Sphere>( urdf_geom );
    std::shared_ptr<fcl::Sphere<double>> fcl_sphere =
        std::make_shared<fcl::Sphere<double>>( urdf_sphere->radius );
    return std::static_pointer_cast<fcl::CollisionGeometry<double>>( fcl_sphere );
  }
  case urdf::Geometry::BOX:
  {
    std::shared_ptr<urdf::Box> urdf_box = std::static_pointer_cast<urdf::Box>( urdf_geom );
    std::shared_ptr<fcl::Box<double>> fcl_box = std::make_shared<fcl::Box<double>>( urdf_box->dim.x, urdf_box->dim.y, urdf_box->dim.z );
    return std::static_pointer_cast<fcl::CollisionGeometry<double>>( fcl_box );
  }
  case urdf::Geometry::CYLINDER :
  {
    std::shared_ptr<urdf::Cylinder> urdf_cylinder =
        std::static_pointer_cast<urdf::Cylinder>( urdf_geom );
    std::shared_ptr<fcl::Cylinder<double>> fcl_cylinder =
        std::make_shared<fcl::Cylinder<double>>( urdf_cylinder->radius, urdf_cylinder->length );
    return std::static_pointer_cast<fcl::CollisionGeometry<double>>( fcl_cylinder );
  }
  default:{
    return nullptr;
  }
  }
}

controller_interface::return_type
PassthroughController::update_reference_from_subscribers( const rclcpp::Time & /*time*/,
                                                          const rclcpp::Duration & /*period*/ )
{
  auto joint_commands = rt_buffer_ptr_.readFromRT();
  // message is valid
  if ( !( !joint_commands || !( *joint_commands ) ) ) {
    if ( reference_interfaces_.size() != ( *joint_commands )->data.size() ) {
      RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(), *( get_node()->get_clock() ), 1000,
          "command size (%zu) does not match number of reference interfaces (%zu)",
          ( *joint_commands )->data.size(), reference_interfaces_.size() );
      return controller_interface::return_type::ERROR;
    }
    reference_interfaces_ = ( *joint_commands )->data;
  }

  return controller_interface::return_type::OK;
}

} // namespace passthrough_controller

PLUGINLIB_EXPORT_CLASS( passthrough_controller::PassthroughController,
                        controller_interface::ChainableControllerInterface )
