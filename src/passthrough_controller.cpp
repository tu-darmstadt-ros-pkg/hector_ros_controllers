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
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for ( std::string joint_name : active_joints_ ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Add state interface %s", joint_name.c_str() );
    state_interfaces_config.names.push_back( joint_name + "/position" );
  }

  return state_interfaces_config;
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

  controlled_joints_ = params_.joints;

  if ( params_.interface_types.empty() ) {
    RCLCPP_ERROR( get_node()->get_logger(), "'interface_types' parameter was empty" );
    return controller_interface::CallbackReturn::ERROR;
  }

  // Use half since collision geom size increases are applied to source and target collision
  safety_margin_ = params_.safety_margin / 2;

  command_interface_names_.reserve( controlled_joints_.size() * params_.interface_types.size() );

  for ( auto i = 0ul; i < controlled_joints_.size(); i++ ) {
    for ( auto j = 0ul; j < params_.interface_types.size(); j++ ) {
      command_interface_names_.push_back( controlled_joints_[i] + "/" + params_.interface_types[j] );
    }
  }

  // pre-reserve command interfaces
  // The names should be in the same order as for command interfaces for easier matching
  for ( auto i = 0ul; i < command_interface_names_.size(); i++ )
    reference_interface_names_.push_back( command_interface_names_[i] );
  // for any case make reference interfaces size of command interfaces
  reference_interfaces_.resize( reference_interface_names_.size(),
                                std::numeric_limits<double>::quiet_NaN() );

  prev_command_vals = std::vector<double>( reference_interface_names_.size() );

  // marker_pub_ = get_node()->create_publisher<visualization_msgs::msg::Marker>( "Origin", 40 );

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF( this->get_robot_description() );

  set_joint_infos( urdf );

  set_potentially_colliding_links();

  collect_collision_primitives( urdf );

  RCLCPP_INFO( this->get_node()->get_logger(), "configure successful" );

  return controller_interface::CallbackReturn::SUCCESS;
}

void PassthroughController::create_debug_marker( int id, std::shared_ptr<urdf::Geometry> urdf_geom )
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = get_node()->get_clock().get()->now();
  marker.id = id;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;

  switch ( urdf_geom->type ) {
  case urdf::Geometry::SPHERE: {
    std::shared_ptr<urdf::Sphere> urdf_sphere = std::static_pointer_cast<urdf::Sphere>( urdf_geom );
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    break;
  }
  case urdf::Geometry::BOX: {
    std::shared_ptr<urdf::Box> urdf_box = std::static_pointer_cast<urdf::Box>( urdf_geom );
    marker.type = visualization_msgs::msg::Marker::CUBE;
    break;
  }
  case urdf::Geometry::CYLINDER: {
    std::shared_ptr<urdf::Cylinder> urdf_cylinder =
        std::static_pointer_cast<urdf::Cylinder>( urdf_geom );
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    break;
  }
  default: {
    return;
  }
  }

  marker_pub_->publish( marker );
}

void PassthroughController::modify_debug_marker( int id, const fcl::Transform3d &t,
                                                 std::string link_name,
                                                 std::shared_ptr<fcl::CollisionGeometry<double>> geom )
{
  visualization_msgs::msg::Marker marker;
  marker.id = id;
  marker.header.frame_id = "chassis_link";
  marker.header.stamp = get_node()->get_clock().get()->now();
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = t( 0, 3 );
  marker.pose.position.y = t( 1, 3 );
  marker.pose.position.z = t( 2, 3 );

  fcl::Quaterniond q = fcl::Quaterniond( t.linear() );
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 0.5;

  if ( link_name == "flipper_fl_link" ) {
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
  }

  if ( link_name == "flipper_fr_link" ) {
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.5;
  }

  if ( link_name == "flipper_bl_link" ) {
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
  }

  if ( link_name == "flipper_br_link" ) {
    marker.color.r = 0.5;
    marker.color.g = 0.0;
    marker.color.b = 0.5;
    marker.color.a = 0.5;
  }

  switch ( geom->getNodeType() ) {
  case fcl::NODE_TYPE::GEOM_SPHERE: {
    std::shared_ptr<fcl::Sphered> sphere = std::static_pointer_cast<fcl::Sphered>( geom );
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.scale.x = 2 * sphere->radius;
    marker.scale.y = 2 * sphere->radius;
    marker.scale.z = 2 * sphere->radius;
    break;
  }
  case fcl::NODE_TYPE::GEOM_BOX: {
    std::shared_ptr<fcl::Boxd> box = std::static_pointer_cast<fcl::Boxd>( geom );
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.scale.x = box->side[0];
    marker.scale.y = box->side[1];
    marker.scale.z = box->side[2];
    break;
  }
  case fcl::NODE_TYPE::GEOM_CYLINDER: {
    std::shared_ptr<fcl::Cylinderd> cylinder = std::static_pointer_cast<fcl::Cylinderd>( geom );
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.scale.x = 2 * cylinder->radius;
    marker.scale.y = 2 * cylinder->radius;
    marker.scale.z = 2 * cylinder->lz;
    break;
  }
  default: {
    break;
  }
  }
  marker_pub_->publish( marker );
}

void PassthroughController::set_potentially_colliding_links()
{
  for ( auto i = 0ul; i < controlled_joints_.size(); i++ ) {

    potentially_colliding_links_.push_back( std::vector<long unsigned>{} );

    for ( auto j = 0ul; j < controlled_joints_.size(); j++ ) {
      if ( params_.collision_mat[i * controlled_joints_.size() + j] == 1 )
        potentially_colliding_links_[i].push_back( j );
    }

    RCLCPP_INFO( this->get_node()->get_logger(), "Coll targets for joint %s : %i",
                 controlled_joints_[i].c_str(), (int)potentially_colliding_links_[i].size() );
  }
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

void PassthroughController::update_joint_angles()
{
  for ( auto index = 0ul; index < q_indices_active.size(); ++index ) {
    auto state_val = state_interfaces_[index].get_value();
    if ( !std::isnan( state_val ) )
      joint_angles_[q_indices_active[index]] = state_val;
  }
}

bool PassthroughController::write_valid_reference_commands( std::vector<bool> &collision_results )
{
  bool success = true;
  for ( size_t i = 0; i < command_interfaces_.size(); ++i ) {
    if ( !std::isnan( reference_interfaces_[i] ) ) {
      if ( !collision_results[i] ) {
        success = success && command_interfaces_[i].set_value( reference_interfaces_[i] );
        prev_command_vals[i] = reference_interfaces_[i];
      } else {
        success = success && command_interfaces_[i].set_value( prev_command_vals[i] );
      }
    }
  }

  return success;
}

controller_interface::return_type
PassthroughController::update_and_write_commands( const rclcpp::Time & /*time*/,
                                                  const rclcpp::Duration & )
{
  std::chrono::time_point<std::chrono::system_clock> start, end;
 
  start = std::chrono::system_clock::now();

  update_joint_angles();

  std::vector<bool> collision_results = std::vector<bool>( controlled_joints_.size() );

  for ( size_t i = 0; i < controlled_joints_.size(); i++ ) {

    std::string child_link = joint_child_link_names_[i];
    std::string joint = controlled_joints_[i];

    // skip if no command received from high level controller
    if ( std::isnan( reference_interfaces_[i] ) )
      continue;
    // Set hypothetical joint position
    joint_angles_[q_indices_controlled[i]] = reference_interfaces_[i];

    auto source_collisions = link_collision_primitives_[child_link];

    bool any_collision = false;

    for ( unsigned long coll_target : potentially_colliding_links_[i] ) {

      std::string target_link = joint_child_link_names_[coll_target];

      try {
        fcl::Transform3d base_to_child_l = get_transform_from_base_link( child_link );
        fcl::Transform3d base_to_target_l = get_transform_from_base_link( target_link );

        any_collision = any_collision ||
                        pairwise_primitive_collision_check( source_collisions,
                                                            link_collision_primitives_[target_link],
                                                            base_to_child_l, base_to_target_l );

        RCLCPP_INFO( this->get_node()->get_logger(), "Collision: %i", any_collision );

      } catch ( ... ) {
        RCLCPP_INFO( this->get_node()->get_logger(), "Error for source %s to target %s.",
                     joint.c_str(), target_link.c_str() );
      }

      // Some link is colliding -> skip other checks
      if ( any_collision )
        break;
    }

    collision_results[i] = any_collision;
  }

  bool success = write_valid_reference_commands( collision_results );

  end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;

  avg_update_dur = (avg_update_dur + elapsed_seconds.count())/2;

  RCLCPP_INFO( this->get_node()->get_logger(), "Update loop took: %f s", avg_update_dur);

  if ( !success )
    return controller_interface::return_type::ERROR;

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

void PassthroughController::log_transform( const fcl::Transform3d &t, const std::string title )
{
  RCLCPP_INFO( get_node()->get_logger(),
               "%s \n| %f %f %f %f |\n| %f %f %f %f |\n| %f %f %f %f |\n| %f %f %f %f |",
               title.c_str(), t( 0, 0 ), t( 0, 1 ), t( 0, 2 ), t( 0, 3 ), t( 1, 0 ), t( 1, 1 ),
               t( 1, 2 ), t( 1, 3 ), t( 2, 0 ), t( 2, 1 ), t( 2, 2 ), t( 2, 3 ), t( 3, 0 ),
               t( 3, 1 ), t( 3, 2 ), t( 3, 3 ) );
}

void PassthroughController::set_joint_infos( const urdf::ModelInterfaceSharedPtr urdf )
{
  joint_child_link_names_.reserve( controlled_joints_.size() );

  for ( auto i = 0ul; i < controlled_joints_.size(); i++ ) {
    auto urdf_joint = urdf->getJoint( controlled_joints_[i] );
    joint_child_link_names_.push_back( urdf_joint->child_link_name );
  }

  transformTree_ = std::make_shared<ad_kinematics::Tree>( urdf );

  active_joints_ = transformTree_->getActiveJointNames();
  q_indices_controlled = transformTree_->getJointQIndices( controlled_joints_ );
  q_indices_active = transformTree_->getJointQIndices( active_joints_ );

  for ( auto idx = 0ul; idx < active_joints_.size(); idx++ ) joint_angles_.push_back( 0.0 );

  joint_angles_.resize( active_joints_.size() );
}

void PassthroughController::collect_collision_primitives( const urdf::ModelInterfaceSharedPtr urdf )
{
  int id = 0;
  for ( std::string &link_name : joint_child_link_names_ ) {

    link_collision_primitives_[link_name] = std::vector<CollisionPrimitive>();
    auto urdf_collision_elements = urdf->getLink( link_name )->collision_array;

    for ( auto it = urdf_collision_elements.begin(); it != urdf_collision_elements.end(); ++it ) {

      auto coll_obj = std::make_shared<fcl::CollisionObjectd>(
          urdf_geom_to_fcl_geom( ( *it )->geometry ), fcl::Transform3d::Identity() );
      auto joint_coll_transform =
          std::make_shared<fcl::Transform3d>( pose_to_fcl_transform( ( *it )->origin ) );

      link_collision_primitives_[link_name].push_back(
          std::make_tuple( coll_obj, joint_coll_transform, id ) );
      id++;
    }
  }
}

fcl::Transform3d PassthroughController::get_transform_from_base_link( std::string &link )
{
  return kinematics_transform_to_fcl_transform(
      transformTree_->computeTransform<double>( link, joint_angles_ ) );
}

fcl::Transform3d PassthroughController::pose_to_fcl_transform( const urdf::Pose &urdf_pose )
{
  return create_fcl_transform_from_data(
      urdf_pose.rotation.w, urdf_pose.rotation.x, urdf_pose.rotation.y, urdf_pose.rotation.z,
      urdf_pose.position.x, urdf_pose.position.y, urdf_pose.position.z );
}

fcl::Transform3d
PassthroughController::geom_transform_to_fcl_transform( const geometry_msgs::msg::Transform &transform )
{
  return create_fcl_transform_from_data(
      transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z,
      transform.translation.x, transform.translation.y, transform.translation.z );
}

fcl::Transform3d PassthroughController::kinematics_transform_to_fcl_transform(
    const ad_kinematics::Transform<double> &transform )
{
  fcl::Transform3d pose = fcl::Transform3d::Identity();
  pose.linear() = transform.rotation.toRotationMatrix();
  pose.translation() = transform.translation;
  return pose;
}

fcl::Transform3d PassthroughController::create_fcl_transform_from_data( double quat_w,
                                                                        double quat_x, double quat_y,
                                                                        double quat_z, double t_x,
                                                                        double t_y, double t_z )
{
  fcl::Quaterniond q = fcl::Quaterniond( quat_w, quat_x, quat_y, quat_z );
  fcl::Vector3d t = fcl::Vector3d( t_x, t_y, t_z );

  fcl::Transform3d pose = fcl::Transform3d::Identity();
  pose.linear() = q.toRotationMatrix();
  pose.translation() = t;

  return pose;
}

std::shared_ptr<fcl::CollisionGeometry<double>>
PassthroughController::urdf_geom_to_fcl_geom( std::shared_ptr<const urdf::Geometry> urdf_geom )
{

  switch ( urdf_geom->type ) {
  case urdf::Geometry::SPHERE: {
    std::shared_ptr<const urdf::Sphere> urdf_sphere =
        std::static_pointer_cast<const urdf::Sphere>( urdf_geom );
    std::shared_ptr<fcl::Sphere<double>> fcl_sphere =
        std::make_shared<fcl::Sphere<double>>( urdf_sphere->radius + safety_margin_ );
    return std::static_pointer_cast<fcl::CollisionGeometry<double>>( fcl_sphere );
  }
  case urdf::Geometry::BOX: {
    std::shared_ptr<const urdf::Box> urdf_box =
        std::static_pointer_cast<const urdf::Box>( urdf_geom );
    std::shared_ptr<fcl::Box<double>> fcl_box = std::make_shared<fcl::Box<double>>(
        urdf_box->dim.x + safety_margin_, urdf_box->dim.y + safety_margin_,
        urdf_box->dim.z + safety_margin_ );
    return std::static_pointer_cast<fcl::CollisionGeometry<double>>( fcl_box );
  }
  case urdf::Geometry::CYLINDER: {
    std::shared_ptr<const urdf::Cylinder> urdf_cylinder =
        std::static_pointer_cast<const urdf::Cylinder>( urdf_geom );
    std::shared_ptr<fcl::Cylinder<double>> fcl_cylinder = std::make_shared<fcl::Cylinder<double>>(
        urdf_cylinder->radius + safety_margin_, urdf_cylinder->length + safety_margin_ );
    return std::static_pointer_cast<fcl::CollisionGeometry<double>>( fcl_cylinder );
  }
  default: {
    return nullptr;
  }
  }
}

bool PassthroughController::pairwise_primitive_collision_check(
    std::vector<passthrough_controller::CollisionPrimitive> &source_colls,
    std::vector<passthrough_controller::CollisionPrimitive> &target_colls,
    fcl::Transform3d &base_to_child_l, fcl::Transform3d &target_link_to_base_t )
{

  std::vector<fcl::CollisionObjectd *> source_coll_objs;
  std::vector<fcl::CollisionObjectd *> target_coll_objs;

  for ( auto source_coll : source_colls ) {
    fcl::Transform3d source_coll_to_base_t = base_to_child_l * *std::get<1>( source_coll );
    fcl::CollisionObjectd *source_coll_obj_p = std::get<0>( source_coll ).get();
    source_coll_obj_p->setTransform( source_coll_to_base_t );

    source_coll_objs.push_back( source_coll_obj_p );
  }

  for ( auto target_coll : target_colls ) {
    fcl::Transform3d target_coll_to_base_t = target_link_to_base_t * *std::get<1>( target_coll );
    fcl::CollisionObjectd *target_coll_obj_p = std::get<0>( target_coll ).get();
    target_coll_obj_p->setTransform( target_coll_to_base_t );

    target_coll_objs.push_back( target_coll_obj_p );
  }

  source_coll_objs.resize( source_colls.size() );
  target_coll_objs.resize( target_colls.size() );

  bool collision_detected = false;
  fcl::CollisionRequest<double> request;
  for ( auto i = 0ul; i < source_coll_objs.size() && !collision_detected; i++ ) {
    for ( auto j = 0ul; j < target_coll_objs.size() && !collision_detected; j++ ) {
      // Check collision in base link frame
      fcl::CollisionResult<double> result;
      fcl::collide( source_coll_objs[i], target_coll_objs[j], request, result );
      collision_detected = result.isCollision();
    }
  }

  // for ( auto j = 0ul; j < target_coll_objs.size(); j++ ) delete ( target_coll_objs[j] );
  // for ( auto i = 0ul; i < source_coll_objs.size(); i++ ) delete ( source_coll_objs[i] );

  return collision_detected;
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
