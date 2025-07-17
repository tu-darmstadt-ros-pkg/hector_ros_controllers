#include "self_collision_avoidance_controller/self_collision_avoidance_controller.hpp"
#include "controller_interface/helpers.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace self_collision_avoidance_controller
{

rcl_interfaces::msg::SetParametersResult
SelfCollisionAvoidanceController::setParamCb( const rclcpp::Parameter &p )
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;

  if ( p.get_name() == "velocity_look_ahead_factor" ) {
    const double val = p.as_double();

    if ( val <= 0 )
      result.successful = false;
    else {
      velocity_look_ahead_factor_ = val;
      RCLCPP_INFO( get_node()->get_logger(), "Reconfigured velocity_look_ahead_factor to %f",
                   velocity_look_ahead_factor_ );
    }
  }

  return result;
}

controller_interface::CallbackReturn SelfCollisionAvoidanceController::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>( get_node() );
    params_ = param_listener_->get_params();
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>( get_node() );
    cb_handle_ = param_subscriber_->add_parameter_callback(
        "velocity_look_ahead_factor",
        std::bind( &SelfCollisionAvoidanceController::setParamCb, this, std::placeholders::_1 ) );

  } catch ( const std::exception &e ) {
    fprintf( stderr, "Exception thrown during init stage with message: %s \n", e.what() );
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
SelfCollisionAvoidanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_names_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
SelfCollisionAvoidanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for ( const std::string &joint_name : active_joints_ ) {
    state_interfaces_config.names.push_back( joint_name + "/position" );
  }

  return state_interfaces_config;
}

controller_interface::return_type SelfCollisionAvoidanceController::update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/ )
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

std::vector<hardware_interface::CommandInterface>
SelfCollisionAvoidanceController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;

  for ( size_t i = 0; i < reference_interface_names_.size(); ++i ) {
    reference_interfaces.push_back( hardware_interface::CommandInterface(
        get_node()->get_name(), reference_interface_names_[i], &reference_interfaces_[i] ) );
  }

  return reference_interfaces;
}

controller_interface::CallbackReturn SelfCollisionAvoidanceController::process_params()
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

  if ( params_.joint_interface_types.size() != params_.joints.size() ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Need to specifiy an interface type for each joint" );
    return controller_interface::CallbackReturn::ERROR;
  }

  if ( params_.passthrough_controller.empty() ) {
    passthrough_controller_ = "";
  } else {
    passthrough_controller_ = params_.passthrough_controller + "/";
  }

  velocity_look_ahead_factor_ = (int)params_.velocity_look_ahead_factor;

  if ( params_.joint_groups.empty() ) {

    // No joint groups
  } else if ( params_.joint_groups.size() != params_.joints.size() ) {
    RCLCPP_ERROR( get_node()->get_logger(),
                  "Need to specifiy a joint group for each joint or none" );
    return controller_interface::CallbackReturn::ERROR;
  } else {

    for ( size_t i = 0ul; i < params_.joint_groups.size(); i++ ) {
      const std::string &joint_group = params_.joint_groups[i];

      if ( joint_group != "None" && joint_group != "" ) {
        joint_groups_[params_.joints[i]] = joint_group;

        if ( group_joints_.find( joint_group ) != group_joints_.end() )
          group_joints_[joint_group].emplace_back( i );
        else
          group_joints_[joint_group] = std::vector<size_t>{ i };
      }
    }
  }

  // Use half since collision geom size increases are applied to source and target collision
  safety_margin_ = params_.safety_margin / 2;

  command_interface_names_.reserve( controlled_joints_.size() );
  interface_types_.reserve( controlled_joints_.size() );

  for ( auto i = 0ul; i < controlled_joints_.size(); i++ ) {

    if ( params_.joint_interface_types[i] != "position" &&
         params_.joint_interface_types[i] != "velocity" ) {
      RCLCPP_ERROR( get_node()->get_logger(),
                    "Only \"position\" and \"velocity\" interfaces are supported" );
      return controller_interface::CallbackReturn::ERROR;
    }

    command_interface_names_.push_back( passthrough_controller_ + controlled_joints_[i] + "/" +
                                        params_.joint_interface_types[i] );
    interface_types_.push_back( params_.joint_interface_types[i] );
  }

  command_interface_names_.shrink_to_fit();
  interface_types_.shrink_to_fit();

  // pre-reserve command interfaces
  // The names should be in the same order as for command interfaces for easier matching
  for ( auto i = 0ul; i < command_interface_names_.size(); i++ ) {
    reference_interface_names_.push_back( command_interface_names_[i] );
  }

  // reference_interface_names_.shrink_to_fit();
  //  for any case make reference interfaces size of command interfaces
  reference_interfaces_.resize( reference_interface_names_.size(),
                                std::numeric_limits<double>::quiet_NaN() );

  prev_command_vals_ = std::vector<double>( reference_interface_names_.size() );

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SelfCollisionAvoidanceController::on_configure( const rclcpp_lifecycle::State & /*previous_state*/ )
{

  controller_interface::CallbackReturn result = process_params();
  if ( result == controller_interface::CallbackReturn::ERROR )
    return controller_interface::CallbackReturn::ERROR;

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF( this->get_robot_description() );

  transformTree_ = std::make_shared<ad_kinematics::Tree>( urdf );

  auto qos = rclcpp::QoS( 10 );
  qos.transient_local();
  srdf_received_ = false;
  semantic_description_sub = get_node()->create_subscription<std_msgs::msg::String>(
      "robot_description_semantic", qos, [this, urdf]( const std_msgs::msg::String::SharedPtr msg ) {
        srdf_ = srdf::Model();
        srdf_.initString( *urdf, msg->data );
        srdf_received_ = true;
      } );

  set_joint_infos( urdf );

  set_dependent_links( urdf );

  if ( set_potentially_colliding_links( urdf ) != controller_interface::CallbackReturn::SUCCESS )
    return controller_interface::CallbackReturn::ERROR;

  collect_collision_primitives( urdf );

  RCLCPP_INFO( this->get_node()->get_logger(), "configure successful" );

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SelfCollisionAvoidanceController::set_potentially_colliding_links(
    const urdf::ModelInterfaceSharedPtr urdf )
{
  // wait for the semantic description message to be received
  rclcpp::Rate rate( 3 );
  int attempt = 0;
  int max_attempts = 50;
  while ( !srdf_received_ ) {
    rate.sleep();
    attempt++;
    if ( attempt % 10 == 0 )
      RCLCPP_INFO( get_node()->get_logger(), "Waiting for semantic robot description on topic " );
    if ( attempt > max_attempts )
      return controller_interface::CallbackReturn::ERROR;
  }

  std::set<std::string> all_link_names;
  for ( auto &link : urdf->links_ ) { all_link_names.insert( link.first ); }

  std::set<std::string> all_dependent_links;
  for ( std::vector<std::string> &dependent_links : controlled_joint_dependent_links_ ) {
    for ( std::string &dependent_link : dependent_links ) {
      auto result = all_dependent_links.insert( dependent_link );
      // If encountering a dependent link for first time initialize
      // potentially colliding links with all links
      if ( result.second ) {
        potentially_colliding_links_[dependent_link] = all_link_names;
        potentially_colliding_links_[dependent_link].erase( dependent_link );
      }
    }
  }

  const std::vector<srdf::Model::CollisionPair> &coll_pairs = srdf_.getDisabledCollisionPairs();
  for ( auto &coll_pair : coll_pairs ) {

    if ( all_dependent_links.find( coll_pair.link1_ ) != all_dependent_links.end() )
      potentially_colliding_links_[coll_pair.link1_].erase( coll_pair.link2_ );

    if ( all_dependent_links.find( coll_pair.link2_ ) != all_dependent_links.end() ) {
      potentially_colliding_links_[coll_pair.link2_].erase( coll_pair.link1_ );
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SelfCollisionAvoidanceController::on_activate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
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
SelfCollisionAvoidanceController::on_deactivate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  // reset command buffer
  rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>>( nullptr );
  return controller_interface::CallbackReturn::SUCCESS;
}

bool SelfCollisionAvoidanceController::on_set_chained_mode( bool /*chained_mode*/ ) { return true; }

void SelfCollisionAvoidanceController::update_joint_angles()
{
  for ( auto index = 0ul; index < q_indices_active_.size(); ++index ) {
    auto state_val = state_interfaces_[index].get_value();
    if ( !std::isnan( state_val ) )
      joint_angles_[q_indices_active_[index]] = state_val;
  }
}

bool SelfCollisionAvoidanceController::block_joint( const size_t &joint_idx )
{
  if ( interface_types_[joint_idx] == "position" )
    return command_interfaces_[joint_idx].set_value( prev_command_vals_[joint_idx] );
  else
    return command_interfaces_[joint_idx].set_value( 0.0 );
}

bool SelfCollisionAvoidanceController::write_valid_reference_commands(
    std::vector<bool> &collision_results )
{

  // Propagate collisions for respective joint group
  for ( size_t i = 0; i < collision_results.size(); i++ ) {

    if ( !collision_results[i] )
      continue;

    // If joint has a defined group, set all of them to collision
    if ( joint_groups_.find( controlled_joints_[i] ) != joint_groups_.end() ) {
      for ( const size_t &joint_idx : group_joints_[joint_groups_[controlled_joints_[i]]] ) {
        collision_results[joint_idx] = true;
      }
    }
  }

  bool success = true;
  for ( size_t i = 0; i < command_interfaces_.size(); i++ ) {
    if ( !std::isnan( reference_interfaces_[i] ) ) {
      if ( !collision_results[i] ) {
        success = success && command_interfaces_[i].set_value( reference_interfaces_[i] );
        prev_command_vals_[i] = reference_interfaces_[i];
      } else {
        success = success && block_joint( i );
      }
    }
  }

  return success;
}

controller_interface::return_type
SelfCollisionAvoidanceController::update_and_write_commands( const rclcpp::Time & /*time*/,
                                                             const rclcpp::Duration &p )
{
  // Apply joint updates read from state interfaces
  update_joint_angles();

  std::vector<bool> collision_results = std::vector<bool>( controlled_joints_.size() );

  // Iterate over controlled joints
  for ( size_t i = 0; i < controlled_joints_.size(); i++ ) {

    // skip if no command received from high level controller
    if ( std::isnan( reference_interfaces_[i] ) )
      continue;

    double pre_command_angle = joint_angles_[q_indices_controlled_[i]];

    // Set hypothetical joint position
    if ( interface_types_[i] == "position" ) {
      joint_angles_[q_indices_controlled_[i]] = reference_interfaces_[i];
    } else {
      joint_angles_[q_indices_controlled_[i]] +=
          reference_interfaces_[i] * velocity_look_ahead_factor_ * p.seconds();
    }

    bool any_collision = false;
    // Iterate over all links that are moved by the new reference value
    // Some dependent link is colliding -> skip other checks
    for ( size_t j = 0; j < controlled_joint_dependent_links_[i].size() && !any_collision; j++ ) {

      std::string &dependent_link = controlled_joint_dependent_links_[i][j];

      auto dependent_link_collisions = link_collision_primitives_[dependent_link];
      fcl::Transform3d base_dependent_link_transform = get_transform_from_base_link( dependent_link );

      // Iterate over all links that can collide with the moved link
      // Some potentially link is colliding -> skip other checks
      for ( auto it = potentially_colliding_links_[dependent_link].begin();
            it != potentially_colliding_links_[dependent_link].end() && !any_collision; it++ ) {

        const std::string &potentially_colliding_link = *it;

        try {
          fcl::Transform3d base_pot_colliding_link_transform =
              get_transform_from_base_link( potentially_colliding_link );

          any_collision = pairwise_primitive_collision_check(
              dependent_link_collisions, link_collision_primitives_[potentially_colliding_link],
              base_dependent_link_transform, base_pot_colliding_link_transform );

          /*RCLCPP_INFO(
              this->get_node()->get_logger(),
              "Collision check %s : %i for dependent link \"%s\" and colliding link \"%s\" ",
              controlled_joints_[i].c_str(), any_collision, dependent_link.c_str(),
              potentially_colliding_link.c_str() );*/

        } catch ( ... ) {
          RCLCPP_ERROR( this->get_node()->get_logger(), "Error for source %s to target %s.",
                        dependent_link.c_str(), potentially_colliding_link.c_str() );
        }
      }

      collision_results[i] = any_collision;
    }

    if ( any_collision ) {
      // Reset hypothetical position
      joint_angles_[q_indices_controlled_[i]] = pre_command_angle;
    }
  }

  bool success = write_valid_reference_commands( collision_results );

  if ( !success )
    return controller_interface::return_type::ERROR;

  return controller_interface::return_type::OK;
}

void SelfCollisionAvoidanceController::set_joint_infos( const urdf::ModelInterfaceSharedPtr &urdf )
{
  for ( auto &joint : controlled_joints_ ) auto urdf_joint = urdf->getJoint( joint );

  active_joints_ = transformTree_->getActiveJointNames();
  q_indices_controlled_ = transformTree_->getJointQIndices( controlled_joints_ );
  q_indices_active_ = transformTree_->getJointQIndices( active_joints_ );

  for ( auto idx = 0ul; idx < active_joints_.size(); idx++ ) joint_angles_.push_back( 0.0 );
  joint_angles_.resize( active_joints_.size() );
}

void SelfCollisionAvoidanceController::get_dependent_links_from_joint(
    const urdf::ModelInterfaceSharedPtr &urdf, const urdf::JointConstSharedPtr &joint,
    std::vector<std::string> &dependent_links )
{
  get_dependent_links_from_link( urdf, urdf->getLink( joint->child_link_name ), dependent_links );
}

void SelfCollisionAvoidanceController::get_dependent_links_from_link(
    const urdf::ModelInterfaceSharedPtr &urdf, const urdf::LinkConstSharedPtr &link,
    std::vector<std::string> &dependent_links )
{
  dependent_links.push_back( link->name );

  for ( const auto &child_joint : link->child_joints )
    get_dependent_links_from_joint( urdf, child_joint, dependent_links );

  for ( const auto &child_link : link->child_links )
    get_dependent_links_from_link( urdf, child_link, dependent_links );
}

void SelfCollisionAvoidanceController::set_dependent_links( const urdf::ModelInterfaceSharedPtr &urdf )
{

  for ( size_t i = 0; i < controlled_joints_.size(); i++ ) {
    controlled_joint_dependent_links_.emplace_back();
    get_dependent_links_from_joint( urdf, urdf->getJoint( controlled_joints_[i] ),
                                    controlled_joint_dependent_links_[i] );
  }

  controlled_joint_dependent_links_.resize( controlled_joints_.size() );
}

void SelfCollisionAvoidanceController::collect_collision_primitives(
    const urdf::ModelInterfaceSharedPtr &urdf )
{
  // Get collision primitives for all links that are considered for collisions
  std::set<std::string> relevant_links;
  for ( auto &it : potentially_colliding_links_ ) {
    for ( auto &potentially_colliding_link : it.second ) {
      relevant_links.insert( potentially_colliding_link );
    }
  }

  // Id is kept for possible debug reasons
  int id = 0;
  for ( const std::string &link_name : relevant_links ) {

    link_collision_primitives_[link_name] = std::vector<CollisionPrimitive>();
    auto urdf_collision_elements = urdf->getLink( link_name )->collision_array;

    for ( auto &urdf_collision_element : urdf_collision_elements ) {

      // Convert urdf::geometry object to fcl::CollisionObject
      // Pose will be set at runtime based on current joint configuration
      auto coll_obj = std::make_shared<fcl::CollisionObjectd>(
          urdf_geom_to_fcl_geom( urdf_collision_element->geometry ), fcl::Transform3d::Identity() );
      // Convert link -> primitive transform to fcl format
      auto joint_coll_transform = std::make_shared<fcl::Transform3d>(
          pose_to_fcl_transform( urdf_collision_element->origin ) );

      link_collision_primitives_[link_name].emplace_back( coll_obj, joint_coll_transform, id );
      id++;
    }
  }
}

fcl::Transform3d
SelfCollisionAvoidanceController::get_transform_from_base_link( const std::string &link ) const
{
  return kinematics_transform_to_fcl_transform(
      transformTree_->computeTransform<double>( link, joint_angles_ ) );
}

bool SelfCollisionAvoidanceController::pairwise_primitive_collision_check(
    const std::vector<self_collision_avoidance_controller::CollisionPrimitive> &dependent_link_colls,
    const std::vector<self_collision_avoidance_controller::CollisionPrimitive> &pot_coll_link_colls,
    const fcl::Transform3d &base_to_dependent_link, const fcl::Transform3d &pot_coll_link_to_base )
{

  std::vector<fcl::CollisionObjectd *> dependent_link_coll_objs;
  std::vector<fcl::CollisionObjectd *> pot_coll_link_coll_objs;

  // Calculate poses (transforms) for source collision primitives
  for ( auto link_coll : dependent_link_colls ) {
    fcl::Transform3d dependent_link_coll_to_base = base_to_dependent_link * *std::get<1>( link_coll );
    fcl::CollisionObjectd *source_coll_obj_p = std::get<0>( link_coll ).get();
    source_coll_obj_p->setTransform( dependent_link_coll_to_base );

    dependent_link_coll_objs.push_back( source_coll_obj_p );
  }

  // Calculate poses (transforms) for target collision primitives
  for ( auto link_coll : pot_coll_link_colls ) {
    fcl::Transform3d pot_coll_link_coll_to_base = pot_coll_link_to_base * *std::get<1>( link_coll );
    fcl::CollisionObjectd *target_coll_obj_p = std::get<0>( link_coll ).get();
    target_coll_obj_p->setTransform( pot_coll_link_coll_to_base );

    pot_coll_link_coll_objs.push_back( target_coll_obj_p );
  }

  dependent_link_coll_objs.resize( dependent_link_colls.size() );
  pot_coll_link_coll_objs.resize( pot_coll_link_colls.size() );

  bool collision_detected = false;
  for ( auto i = 0ul; i < dependent_link_coll_objs.size() && !collision_detected; i++ ) {
    for ( auto j = 0ul; j < pot_coll_link_coll_objs.size() && !collision_detected; j++ ) {
      fcl::CollisionRequest<double> request;
      // Check collision in base link frame
      fcl::CollisionResult<double> result;
      fcl::collide( dependent_link_coll_objs[i], pot_coll_link_coll_objs[j], request, result );
      collision_detected = result.isCollision();
    }
  }

  return collision_detected;
}

/*
 Conversion functions
*/

fcl::Transform3d SelfCollisionAvoidanceController::pose_to_fcl_transform( const urdf::Pose &urdf_pose )
{
  return create_fcl_transform_from_data(
      urdf_pose.rotation.w, urdf_pose.rotation.x, urdf_pose.rotation.y, urdf_pose.rotation.z,
      urdf_pose.position.x, urdf_pose.position.y, urdf_pose.position.z );
}

fcl::Transform3d SelfCollisionAvoidanceController::geom_transform_to_fcl_transform(
    const geometry_msgs::msg::Transform &transform )
{
  return create_fcl_transform_from_data(
      transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z,
      transform.translation.x, transform.translation.y, transform.translation.z );
}

fcl::Transform3d SelfCollisionAvoidanceController::kinematics_transform_to_fcl_transform(
    const ad_kinematics::Transform<double> &transform )
{
  fcl::Transform3d pose = fcl::Transform3d::Identity();
  pose.linear() = transform.rotation.toRotationMatrix();
  pose.translation() = transform.translation;
  return pose;
}

fcl::Transform3d SelfCollisionAvoidanceController::create_fcl_transform_from_data(
    const double quat_w, const double quat_x, const double quat_y, const double quat_z,
    const double t_x, const double t_y, const double t_z )
{
  fcl::Quaterniond q = fcl::Quaterniond( quat_w, quat_x, quat_y, quat_z );
  fcl::Vector3d t = fcl::Vector3d( t_x, t_y, t_z );

  fcl::Transform3d pose = fcl::Transform3d::Identity();
  pose.linear() = q.toRotationMatrix();
  pose.translation() = t;

  return pose;
}

std::shared_ptr<fcl::CollisionGeometry<double>> SelfCollisionAvoidanceController::urdf_geom_to_fcl_geom(
    const std::shared_ptr<const urdf::Geometry> &urdf_geom ) const
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
        urdf_box->dim.x + 2 * safety_margin_, urdf_box->dim.y + 2 * safety_margin_,
        urdf_box->dim.z + 2 * safety_margin_ );
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

/*
  Debug functions
*/

/*

void PassthroughController::log_transform( const fcl::Transform3d &t, const std::string title )
{
  RCLCPP_INFO( get_node()->get_logger(),
               "%s \n| %f %f %f %f |\n| %f %f %f %f |\n| %f %f %f %f |\n| %f %f %f %f |",
               title.c_str(), t( 0, 0 ), t( 0, 1 ), t( 0, 2 ), t( 0, 3 ), t( 1, 0 ), t( 1, 1 ),
               t( 1, 2 ), t( 1, 3 ), t( 2, 0 ), t( 2, 1 ), t( 2, 2 ), t( 2, 3 ), t( 3, 0 ),
               t( 3, 1 ), t( 3, 2 ), t( 3, 3 ) );
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

  // marker_pub_->publish( marker );
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
  // marker_pub_->publish( marker );
}*/

} // namespace self_collision_avoidance_controller

PLUGINLIB_EXPORT_CLASS( self_collision_avoidance_controller::SelfCollisionAvoidanceController,
                        controller_interface::ChainableControllerInterface )
