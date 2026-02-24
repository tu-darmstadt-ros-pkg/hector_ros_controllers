// Pinocchio headers must come first (Eigen plugin macros).
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "dynamics_mock_hardware/dynamics_mock_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace dynamics_mock_hardware
{

// =============================================================================
// Utilities
// =============================================================================

double DynamicsMockHardware::clampSymmetric( double val, double limit )
{
  return std::clamp( val, -limit, limit );
}

double DynamicsMockHardware::wrapAngle( double angle )
{
  angle = std::fmod( angle + M_PI, 2.0 * M_PI );
  if ( angle < 0.0 )
    angle += 2.0 * M_PI;
  return angle - M_PI;
}

double DynamicsMockHardware::readParam( const std::unordered_map<std::string, std::string> &params,
                                        const std::string &key, double fallback ) const
{
  auto it = params.find( key );
  if ( it != params.end() ) {
    try {
      return hardware_interface::stod( it->second );
    } catch ( const std::exception &e ) {
      RCLCPP_WARN( get_logger(), "Failed to parse param '%s': %s (using default %f)", key.c_str(),
                   e.what(), fallback );
    }
  }
  return fallback;
}

// =============================================================================
// Pinocchio model initialisation
// =============================================================================

bool DynamicsMockHardware::initPinocchioModel( const std::string &urdf_xml )
{
  try {
    pinocchio::urdf::buildModelFromXML( urdf_xml, model_ );
    model_.gravity = pinocchio::Motion( gravity_, Eigen::Vector3d::Zero() );
    data_ = pinocchio::Data( model_ );
    return true;
  } catch ( const std::exception &e ) {
    RCLCPP_ERROR( get_logger(), "Failed to build pinocchio model: %s", e.what() );
    return false;
  }
}

// =============================================================================
// Pinocchio <-> ROS conversions
// =============================================================================

void DynamicsMockHardware::rosToPin_q( Eigen::VectorXd &q_pin ) const
{
  for ( const auto &jd : joints_ ) {
    if ( jd.is_continuous ) {
      const double angle = jd.accumulated_angle;
      q_pin[jd.idx_q] = std::cos( angle );
      q_pin[jd.idx_q + 1] = std::sin( angle );
    } else {
      q_pin[jd.idx_q] = get_state( jd.name + "/" + hardware_interface::HW_IF_POSITION );
    }
  }
  // Enforce mimic joint positions in q
  for ( const auto &mjd : mimic_joints_ ) {
    const auto &ref = joints_[mjd.reference_joint_idx];
    const double ref_angle = ref.is_continuous ? ref.accumulated_angle : q_pin[ref.idx_q];
    const double mimic_angle = mjd.multiplier * ref_angle + mjd.offset;
    if ( mjd.is_continuous ) {
      q_pin[mjd.idx_q] = std::cos( mimic_angle );
      q_pin[mjd.idx_q + 1] = std::sin( mimic_angle );
    } else {
      q_pin[mjd.idx_q] = mimic_angle;
    }
  }
}

void DynamicsMockHardware::rosToPin_v( Eigen::VectorXd &v_pin ) const
{
  for ( const auto &jd : joints_ ) {
    v_pin[jd.idx_v] = get_state( jd.name + "/" + hardware_interface::HW_IF_VELOCITY );
  }
  for ( const auto &mjd : mimic_joints_ ) {
    const auto &ref = joints_[mjd.reference_joint_idx];
    v_pin[mjd.idx_v] = mjd.multiplier * v_pin[ref.idx_v];
  }
}

// =============================================================================
// Mimic constraint enforcement
// =============================================================================

void DynamicsMockHardware::enforceMimicConstraints( Eigen::VectorXd &q, Eigen::VectorXd &v ) const
{
  for ( const auto &mjd : mimic_joints_ ) {
    const auto &ref = joints_[mjd.reference_joint_idx];

    // Position: q_mimic = multiplier * q_ref + offset
    double ref_angle;
    if ( ref.is_continuous ) {
      ref_angle = ref.accumulated_angle;
    } else {
      ref_angle = q[ref.idx_q];
    }
    const double mimic_angle = mjd.multiplier * ref_angle + mjd.offset;

    if ( mjd.is_continuous ) {
      q[mjd.idx_q] = std::cos( mimic_angle );
      q[mjd.idx_q + 1] = std::sin( mimic_angle );
    } else {
      q[mjd.idx_q] = mimic_angle;
    }

    // Velocity: v_mimic = multiplier * v_ref
    v[mjd.idx_v] = mjd.multiplier * v[ref.idx_v];
  }
}

// =============================================================================
// Control law
// =============================================================================

void DynamicsMockHardware::computeTorques( const Eigen::VectorXd &q, const Eigen::VectorXd &v,
                                           Eigen::VectorXd &tau )
{
  tau.setZero();

  bool need_rnea = false;
  Eigen::VectorXd a_desired = Eigen::VectorXd::Zero( model_.nv );

  for ( const auto &jd : joints_ ) {
    const int iv = jd.idx_v;

    switch ( jd.control_mode ) {
    case ControlMode::POSITION: {
      double q_cmd = get_command( jd.name + "/" + hardware_interface::HW_IF_POSITION );
      if ( std::isnan( q_cmd ) )
        q_cmd = get_state( jd.name + "/" + hardware_interface::HW_IF_POSITION );

      const double q_cur = jd.is_continuous ? jd.accumulated_angle : q[jd.idx_q];
      const double v_cur = v[iv];
      const double raw_tau = position_kp_ * ( q_cmd - q_cur ) + position_kd_ * ( 0.0 - v_cur );
      tau[iv] = clampSymmetric( raw_tau, jd.max_torque );
      break;
    }

    case ControlMode::VELOCITY: {
      double v_cmd = get_command( jd.name + "/" + hardware_interface::HW_IF_VELOCITY );
      if ( std::isnan( v_cmd ) )
        v_cmd = 0.0;

      const double raw_tau = velocity_kp_ * ( v_cmd - v[iv] );
      tau[iv] = clampSymmetric( raw_tau, jd.max_torque );
      break;
    }

    case ControlMode::ACCELERATION: {
      double a_cmd = get_command( jd.name + "/" + hardware_interface::HW_IF_ACCELERATION );
      if ( std::isnan( a_cmd ) )
        a_cmd = 0.0;

      a_desired[iv] = a_cmd;
      need_rnea = true;
      break;
    }

    case ControlMode::EFFORT: {
      double tau_cmd = get_command( jd.name + "/" + hardware_interface::HW_IF_EFFORT );
      if ( std::isnan( tau_cmd ) )
        tau_cmd = 0.0;

      tau[iv] = clampSymmetric( tau_cmd, jd.max_torque );
      break;
    }
    }
  }

  if ( need_rnea ) {
    pinocchio::rnea( model_, data_, q, v, a_desired );
    for ( const auto &jd : joints_ ) {
      if ( jd.control_mode == ControlMode::ACCELERATION ) {
        tau[jd.idx_v] = clampSymmetric( data_.tau[jd.idx_v], jd.max_torque );
      }
    }
  }
}

// =============================================================================
// Lifecycle: on_init
// =============================================================================

CallbackReturn
DynamicsMockHardware::on_init( const hardware_interface::HardwareComponentInterfaceParams &params )
{
  if ( SystemInterface::on_init( params ) != CallbackReturn::SUCCESS )
    return CallbackReturn::ERROR;

  const auto &hw_params = get_hardware_info().hardware_parameters;

  // Global hardware parameters
  gravity_.x() = readParam( hw_params, "gravity_x", 0.0 );
  gravity_.y() = readParam( hw_params, "gravity_y", 0.0 );
  gravity_.z() = readParam( hw_params, "gravity_z", -9.81 );
  position_kp_ = readParam( hw_params, "position_kp", 100.0 );
  position_kd_ = readParam( hw_params, "position_kd", 10.0 );
  velocity_kp_ = readParam( hw_params, "velocity_kp", 10.0 );
  integration_dt_ = readParam( hw_params, "integration_dt", 0.001 );
  debug_log_interval_ = static_cast<int>( readParam( hw_params, "debug_log_interval", 0.0 ) );

  // Build pinocchio model from the full URDF
  if ( !initPinocchioModel( get_hardware_info().original_xml ) )
    return CallbackReturn::ERROR;

  // Map pinocchio joint names -> indices (skip universe joint 0)
  std::unordered_map<std::string, pinocchio::JointIndex> pin_name_to_id;
  for ( pinocchio::JointIndex jid = 1;
        jid < static_cast<pinocchio::JointIndex>( model_.joints.size() ); ++jid ) {
    pin_name_to_id[model_.names[jid]] = jid;
  }

  // Build per-joint mapping
  const auto &info = get_hardware_info();
  joints_.reserve( info.joints.size() );

  // Identify mimic joints from pre-parsed URDF data
  std::unordered_set<std::size_t> mimic_joint_ros_indices;
  std::unordered_map<std::size_t, const hardware_interface::MimicJoint *> mimic_info_map;
  for ( const auto &mj : info.mimic_joints ) {
    mimic_joint_ros_indices.insert( mj.joint_index );
    mimic_info_map[mj.joint_index] = &mj;
  }

  for ( std::size_t i = 0; i < info.joints.size(); ++i ) {
    const auto &joint_info = info.joints[i];
    auto pit = pin_name_to_id.find( joint_info.name );
    if ( pit == pin_name_to_id.end() ) {
      // Joint not in pinocchio model.
      // If it only has state interfaces (no command interfaces), skip it gracefully.
      if ( joint_info.command_interfaces.empty() ) {
        RCLCPP_INFO( get_logger(),
                     "Joint '%s' not found in pinocchio model and has no command interfaces "
                     "(passive joint) — skipping",
                     joint_info.name.c_str() );
        continue;
      }
      RCLCPP_ERROR( get_logger(),
                    "Joint '%s' declared in <ros2_control> not found in pinocchio model",
                    joint_info.name.c_str() );
      return CallbackReturn::ERROR;
    }

    const auto pin_id = pit->second;

    // Mimic joints: in pinocchio model but controlled via reference joint
    if ( mimic_joint_ros_indices.count( i ) > 0 ) {
      const auto *mj_info = mimic_info_map[i];
      MimicJointData mjd;
      mjd.name = joint_info.name;
      mjd.pinocchio_id = pin_id;
      mjd.idx_q = model_.idx_qs[pin_id];
      mjd.idx_v = model_.idx_vs[pin_id];
      mjd.nq = model_.joints[pin_id].nq();
      mjd.nv = model_.joints[pin_id].nv();
      mjd.is_continuous = ( mjd.nq == 2 && mjd.nv == 1 );
      mjd.multiplier = mj_info->multiplier;
      mjd.offset = mj_info->offset;
      // Temporarily store info.joints[] index; resolved to joints_[] index below
      mjd.reference_joint_idx = mj_info->mimicked_joint_index;

      RCLCPP_INFO( get_logger(),
                   "Mimic joint '%s': pin_id=%lu, mimics '%s' with multiplier=%.3f, offset=%.3f",
                   mjd.name.c_str(), static_cast<unsigned long>( pin_id ),
                   info.joints[mj_info->mimicked_joint_index].name.c_str(), mjd.multiplier,
                   mjd.offset );

      mimic_joints_.push_back( std::move( mjd ) );
      continue;
    }

    JointData jd;
    jd.name = joint_info.name;
    jd.ros_index = i;
    jd.pinocchio_id = pin_id;
    jd.idx_q = model_.idx_qs[pin_id];
    jd.idx_v = model_.idx_vs[pin_id];
    jd.nq = model_.joints[pin_id].nq();
    jd.nv = model_.joints[pin_id].nv();
    jd.is_continuous = ( jd.nq == 2 && jd.nv == 1 );

    // Resolve limits: per-joint <param> > URDF limits > infinity
    const auto &jp = joint_info.parameters;

    double urdf_max_torque = std::numeric_limits<double>::infinity();
    double urdf_max_vel = std::numeric_limits<double>::infinity();
    double urdf_pos_lower = -std::numeric_limits<double>::infinity();
    double urdf_pos_upper = std::numeric_limits<double>::infinity();

    auto lim_it = info.limits.find( joint_info.name );
    if ( lim_it != info.limits.end() ) {
      const auto &lim = lim_it->second;
      if ( lim.has_effort_limits )
        urdf_max_torque = lim.max_effort;
      if ( lim.has_velocity_limits )
        urdf_max_vel = lim.max_velocity;
      if ( lim.has_position_limits ) {
        urdf_pos_lower = lim.min_position;
        urdf_pos_upper = lim.max_position;
      }
    }

    jd.max_torque = readParam( jp, "max_torque", urdf_max_torque );
    jd.max_velocity = readParam( jp, "max_velocity", urdf_max_vel );
    jd.position_lower = readParam( jp, "position_lower_limit", urdf_pos_lower );
    jd.position_upper = readParam( jp, "position_upper_limit", urdf_pos_upper );
    jd.max_acceleration =
        readParam( jp, "max_acceleration", std::numeric_limits<double>::infinity() );

    // wrap_position: explicit param, else default to false.
    // For continuous joints the unwrapped accumulated_angle is published so
    // that controllers (e.g. vel_to_pos) see a monotonically changing value.
    // Wrapping to [-pi,pi] would cause a position command discontinuity
    // whenever the joint crosses +/-pi, making the PD controller snap back.
    auto wp_it = jp.find( "wrap_position" );
    if ( wp_it != jp.end() ) {
      jd.wrap_position = ( wp_it->second == "true" || wp_it->second == "1" );
    } else {
      jd.wrap_position = false;
    }

    jd.control_mode = ControlMode::POSITION;

    RCLCPP_INFO( get_logger(),
                 "Joint '%s': pin_id=%lu, nq=%d, nv=%d, continuous=%s, max_torque=%.2f, "
                 "max_vel=%.2f, pos=[%.2f, %.2f], wrap=%s",
                 jd.name.c_str(), static_cast<unsigned long>( jd.pinocchio_id ), jd.nq, jd.nv,
                 jd.is_continuous ? "yes" : "no", jd.max_torque, jd.max_velocity, jd.position_lower,
                 jd.position_upper, jd.wrap_position ? "yes" : "no" );

    joints_.push_back( std::move( jd ) );
  }

  // Resolve mimic reference indices: info.joints[] index -> joints_[] index
  std::unordered_map<std::size_t, std::size_t> ros_idx_to_joints_idx;
  for ( std::size_t j = 0; j < joints_.size(); ++j ) {
    ros_idx_to_joints_idx[joints_[j].ros_index] = j;
  }
  for ( auto &mjd : mimic_joints_ ) {
    const std::size_t ref_ros_idx = mjd.reference_joint_idx;
    auto it = ros_idx_to_joints_idx.find( ref_ros_idx );
    if ( it == ros_idx_to_joints_idx.end() ) {
      RCLCPP_ERROR( get_logger(),
                    "Mimic joint '%s' references joint '%s' which is not an actuated joint",
                    mjd.name.c_str(), info.joints[ref_ros_idx].name.c_str() );
      return CallbackReturn::ERROR;
    }
    mjd.reference_joint_idx = it->second;
  }

  // Allocate working vectors
  q_ = pinocchio::neutral( model_ );
  v_ = Eigen::VectorXd::Zero( model_.nv );
  tau_ = Eigen::VectorXd::Zero( model_.nv );
  ddq_ = Eigen::VectorXd::Zero( model_.nv );
  q_next_ = Eigen::VectorXd::Zero( model_.nq );

  return CallbackReturn::SUCCESS;
}

// =============================================================================
// Lifecycle: on_configure
// =============================================================================

CallbackReturn DynamicsMockHardware::on_configure( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  for ( auto &jd : joints_ ) {
    const std::string pos_if = jd.name + "/" + hardware_interface::HW_IF_POSITION;
    const std::string vel_if = jd.name + "/" + hardware_interface::HW_IF_VELOCITY;
    const std::string acc_if = jd.name + "/" + hardware_interface::HW_IF_ACCELERATION;
    const std::string eff_if = jd.name + "/" + hardware_interface::HW_IF_EFFORT;

    // Initialise states to zero if not yet set (URDF initial_value takes precedence)
    if ( has_state( pos_if ) && std::isnan( get_state( pos_if ) ) )
      set_state( pos_if, 0.0 );
    if ( has_state( vel_if ) && std::isnan( get_state( vel_if ) ) )
      set_state( vel_if, 0.0 );
    if ( has_state( acc_if ) && std::isnan( get_state( acc_if ) ) )
      set_state( acc_if, 0.0 );
    if ( has_state( eff_if ) && std::isnan( get_state( eff_if ) ) )
      set_state( eff_if, 0.0 );

    jd.accumulated_angle = get_state( pos_if );
    jd.control_mode = ControlMode::POSITION;
  }

  // Initialize mimic joint states
  for ( const auto &mjd : mimic_joints_ ) {
    const std::string pos_if = mjd.name + "/" + hardware_interface::HW_IF_POSITION;
    const std::string vel_if = mjd.name + "/" + hardware_interface::HW_IF_VELOCITY;
    const std::string acc_if = mjd.name + "/" + hardware_interface::HW_IF_ACCELERATION;
    const std::string eff_if = mjd.name + "/" + hardware_interface::HW_IF_EFFORT;

    if ( has_state( pos_if ) && std::isnan( get_state( pos_if ) ) )
      set_state( pos_if, 0.0 );
    if ( has_state( vel_if ) && std::isnan( get_state( vel_if ) ) )
      set_state( vel_if, 0.0 );
    if ( has_state( acc_if ) && std::isnan( get_state( acc_if ) ) )
      set_state( acc_if, 0.0 );
    if ( has_state( eff_if ) && std::isnan( get_state( eff_if ) ) )
      set_state( eff_if, 0.0 );
  }

  // Sync pinocchio vectors from state interfaces
  rosToPin_q( q_ );
  rosToPin_v( v_ );

  return CallbackReturn::SUCCESS;
}

// =============================================================================
// Lifecycle: on_activate / on_deactivate
// =============================================================================

CallbackReturn DynamicsMockHardware::on_activate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  // Seed command interfaces from current states to prevent an initial jump.
  for ( const auto &jd : joints_ ) {
    const auto &n = jd.name;
    if ( has_command( n + "/" + hardware_interface::HW_IF_POSITION ) )
      set_command( n + "/" + hardware_interface::HW_IF_POSITION,
                   get_state( n + "/" + hardware_interface::HW_IF_POSITION ) );
    if ( has_command( n + "/" + hardware_interface::HW_IF_VELOCITY ) )
      set_command( n + "/" + hardware_interface::HW_IF_VELOCITY, 0.0 );
    if ( has_command( n + "/" + hardware_interface::HW_IF_ACCELERATION ) )
      set_command( n + "/" + hardware_interface::HW_IF_ACCELERATION, 0.0 );
    if ( has_command( n + "/" + hardware_interface::HW_IF_EFFORT ) )
      set_command( n + "/" + hardware_interface::HW_IF_EFFORT, 0.0 );
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamicsMockHardware::on_deactivate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  // Simulate brake: zero velocities and efforts
  v_.setZero();
  for ( auto &jd : joints_ ) {
    set_state( jd.name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0 );
    if ( has_state( jd.name + "/" + hardware_interface::HW_IF_EFFORT ) )
      set_state( jd.name + "/" + hardware_interface::HW_IF_EFFORT, 0.0 );
    if ( has_state( jd.name + "/" + hardware_interface::HW_IF_ACCELERATION ) )
      set_state( jd.name + "/" + hardware_interface::HW_IF_ACCELERATION, 0.0 );
  }
  for ( const auto &mjd : mimic_joints_ ) {
    set_state( mjd.name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0 );
    if ( has_state( mjd.name + "/" + hardware_interface::HW_IF_EFFORT ) )
      set_state( mjd.name + "/" + hardware_interface::HW_IF_EFFORT, 0.0 );
    if ( has_state( mjd.name + "/" + hardware_interface::HW_IF_ACCELERATION ) )
      set_state( mjd.name + "/" + hardware_interface::HW_IF_ACCELERATION, 0.0 );
  }
  return CallbackReturn::SUCCESS;
}

// =============================================================================
// Mode switching
// =============================================================================

return_type DynamicsMockHardware::prepare_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> & /*stop_interfaces*/ )
{
  // Validate: at most one standard command interface per joint
  std::unordered_map<std::string, int> joint_start_count;

  for ( const auto &key : start_interfaces ) {
    for ( const auto &jd : joints_ ) {
      if ( key == jd.name + "/" + hardware_interface::HW_IF_POSITION ||
           key == jd.name + "/" + hardware_interface::HW_IF_VELOCITY ||
           key == jd.name + "/" + hardware_interface::HW_IF_ACCELERATION ||
           key == jd.name + "/" + hardware_interface::HW_IF_EFFORT ) {
        joint_start_count[jd.name]++;
      }
    }
  }

  for ( const auto &[name, count] : joint_start_count ) {
    if ( count > 1 ) {
      RCLCPP_ERROR( get_logger(),
                    "Joint '%s' has %d command interfaces requested simultaneously -- only one "
                    "allowed",
                    name.c_str(), count );
      return return_type::ERROR;
    }
  }
  return return_type::OK;
}

return_type DynamicsMockHardware::perform_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> & /*stop_interfaces*/ )
{
  for ( const auto &key : start_interfaces ) {
    for ( auto &jd : joints_ ) {
      if ( key == jd.name + "/" + hardware_interface::HW_IF_POSITION ) {
        jd.control_mode = ControlMode::POSITION;
        // Seed position command to current position to prevent jump
        if ( has_command( key ) )
          set_command( key, get_state( key ) );
      } else if ( key == jd.name + "/" + hardware_interface::HW_IF_VELOCITY ) {
        jd.control_mode = ControlMode::VELOCITY;
      } else if ( key == jd.name + "/" + hardware_interface::HW_IF_ACCELERATION ) {
        jd.control_mode = ControlMode::ACCELERATION;
      } else if ( key == jd.name + "/" + hardware_interface::HW_IF_EFFORT ) {
        jd.control_mode = ControlMode::EFFORT;
      }
    }
  }
  return return_type::OK;
}

// =============================================================================
// Debug logging
// =============================================================================

static const char *controlModeStr( ControlMode m )
{
  switch ( m ) {
  case ControlMode::POSITION:
    return "P";
  case ControlMode::VELOCITY:
    return "V";
  case ControlMode::ACCELERATION:
    return "A";
  case ControlMode::EFFORT:
    return "E";
  }
  return "?";
}

void DynamicsMockHardware::logDebugState() const
{
  std::ostringstream oss;
  oss << std::fixed;

  // Header: joint name abbreviations
  // Format per joint:  name[mode] pos vel cmd tau
  oss << "\n--- DynamicsMockHardware state ---\n";
  oss << std::setprecision( 4 );

  for ( const auto &jd : joints_ ) {
    // Short name: take last component after last '_' or full name if short
    const auto &name = jd.name;

    const double pos = get_state( name + "/" + hardware_interface::HW_IF_POSITION );
    const double vel = get_state( name + "/" + hardware_interface::HW_IF_VELOCITY );

    // Get current command based on control mode
    double cmd = 0.0;
    switch ( jd.control_mode ) {
    case ControlMode::POSITION:
      cmd = has_command( name + "/" + hardware_interface::HW_IF_POSITION )
                ? get_command( name + "/" + hardware_interface::HW_IF_POSITION )
                : std::numeric_limits<double>::quiet_NaN();
      break;
    case ControlMode::VELOCITY:
      cmd = has_command( name + "/" + hardware_interface::HW_IF_VELOCITY )
                ? get_command( name + "/" + hardware_interface::HW_IF_VELOCITY )
                : std::numeric_limits<double>::quiet_NaN();
      break;
    case ControlMode::ACCELERATION:
      cmd = has_command( name + "/" + hardware_interface::HW_IF_ACCELERATION )
                ? get_command( name + "/" + hardware_interface::HW_IF_ACCELERATION )
                : std::numeric_limits<double>::quiet_NaN();
      break;
    case ControlMode::EFFORT:
      cmd = has_command( name + "/" + hardware_interface::HW_IF_EFFORT )
                ? get_command( name + "/" + hardware_interface::HW_IF_EFFORT )
                : std::numeric_limits<double>::quiet_NaN();
      break;
    }

    oss << "  " << name << "[" << controlModeStr( jd.control_mode ) << "]"
        << "  pos=" << pos;
    if ( jd.is_continuous )
      oss << " (acc=" << jd.accumulated_angle << ")";
    oss << "  vel=" << vel << "  cmd=" << cmd << "  tau=" << tau_[jd.idx_v] << "\n";
  }

  RCLCPP_INFO( get_logger(), "%s", oss.str().c_str() );
}

// =============================================================================
// Read / Write
// =============================================================================

return_type DynamicsMockHardware::read( const rclcpp::Time & /*time*/,
                                        const rclcpp::Duration & /*period*/ )
{
  // States are already set by the previous write() cycle.
  return return_type::OK;
}

return_type DynamicsMockHardware::write( const rclcpp::Time & /*time*/,
                                         const rclcpp::Duration &period )
{
  // Cap the simulation period to avoid runaway sub-stepping when the
  // controller manager accumulates large periods from missed cycles.
  static constexpr double MAX_SIM_PERIOD = 0.1; // 100 ms
  const double total_dt = std::min( period.seconds(), MAX_SIM_PERIOD );
  if ( total_dt <= 0.0 )
    return return_type::OK;

  // Subdivide the control period into fixed-size integration steps
  const int num_steps = std::max( 1, static_cast<int>( std::ceil( total_dt / integration_dt_ ) ) );
  const double dt = total_dt / static_cast<double>( num_steps );

  // Convert current ROS states into pinocchio vectors
  rosToPin_q( q_ );
  rosToPin_v( v_ );

  for ( int step = 0; step < num_steps; ++step ) {
    // Enforce mimic constraints before computing torques so ABA sees consistent state
    enforceMimicConstraints( q_, v_ );

    // Compute per-joint torques from commands (re-evaluated each sub-step
    // so position/velocity PD controllers use the latest simulated state)
    computeTorques( q_, v_, tau_ );

    // Mimic joints receive no external torque (mechanically constrained)
    for ( const auto &mjd : mimic_joints_ ) { tau_[mjd.idx_v] = 0.0; }

    // Forward dynamics (ABA)
    pinocchio::aba( model_, data_, q_, v_, tau_ );
    ddq_ = data_.ddq;

    // Clamp accelerations
    for ( const auto &jd : joints_ ) {
      ddq_[jd.idx_v] = clampSymmetric( ddq_[jd.idx_v], jd.max_acceleration );
    }

    // Semi-implicit Euler: v += a * dt
    v_ += ddq_ * dt;

    // Clamp velocities
    for ( const auto &jd : joints_ ) {
      v_[jd.idx_v] = clampSymmetric( v_[jd.idx_v], jd.max_velocity );
    }

    // Lie-group integration: q_next = q (+) v*dt
    const Eigen::VectorXd v_dt = v_ * dt;
    pinocchio::integrate( model_, q_, v_dt, q_next_ );
    q_ = q_next_;

    // Position clamping and limit bounce prevention
    for ( const auto &jd : joints_ ) {
      if ( !jd.is_continuous ) {
        if ( std::isfinite( jd.position_lower ) && q_[jd.idx_q] <= jd.position_lower ) {
          q_[jd.idx_q] = jd.position_lower;
          if ( v_[jd.idx_v] < 0.0 )
            v_[jd.idx_v] = 0.0;
        }
        if ( std::isfinite( jd.position_upper ) && q_[jd.idx_q] >= jd.position_upper ) {
          q_[jd.idx_q] = jd.position_upper;
          if ( v_[jd.idx_v] > 0.0 )
            v_[jd.idx_v] = 0.0;
        }
      }
    }

    // Update accumulated angles for continuous joints each sub-step
    for ( auto &jd : joints_ ) {
      if ( jd.is_continuous ) {
        jd.accumulated_angle += v_[jd.idx_v] * dt;
      }
    }

    // Re-enforce mimic constraints after integration (overwrites ABA-computed mimic DOFs)
    enforceMimicConstraints( q_, v_ );
  }

  // Write final results to ROS state interfaces -- actuated joints
  for ( auto &jd : joints_ ) {
    // Position
    if ( jd.is_continuous ) {
      const double pos = jd.wrap_position ? wrapAngle( jd.accumulated_angle ) : jd.accumulated_angle;
      set_state( jd.name + "/" + hardware_interface::HW_IF_POSITION, pos );
    } else {
      set_state( jd.name + "/" + hardware_interface::HW_IF_POSITION, q_[jd.idx_q] );
    }
    // Velocity
    set_state( jd.name + "/" + hardware_interface::HW_IF_VELOCITY, v_[jd.idx_v] );
    // Acceleration (from last sub-step)
    if ( has_state( jd.name + "/" + hardware_interface::HW_IF_ACCELERATION ) )
      set_state( jd.name + "/" + hardware_interface::HW_IF_ACCELERATION, ddq_[jd.idx_v] );
    // Effort (from last sub-step)
    if ( has_state( jd.name + "/" + hardware_interface::HW_IF_EFFORT ) )
      set_state( jd.name + "/" + hardware_interface::HW_IF_EFFORT, tau_[jd.idx_v] );
  }

  // Write final results to ROS state interfaces -- mimic joints
  for ( const auto &mjd : mimic_joints_ ) {
    const auto &ref = joints_[mjd.reference_joint_idx];
    const double ref_pos = ref.is_continuous ? ref.accumulated_angle : q_[ref.idx_q];
    const double mimic_pos = mjd.multiplier * ref_pos + mjd.offset;
    set_state( mjd.name + "/" + hardware_interface::HW_IF_POSITION, mimic_pos );
    set_state( mjd.name + "/" + hardware_interface::HW_IF_VELOCITY, mjd.multiplier * v_[ref.idx_v] );
    if ( has_state( mjd.name + "/" + hardware_interface::HW_IF_EFFORT ) )
      set_state( mjd.name + "/" + hardware_interface::HW_IF_EFFORT, 0.0 );
    if ( has_state( mjd.name + "/" + hardware_interface::HW_IF_ACCELERATION ) )
      set_state( mjd.name + "/" + hardware_interface::HW_IF_ACCELERATION,
                 mjd.multiplier * ddq_[ref.idx_v] );
  }

  // Throttled debug logging
  if ( debug_log_interval_ > 0 ) {
    if ( ++write_cycle_count_ >= debug_log_interval_ ) {
      write_cycle_count_ = 0;
      logDebugState();
    }
  }

  return return_type::OK;
}

} // namespace dynamics_mock_hardware

PLUGINLIB_EXPORT_CLASS( dynamics_mock_hardware::DynamicsMockHardware,
                        hardware_interface::SystemInterface )
