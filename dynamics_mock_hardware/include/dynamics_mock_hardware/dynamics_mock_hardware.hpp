#ifndef DYNAMICS_MOCK_HARDWARE__DYNAMICS_MOCK_HARDWARE_HPP_
#define DYNAMICS_MOCK_HARDWARE__DYNAMICS_MOCK_HARDWARE_HPP_

#include <cstddef>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace dynamics_mock_hardware
{

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/// TORQUE_OFF is claimed through <joint>/torque_off: the motor applies no torque and the joint goes limp.
enum class ControlMode { POSITION, VELOCITY, ACCELERATION, EFFORT, TORQUE_OFF };

/// Per-joint data bridging ros2_control and pinocchio.
struct JointData {
  std::string name;
  std::size_t ros_index{ 0 };
  pinocchio::JointIndex pinocchio_id{ 0 };
  int idx_q{ 0 };
  int idx_v{ 0 };
  int nq{ 0 };
  int nv{ 0 };
  bool is_continuous{ false };

  // Limits (resolved from per-joint <param> overrides, then URDF limits, then infinity)
  double max_torque{ std::numeric_limits<double>::infinity() };
  double max_velocity{ std::numeric_limits<double>::infinity() };
  double position_lower{ -std::numeric_limits<double>::infinity() };
  double position_upper{ std::numeric_limits<double>::infinity() };
  double max_acceleration{ std::numeric_limits<double>::infinity() };
  bool wrap_position{ false };

  ControlMode control_mode{ ControlMode::POSITION };
  /// Position held while the position command is NaN, latched at the first NaN.
  double nan_hold_position{ std::numeric_limits<double>::quiet_NaN() };

  /// Accumulated angle for continuous joints (avoids atan2 discontinuity).
  double accumulated_angle{ 0.0 };
};

/// Per-mimic-joint data linking a mimic joint to its reference joint.
struct MimicJointData {
  std::string name;
  pinocchio::JointIndex pinocchio_id{ 0 };
  int idx_q{ 0 };
  int idx_v{ 0 };
  int nq{ 0 };
  int nv{ 0 };
  bool is_continuous{ false };

  /// Index into joints_ for the reference (mimicked) joint.
  std::size_t reference_joint_idx{ 0 };

  /// q_mimic = multiplier * q_ref + offset
  double multiplier{ 1.0 };
  double offset{ 0.0 };
};

/// Pinocchio-based mock hardware interface with forward dynamics simulation.
///
/// Receives position/velocity/acceleration/effort commands and simulates the
/// resulting motion using pinocchio's ABA forward dynamics algorithm.  Per-joint
/// torque limits allow realistic reproduction of motor saturation effects.
/// The command interface whose claim switches a joint's torque off. Nothing is written to it.
constexpr char TORQUE_OFF_INTERFACE[] = "torque_off";

class DynamicsMockHardware : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init( const hardware_interface::HardwareComponentInterfaceParams &params ) override;

  CallbackReturn on_configure( const rclcpp_lifecycle::State &previous_state ) override;

  CallbackReturn on_activate( const rclcpp_lifecycle::State &previous_state ) override;

  CallbackReturn on_deactivate( const rclcpp_lifecycle::State &previous_state ) override;

  return_type prepare_command_mode_switch( const std::vector<std::string> &start_interfaces,
                                           const std::vector<std::string> &stop_interfaces ) override;

  return_type perform_command_mode_switch( const std::vector<std::string> &start_interfaces,
                                           const std::vector<std::string> &stop_interfaces ) override;

  return_type read( const rclcpp::Time &time, const rclcpp::Duration &period ) override;

  return_type write( const rclcpp::Time &time, const rclcpp::Duration &period ) override;

private:
  bool initPinocchioModel( const std::string &urdf_xml );

  void rosToPin_q( Eigen::VectorXd &q_pin ) const;
  void rosToPin_v( Eigen::VectorXd &v_pin ) const;

  void enforceMimicConstraints( Eigen::VectorXd &q, Eigen::VectorXd &v ) const;

  void computeTorques( const Eigen::VectorXd &q, const Eigen::VectorXd &v, Eigen::VectorXd &tau );

  static double clampSymmetric( double val, double limit );
  static double wrapAngle( double angle );

  double readParam( const std::unordered_map<std::string, std::string> &params,
                    const std::string &key, double fallback ) const;

  void logDebugState() const;

  // Pinocchio model and data
  pinocchio::Model model_;
  pinocchio::Data data_{ model_ };

  // Actuated joints (in pinocchio model, with command interfaces)
  std::vector<JointData> joints_;

  // Mimic joints (in pinocchio model, state-only, derived from reference joint)
  std::vector<MimicJointData> mimic_joints_;

  // Pre-allocated pinocchio working vectors
  Eigen::VectorXd q_;
  Eigen::VectorXd v_;
  Eigen::VectorXd tau_;
  Eigen::VectorXd ddq_;
  Eigen::VectorXd q_next_;

  // Global PD gains for internal position/velocity controllers
  double position_kp_{ 100.0 };
  double position_kd_{ 10.0 };
  double velocity_kp_{ 10.0 };

  /// Fixed integration timestep (seconds). The control period is subdivided
  /// into ceil(period / integration_dt) steps for numerical stability.
  double integration_dt_{ 0.001 };

  Eigen::Vector3d gravity_{ 0.0, 0.0, -9.81 };

  // Debug logging
  int debug_log_interval_{ 0 }; // 0 = disabled, >0 = every N write() cycles
  int write_cycle_count_{ 0 };
};

} // namespace dynamics_mock_hardware

#endif // DYNAMICS_MOCK_HARDWARE__DYNAMICS_MOCK_HARDWARE_HPP_
