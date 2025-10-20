#pragma once

#include <controller_interface/chainable_controller_interface.hpp>
#include <urdf_model/joint.h>

namespace safety_position_controller
{

class SafetyPositionController : public controller_interface::ChainableControllerInterface
{
public:
  SafetyPositionController();

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn
  on_configure( const rclcpp_lifecycle::State &previous_state ) override;
  controller_interface::CallbackReturn
  on_activate( const rclcpp_lifecycle::State &previous_state ) override;
  controller_interface::CallbackReturn
  on_deactivate( const rclcpp_lifecycle::State &previous_state ) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  controller_interface::return_type
  update_reference_from_subscribers( const rclcpp::Time &time,
                                     const rclcpp::Duration &period ) override;
  controller_interface::return_type
  update_and_write_commands( const rclcpp::Time &time, const rclcpp::Duration &period ) override;

  bool on_set_chained_mode( bool chained_mode ) override;

private:
  enum class JointType { CONTINUOUS, REVOLUTE_BOUNDED, PRISMATIC_BOUNDED, FIXED, OTHER };

  // Internal helpers
  double unwrap_to_nearest( double current, double target );
  double clamp( size_t i, double value ) const;
  bool parse_urdf_and_fill_joint_info( const std::string &urdf_xml );
  bool gather_interface_indices();

  // ---- Parameters ----
  std::vector<std::string> joint_names_;
  std::string robot_description_param_ = "robot_description";
  bool unwrap_continuous_ = true;
  bool enforce_limits_ = true;
  bool is_chained_ = true;

  // ---- State ----
  std::vector<int> state_interface_index_;

  // ---- URDF-based joint info ----
  std::vector<JointType> kinds_;
  std::vector<bool> has_limits_;
  std::vector<double> lower_limits_;
  std::vector<double> upper_limits_;
};

} // namespace safety_position_controller
