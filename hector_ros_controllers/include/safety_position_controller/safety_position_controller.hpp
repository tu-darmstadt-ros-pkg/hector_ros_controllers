//
// Created by aljoscha-schmidt on 10/19/25.
//

#ifndef SAFETY_POSITION_CONTROLLER_HPP
#define SAFETY_POSITION_CONTROLLER_HPP

#include <controller_interface/chainable_controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace safety_position_controller
{

class SafetyPositionController final : public controller_interface::ChainableControllerInterface
{
public:
  SafetyPositionController();

  // Lifecycle
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn
  on_configure( const rclcpp_lifecycle::State &previous_state ) override;
  controller_interface::CallbackReturn
  on_activate( const rclcpp_lifecycle::State &previous_state ) override;
  controller_interface::CallbackReturn
  on_deactivate( const rclcpp_lifecycle::State &previous_state ) override;

  // Interfaces
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // Chained-only: do not consume topics
  controller_interface::return_type
  update_reference_from_subscribers( const rclcpp::Time &time,
                                     const rclcpp::Duration &period ) override;

  // Upstream writes references here
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  // Main step
  controller_interface::return_type
  update_and_write_commands( const rclcpp::Time &time, const rclcpp::Duration &period ) override;

  bool on_set_chained_mode( bool chained_mode ) override;

private:
  // Parameters
  std::vector<std::string> joint_names_;
  std::string command_interface_name_{ "position" };
  std::string state_interface_name_{ "position" };
  std::string robot_description_param_{ "robot_description" };
  bool unwrap_continuous_{ true };
  bool enforce_limits_{ true };
  bool is_chained_{ false };

  // URDF-derived joint info
  enum class JointType : uint8_t { FIXED, CONTINUOUS, REVOLUTE_BOUNDED, PRISMATIC_BOUNDED, OTHER };
  std::vector<JointType> kinds_;
  std::vector<bool> has_limits_;
  std::vector<double> lower_limits_;
  std::vector<double> upper_limits_;

  std::vector<hardware_interface::LoanedCommandInterface *> cmd_handles_;
  std::vector<const hardware_interface::LoanedStateInterface *> state_handles_;
  std::vector<double> ref_buffer_;
  std::vector<double *> ref_ptrs_;
  std::vector<double> last_unwrapped_cmd_;

  void gather_interface_handles(); // called in on_activate()

private:
  // Helpers
  static inline double two_pi() { return 2.0 * M_PI; }
  static double unwrap_to_nearest( double current, double target );

  double clamp( size_t i, double value ) const;
  bool parse_urdf_and_fill_joint_info( const std::string &urdf_xml );

  void gather_interface_pointers(); // called in on_activate()
};

} // namespace safety_position_controller

#endif // SAFETY_POSITION_CONTROLLER_HPP
