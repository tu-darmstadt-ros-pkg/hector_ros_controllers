#ifndef HECTOR_CONTROLLER_SPAWNER_HECTOR_CONTROLLER_SPAWNER_HPP
#define HECTOR_CONTROLLER_SPAWNER_HECTOR_CONTROLLER_SPAWNER_HPP

#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <controller_manager_msgs/srv/configure_controller.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/load_controller.hpp>
#include <controller_manager_msgs/srv/set_hardware_component_state.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/parameter_client.hpp>
namespace hector_controller_spawner
{

inline std::string vecToString( const std::vector<std::string> &vec )
{
  std::string result;
  for ( const auto &s : vec ) {
    if ( !result.empty() )
      result += ", ";
    result += s;
  }
  result += " (size: " + std::to_string( vec.size() ) + ")";
  return result;
}

/**
 *  @brief  Multispawner waits for an (optional) e‑stop, then loads & activates
 *          hardware interfaces followed by controllers .
 */
class MultiSpawner : public rclcpp::Node
{
public:
  using ControllerGroup = std::vector<std::string>; // group of controllers to activate together

  explicit MultiSpawner();
  void initialize();
  bool is_finished() const noexcept { return done_; }
  bool estop_released_and_not_started() const noexcept { return released_ && !started_; }
  void start_sequence();

private:
  // ----- helper structs -----
  struct ControllerCfg {
    bool activate{ true };
  };

  // ----- callbacks -----
  void estopCb( const std_msgs::msg::Bool::SharedPtr msg );

  // ----- helpers -----
  bool loadAndActivateHardware( const std::string &name );
  bool loadController( const std::string &name );
  bool configureController( const std::string &name );
  bool replicateParamsToCM();
  void verifyFinalStates();
  void parseControllerInfo( const controller_manager_msgs::srv::ListControllers_Response &resp,
                            std::unordered_map<std::string, std::string> &current_state );
  bool ensureControllerState( bool desired_state,
                              const std::unordered_map<std::string, std::string> &current_state );
  bool switchControllersRequest( const std::vector<std::string> &to_activate,
                                 const std::vector<std::string> &to_deactivate );

  // ----- parameters -----
  std::vector<std::string> hw_interfaces_;
  std::vector<std::string> controllers_;
  std::unordered_map<std::string, ControllerCfg> controller_cfg_;
  std::vector<ControllerGroup> controller_groups_;
  double retry_delay_{ 5.0 };
  std::string estop_topic_;
  bool started_{ false };
  std::atomic<bool> done_{ false };
  std::atomic<bool> released_{ false };

  // ----- service clients -----
  rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr set_hw_state_client_;
  rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedPtr load_ctrl_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_ctrl_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_ctrl_client_;
  rclcpp::Client<controller_manager_msgs::srv::ConfigureController>::SharedPtr configure_ctrl_client_;
  rclcpp::AsyncParametersClient::SharedPtr cm_param_client_;
  // ----- subscription -----
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
};

} // namespace hector_controller_spawner

#endif // HECTOR_CONTROLLER_SPAWNER_HECTOR_CONTROLLER_SPAWNER_HPP
