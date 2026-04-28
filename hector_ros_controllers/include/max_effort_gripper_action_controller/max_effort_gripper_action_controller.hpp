#ifndef MAX_EFFORT_GRIPPER_ACTION_CONTROLLER__MAX_EFFORT_GRIPPER_ACTION_CONTROLLER_HPP_
#define MAX_EFFORT_GRIPPER_ACTION_CONTROLLER__MAX_EFFORT_GRIPPER_ACTION_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <optional>
#include <string>

#include "control_msgs/action/gripper_command.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/create_server.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_server_goal_handle.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

#include <hector_ros_controllers/max_effort_gripper_action_controller_parameters.hpp>

namespace max_effort_gripper_action_controller
{

/**
 * Gripper action controller that simultaneously commands a position and a max-effort
 * (current/torque limit) on a single joint. Designed for Dynamixel actuators in
 * current-based position control mode where writing the effort interface sets the
 * goal-current register.
 *
 * Three goal sources, all writing to a single internal target. Last writer wins;
 * any topic command preempts an active action goal.
 *   - control_msgs/action/GripperCommand on ~/gripper_cmd
 *   - std_msgs/msg/Float64 on ~/position_command  (uses default_max_effort)
 *   - std_msgs/msg/Float64 on ~/velocity_command  (integrated, uses default_max_effort)
 *
 * Publishes std_msgs/msg/Bool on ~/is_grasped at action_monitor_rate.
 */
class MaxEffortGripperActionController : public controller_interface::ControllerInterface
{
public:
  using GripperCommandAction = control_msgs::action::GripperCommand;
  using GoalHandle = rclcpp_action::ServerGoalHandle<GripperCommandAction>;
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<GripperCommandAction>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;

  struct Command {
    double position;
    double max_effort;
  };

  MaxEffortGripperActionController();
  ~MaxEffortGripperActionController() override = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn
  on_configure( const rclcpp_lifecycle::State &previous_state ) override;
  controller_interface::CallbackReturn
  on_activate( const rclcpp_lifecycle::State &previous_state ) override;
  controller_interface::CallbackReturn
  on_deactivate( const rclcpp_lifecycle::State &previous_state ) override;

  controller_interface::return_type update( const rclcpp::Time &time,
                                            const rclcpp::Duration &period ) override;

private:
  // Which input source won arbitration for the current update() cycle.
  enum class InputSource { None, Action, PositionTopic, VelocityTopic };

  // Lifecycle helpers
  void parse_joint_limits_from_urdf();

  // Action callbacks
  rclcpp_action::GoalResponse goal_callback( const rclcpp_action::GoalUUID &uuid,
                                             std::shared_ptr<const GripperCommandAction::Goal> goal );
  rclcpp_action::CancelResponse cancel_callback( std::shared_ptr<GoalHandle> goal_handle );
  void accepted_callback( std::shared_ptr<GoalHandle> goal_handle );

  // Goal management
  void preempt_active_goal( const std::string &reason );
  void clear_active_goal();
  void clear_pending_action_command();
  void flush_previous_goal_if_any();
  void set_hold_position();
  void check_for_success( const rclcpp::Time &time, double error_position, double current_position,
                          double current_velocity, double current_effort );

  // Lock-free accessors for goal-handle slots (touched from both RT and non-RT).
  // std::atomic_store/load/exchange on shared_ptr (correct in C++17, deprecated in C++20).
  static void store_goal_slot( RealtimeGoalHandlePtr &slot, RealtimeGoalHandlePtr handle );
  static RealtimeGoalHandlePtr load_goal_slot( const RealtimeGoalHandlePtr &slot );
  static RealtimeGoalHandlePtr exchange_goal_slot( RealtimeGoalHandlePtr &slot,
                                                   RealtimeGoalHandlePtr handle );

  // Continues integrating the cached velocity message if it's still within the watchdog
  // window. Used by the NONE arbitration branch.
  bool continue_velocity_integration_if_within_watchdog( const rclcpp::Time &time,
                                                         const rclcpp::Duration &period );

  // Parameters
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  // Joint limits (NaN if no limit, e.g. continuous joint)
  double joint_lower_limit_;
  double joint_upper_limit_;

  // Hardware interface handles
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> position_command_interface_;
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> effort_command_interface_;
  std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> position_state_interface_;
  std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> velocity_state_interface_;
  std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> effort_state_interface_;

  // Active target written to hardware each cycle
  Command target_;

  // Action server
  rclcpp_action::Server<GripperCommandAction>::SharedPtr action_server_;
  // Currently-active goal. Access ONLY via {store,load,exchange}_goal_slot.
  RealtimeGoalHandlePtr rt_active_goal_;
  // Most-recently-cleared goal, kept alive so the wall_timer can flush its deferred
  // terminal flag (succeed/abort/canceled set in RT) before the next accept replaces
  // the timer. Access ONLY via {store,load,exchange}_goal_slot.
  RealtimeGoalHandlePtr previous_rt_goal_;
  GripperCommandAction::Result::SharedPtr pre_alloc_result_;
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;
  rclcpp::Duration action_monitor_period_;
  rclcpp::Time last_movement_time_;

  // Per-source pending input. Every writer bumps input_seq_counter_ and stores the
  // returned sequence in its source-specific atomic. update() picks the highest
  // unconsumed seq, giving last-writer-wins arbitration regardless of dispatch order.
  std::atomic<uint64_t> input_seq_counter_;

  realtime_tools::RealtimeBuffer<Command> rt_action_command_;
  std::atomic<uint64_t> action_cmd_seq_;
  uint64_t last_consumed_action_seq_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_cmd_sub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64>> rt_position_cmd_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64>> rt_velocity_cmd_;
  std::atomic<uint64_t> position_cmd_seq_;
  std::atomic<uint64_t> velocity_cmd_seq_;
  uint64_t last_consumed_position_seq_;
  uint64_t last_consumed_velocity_seq_;
  rclcpp::Time last_velocity_msg_time_;
  // RT-only flag gating cached-velocity integration. Cleared by watchdog OR when a
  // non-velocity source wins.
  bool velocity_cached_valid_;

  // Effort hold-mode reference for is_grasped publisher (heartbeat throttling)
  rclcpp::Time last_is_grasped_publish_time_;
  bool is_grasped_;
  int is_grasped_dwell_counter_;
  realtime_tools::RealtimePublisherSharedPtr<std_msgs::msg::Bool> rt_is_grasped_pub_;
};

} // namespace max_effort_gripper_action_controller

#endif // MAX_EFFORT_GRIPPER_ACTION_CONTROLLER__MAX_EFFORT_GRIPPER_ACTION_CONTROLLER_HPP_
