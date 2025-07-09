#include "hector_controller_spawner/hector_controller_spawner.hpp"
#include <functional>

#include <hector_controller_spawner/hector_controller_spawner.hpp>

namespace hector_controller_spawner
{

using namespace std::chrono_literals;

// --------------------------------------------------------------
MultiSpawner::MultiSpawner() : Node( "multi_controller_spawner" ) { }
void MultiSpawner::initialize()
{
  // 1) Declare & fetch parameters
  hw_interfaces_ = this->declare_parameter<std::vector<std::string>>( "hardware_interfaces",
                                                                      std::vector<std::string>() );
  controllers_ =
      this->declare_parameter<std::vector<std::string>>( "controllers", std::vector<std::string>() );
  retry_delay_ = this->declare_parameter<double>( "retry_delay", 5.0 );
  estop_topic_ = this->declare_parameter<std::string>( "estop_topic", "" );

  // output parameters for debugging
  std::stringstream ss;
  ss << "Parameters:\n";
  ss << "  hardware_interfaces: " << hw_interfaces_.size() << "\n";
  for ( const auto &hw : hw_interfaces_ ) { ss << "    - " << hw << "\n"; }
  ss << "  controllers: " << controllers_.size() << "\n";
  for ( const auto &ctrl : controllers_ ) { ss << "    - " << ctrl << "\n"; }
  ss << "  retry_delay: " << retry_delay_ << " seconds\n";
  ss << "  estop_topic: '" << estop_topic_ << "'\n";
  RCLCPP_INFO( get_logger(), "%s", ss.str().c_str() );

  for ( const auto &ctrl : controllers_ ) {
    ControllerCfg cfg;
    cfg.activate = this->declare_parameter<bool>( ctrl + ".activate", true );
    cfg.retry_on_failure = this->declare_parameter<bool>( ctrl + ".retry_on_failure", false );
    controller_cfg_[ctrl] = cfg;
  }

  // 2) Create service clients
  set_hw_state_client_ = this->create_client<controller_manager_msgs::srv::SetHardwareComponentState>(
      "controller_manager/set_hardware_component_state" );
  load_ctrl_client_ = this->create_client<controller_manager_msgs::srv::LoadController>(
      "controller_manager/load_controller" );
  switch_ctrl_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
      "controller_manager/switch_controller" );

  // 3) Handle e‑stop logic
  if ( estop_topic_.empty() ) {
    start_sequence();
  } else {
    estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        estop_topic_, rclcpp::SensorDataQoS(),
        std::bind( &MultiSpawner::estopCb, this, std::placeholders::_1 ) );
    RCLCPP_INFO( get_logger(), "Waiting for e‑stop topic '%s' to become false…",
                 estop_topic_.c_str() );
  }
}

// --------------------------------------------------------------
void MultiSpawner::estopCb( const std_msgs::msg::Bool::SharedPtr msg )
{
  if ( !started_ && !msg->data ) {
    RCLCPP_INFO( get_logger(), "E‑stop released — commencing startup sequence." );
    start_sequence();
  }
}

// --------------------------------------------------------------
void MultiSpawner::start_sequence()
{
  RCLCPP_INFO( get_logger(),
               "[MultiControllerSpawner] Starting hardware & controller activation sequence." );
  started_ = true;
  long retry_delay = static_cast<long>( retry_delay_ * 1e9 ); // convert to nanoseconds

  // --- Hardware Interfaces ---
  for ( const auto &hw : hw_interfaces_ ) {
    while ( rclcpp::ok() ) {
      if ( loadAndActivateHardware( hw ) ) {
        RCLCPP_INFO( get_logger(), "Hardware '%s' is active.", hw.c_str() );
        break;
      }
      RCLCPP_WARN( get_logger(), "Hardware '%s' failed to start – retrying in %.1fs", hw.c_str(),
                   retry_delay_ );
      rclcpp::sleep_for( std::chrono::nanoseconds( retry_delay ) );
    }
  }

  // --- Controllers ---
  for ( const auto &ctrl : controllers_ ) {
    const auto &cfg = controller_cfg_.at( ctrl );
    while ( rclcpp::ok() ) {
      if ( loadController( ctrl, cfg.activate ) ) {
        RCLCPP_INFO( get_logger(), "Controller '%s' ready%s.", ctrl.c_str(),
                     cfg.activate ? " & active" : " (inactive)" );
        break;
      }

      if ( !cfg.retry_on_failure ) {
        RCLCPP_ERROR( get_logger(), "Controller '%s' failed & retry disabled — giving up.",
                      ctrl.c_str() );
        break;
      }
      RCLCPP_WARN( get_logger(), "Controller '%s' failed — retrying in %.1fs", ctrl.c_str(),
                   retry_delay_ );
      rclcpp::sleep_for( std::chrono::nanoseconds( retry_delay ) );
    }
  }

  RCLCPP_INFO( get_logger(), "[MultiControllerSpawner]  All requested hardware & controllers are "
                             "processed. Shutting down." );
  rclcpp::shutdown();
}

// --------------------------------------------------------------
bool MultiSpawner::loadAndActivateHardware( const std::string &name )
{
  if ( !set_hw_state_client_->wait_for_service( std::chrono::seconds( 3 ) ) ) {
    RCLCPP_WARN(
        get_logger(),
        "[MultiControllerSpawner] Service /set_hardware_component_state not available yet." );
    return false;
  }
  // 2) Activate
  auto act_req = std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>();
  act_req->name = name;
  act_req->target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  act_req->target_state.label = "active";
  auto act_future = set_hw_state_client_->async_send_request( act_req );
  if ( rclcpp::spin_until_future_complete( shared_from_this(), act_future ) !=
       rclcpp::FutureReturnCode::SUCCESS ) {
    return false;
  }
  return act_future.get()->ok;
}

// --------------------------------------------------------------
bool MultiSpawner::loadController( const std::string &name, bool activate )
{
  if ( !load_ctrl_client_->wait_for_service( 2s ) ) {
    RCLCPP_DEBUG( get_logger(), "Service %s not available yet.",
                  load_ctrl_client_->get_service_name() );
    return false;
  }

  // 1) Load
  auto load_req = std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
  load_req->name = name;
  auto load_future = load_ctrl_client_->async_send_request( load_req );
  if ( rclcpp::spin_until_future_complete( shared_from_this(), load_future ) !=
       rclcpp::FutureReturnCode::SUCCESS ) {
    return false;
  }
  if ( !load_future.get()->ok ) {
    RCLCPP_ERROR( get_logger(), "Loading controller '%s' failed", name.c_str() );
    return false;
  }

  if ( !activate ) {
    return true; // loaded only
  }

  // 2) Activate
  if ( !switch_ctrl_client_->wait_for_service( 2s ) ) {
    return false;
  }
  auto sw_req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  sw_req->activate_controllers.push_back( name );
  sw_req->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  sw_req->timeout = rclcpp::Duration::from_seconds( 0.0 );
  auto sw_future = switch_ctrl_client_->async_send_request( sw_req );
  if ( rclcpp::spin_until_future_complete( shared_from_this(), sw_future ) !=
       rclcpp::FutureReturnCode::SUCCESS ) {
    return false;
  }
  return sw_future.get()->ok;
}

} // namespace hector_controller_spawner

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  auto node = std::make_shared<hector_controller_spawner::MultiSpawner>();
  node->initialize();
  rclcpp::spin( node );
  return 0;
}