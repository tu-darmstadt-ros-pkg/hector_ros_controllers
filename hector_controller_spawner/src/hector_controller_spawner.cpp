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

  for ( const auto &ctrl : controllers_ ) {
    ControllerCfg cfg;
    cfg.activate = this->declare_parameter<bool>( ctrl + ".activate", true );
    cfg.retry_on_failure = this->declare_parameter<bool>( ctrl + ".retry_on_failure", false );
    controller_cfg_[ctrl] = cfg;
  }

  // output parameters for debugging
  std::stringstream ss;
  ss << "Parameters:\n";
  ss << "  hardware_interfaces: " << hw_interfaces_.size() << "\n";
  for ( const auto &hw : hw_interfaces_ ) { ss << "    - " << hw << "\n"; }
  ss << "  controllers: " << controllers_.size() << "\n";
  for ( const auto &ctrl : controllers_ ) {
    ss << "    - " << ctrl << " (activate: " << controller_cfg_.at( ctrl ).activate << ")\n";
  }
  ss << "  retry_delay: " << retry_delay_ << " seconds\n";
  ss << "  estop_topic: '" << estop_topic_ << "'\n";
  RCLCPP_INFO( get_logger(), "%s", ss.str().c_str() );

  // 2) Create service clients
  set_hw_state_client_ = this->create_client<controller_manager_msgs::srv::SetHardwareComponentState>(
      "controller_manager/set_hardware_component_state" );
  load_ctrl_client_ = this->create_client<controller_manager_msgs::srv::LoadController>(
      "controller_manager/load_controller" );
  switch_ctrl_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
      "controller_manager/switch_controller" );
  list_ctrl_client_ = this->create_client<controller_manager_msgs::srv::ListControllers>(
      "controller_manager/list_controllers" );
  configure_ctrl_client_ = this->create_client<controller_manager_msgs::srv::ConfigureController>(
      "controller_manager/configure_controller" );
  cm_param_client_ =
      std::make_shared<rclcpp::AsyncParametersClient>( shared_from_this(), "controller_manager" );

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
  started_ = true;

  const auto sleep_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>( retry_delay_ ) );

  // ===== Controller Manager Availability =================================
  while ( rclcpp::ok() && !list_ctrl_client_->wait_for_service( sleep_ns ) ) {
    RCLCPP_WARN( get_logger(), "Controller Manager is not yet available %.1fs", retry_delay_ );
  }

  // ===== Copy Parameters =================================
  // replicateParamsToCM();

  // ===== Hardware =========================================================
  for ( const auto &hw : hw_interfaces_ ) {
    while ( rclcpp::ok() ) {
      if ( loadAndActivateHardware( hw ) ) {
        RCLCPP_INFO( get_logger(), "Hardware '%s' is active.", hw.c_str() );
        break;
      }
      RCLCPP_WARN( get_logger(), "Hardware '%s' failed – retrying in %.1fs", hw.c_str(),
                   retry_delay_ );
      rclcpp::sleep_for( sleep_ns );
    }
  }

  // ===== Controllers ======================================================
  // 0) Snapshot current controller states once -----------------------------
  std::unordered_map<std::string, std::string> current_state; // name → state string

  if ( list_ctrl_client_->wait_for_service( 2s ) ) {
    auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    auto fut = list_ctrl_client_->async_send_request( req );
    if ( rclcpp::spin_until_future_complete( shared_from_this(), fut ) ==
         rclcpp::FutureReturnCode::SUCCESS ) {
      auto resp = fut.get();
      for ( const auto &c : resp->controller )
        current_state[c.name] = c.state; // ACTIVE / inactive / etc.
    }
  }

  // 1) Decide what to load / activate / deactivate -------------------------
  std::vector<std::string> to_load;
  std::vector<std::string> to_activate;
  std::vector<std::string> to_deactivate;

  // a) Pass 1 – deal with requested controllers
  for ( const auto &name : controllers_ ) {
    const auto cfg = controller_cfg_.at( name );
    const auto it = current_state.find( name );

    const bool present = ( it != current_state.end() );
    const bool active = present && ( it->second == "active" || it->second == "ACTIVE" );

    if ( !present )
      to_load.push_back( name );

    if ( cfg.activate ) {
      if ( !active )
        to_activate.push_back( name );
    } else // requested inactive
    {
      if ( active )
        to_deactivate.push_back( name );
    }
  }

  // b) Pass 2 – any other active controllers that should be shut down?
  for ( const auto &[name, state] : current_state ) {
    if ( state == "active" || state == "ACTIVE" ) {
      // if not in our list *or* listed but with activate=false we already handled
      if ( std::find( controllers_.begin(), controllers_.end(), name ) == controllers_.end() )
        to_deactivate.push_back( name );
    }
  }

  // 2) Load missing controllers (one service call per controller) ----------
  for ( const auto &name : to_load ) {
    while ( rclcpp::ok() ) {
      if ( loadController( name ) ) {
        RCLCPP_INFO( get_logger(), "Controller '%s' loaded.", name.c_str() );
        break;
      }
      RCLCPP_WARN( get_logger(), "Failed to load '%s' – retrying in %.1fs", name.c_str(),
                   retry_delay_ );
      rclcpp::sleep_for( sleep_ns );
    }
  }

  // 2.5) Configure missing controllers
  for ( const auto &name : to_load ) {
    while ( rclcpp::ok() ) {
      if ( configureController( name ) ) {
        RCLCPP_INFO( get_logger(), "Controller '%s' configured.", name.c_str() );
        break;
      }
      RCLCPP_WARN( get_logger(), "Failed to configure '%s' – retrying in %.1fs", name.c_str(),
                   retry_delay_ );
      rclcpp::sleep_for( sleep_ns );
    }
  }

  // 3) Single switch_controller call --------------------------------------
  if ( !to_activate.empty() || !to_deactivate.empty() ) {
    if ( !switch_ctrl_client_->wait_for_service( 2s ) ) {
      RCLCPP_ERROR( get_logger(),
                    "switch_controller service unavailable – cannot activate/deactivate batch" );
    } else {
      const auto sw_req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
      sw_req->activate_controllers = to_activate;
      sw_req->deactivate_controllers = to_deactivate;
      sw_req->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
      sw_req->timeout = rclcpp::Duration::from_seconds( 5.0 );

      auto sw_future = switch_ctrl_client_->async_send_request( sw_req );
      if ( rclcpp::spin_until_future_complete( shared_from_this(), sw_future ) !=
               rclcpp::FutureReturnCode::SUCCESS ||
           !sw_future.get()->ok ) {
        RCLCPP_WARN( get_logger(),
                     "Batch switch_controller call failed – controllers may be in mixed states" );
      } else {
        RCLCPP_INFO( get_logger(), "Batch activation/deactivation complete (A:%zu, D:%zu).",
                     to_activate.size(), to_deactivate.size() );
      }
    }
  } else {
    RCLCPP_INFO( get_logger(), "All required controllers are loaded and in the desired state. No "
                               "further activation necessary." );
  }

  // ===== Done =============================================================
  verifyFinalStates();
  RCLCPP_INFO( get_logger(), " Multi Controller Spawner complete – shutting down." );
  done_.store( true );
}

bool MultiSpawner::loadAndActivateHardware( const std::string &name )
{
  if ( !set_hw_state_client_->wait_for_service( std::chrono::seconds( 3 ) ) ) {
    RCLCPP_WARN( get_logger(), "[MultiControllerSpawner] Service %s not available yet.",
                 set_hw_state_client_->get_service_name() );
    return false;
  }
  // 2) Activate
  const auto act_req =
      std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>();
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

bool MultiSpawner::loadController( const std::string &name )
{
  if ( !load_ctrl_client_->wait_for_service( 2s ) )
    return false;

  const auto req = std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
  req->name = name;
  auto fut = load_ctrl_client_->async_send_request( req );
  return rclcpp::spin_until_future_complete( shared_from_this(), fut ) ==
             rclcpp::FutureReturnCode::SUCCESS &&
         fut.get()->ok;
}

bool MultiSpawner::configureController( const std::string &name )
{
  if ( !configure_ctrl_client_->wait_for_service( 2s ) )
    return false;
  const auto req = std::make_shared<controller_manager_msgs::srv::ConfigureController::Request>();
  req->name = name;
  auto fut = configure_ctrl_client_->async_send_request( req );
  return rclcpp::spin_until_future_complete( shared_from_this(), fut ) ==
             rclcpp::FutureReturnCode::SUCCESS &&
         fut.get()->ok;
}

/*bool MultiSpawner::replicateParamsToCM()
{
  // gather *all* current parameters of this node
  const auto names = this->list_parameters( {}, 10 ).names;
  std::vector<rclcpp::Parameter> params;
  params.reserve( names.size() );
  for ( const auto &n : names ) {
    if ( n.find( "qos_overrides" ) == std::string::npos ) {
      params.push_back( this->get_parameter( n ) );
      RCLCPP_INFO( get_logger(), "[MultiControllerSpawner] Adding Parameter %s.", n.c_str() );
    }
  }

  // single atomic set_parameters call
  auto fut = cm_param_client_->set_parameters_atomically( params );
  if ( rclcpp::spin_until_future_complete( shared_from_this(), fut ) !=
       rclcpp::FutureReturnCode::SUCCESS )
    return false;

  const auto result = fut.get();
  if ( !result.successful ) {
    RCLCPP_ERROR( get_logger(), "Atomic parameter set failed: %s", result.reason.c_str() );
  } else {
    RCLCPP_INFO( get_logger(), "Atomic parameter set successful." );
  }
  return result.successful;
}*/
void MultiSpawner::verifyFinalStates()
{
  static const char *GREEN = "\033[32m";
  static const char *RED = "\033[31m";
  static const char *RESET = "\033[0m";

  if ( !list_ctrl_client_->wait_for_service( 2s ) ) {
    RCLCPP_WARN( get_logger(),
                 "Cannot verify final controller states – list_controllers unavailable." );
    return;
  }

  auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto fut = list_ctrl_client_->async_send_request( req );
  if ( rclcpp::spin_until_future_complete( shared_from_this(), fut ) !=
       rclcpp::FutureReturnCode::SUCCESS ) {
    RCLCPP_WARN( get_logger(), "Failed to query controller states for final verification." );
    return;
  }

  std::unordered_map<std::string, std::string> state;
  auto resp = fut.get();
  for ( const auto &c : resp->controller ) state[c.name] = c.state;

  size_t ok_cnt = 0, fail_cnt = 0;
  std::stringstream report;
  report << "Final controller states:\n";

  for ( const auto &name : controllers_ ) {
    std::string current = state.count( name ) ? state.at( name ) : "missing";
    bool should_be_active = controller_cfg_[name].activate;
    bool success = ( should_be_active && ( current == "active" || current == "ACTIVE" ) ) ||
                   ( !should_be_active &&
                     ( current == "inactive" || current == "configured" || current == "INACTIVE" ) );

    if ( success ) {
      ++ok_cnt;
      report << "  " << GREEN << "✔ " << name << " → " << current << RESET << "\n";
    } else {
      ++fail_cnt;
      report << "  " << RED << "✘ " << name << " → " << current << RESET << "\n";
    }
  }

  report << "Summary: " << ok_cnt << " OK / " << fail_cnt << " failed.";
  if ( fail_cnt == 0 )
    RCLCPP_INFO( get_logger(), "%s", report.str().c_str() );
  else
    RCLCPP_WARN( get_logger(), "%s", report.str().c_str() );
}
} // namespace hector_controller_spawner

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );

  auto node = std::make_shared<hector_controller_spawner::MultiSpawner>();
  node->initialize();

  using namespace std::chrono_literals;
  while ( rclcpp::ok() && !node->is_finished() ) {
    rclcpp::spin_some( node );
    std::this_thread::sleep_for( 50ms );
  }

  node.reset();
  rclcpp::shutdown();
  return 0;
}
