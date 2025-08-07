#include "hector_controller_spawner/hector_controller_spawner.hpp"
#include <functional>

namespace hector_controller_spawner
{

using namespace std::chrono_literals;

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
  restart_after_estop_deactivation_ =
      this->declare_parameter<bool>( "restart_after_estop_deactivation", true );

  for ( const auto &ctrl : controllers_ ) {
    ControllerCfg cfg;
    cfg.activate = this->declare_parameter<bool>( ctrl + ".activate", true );
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
  RCLCPP_DEBUG( get_logger(), "%s", ss.str().c_str() );

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
  list_hardware_ctrl_client_ =
      this->create_client<controller_manager_msgs::srv::ListHardwareComponents>(
          "controller_manager/list_hardware_components" );
  cm_param_client_ =
      std::make_shared<rclcpp::AsyncParametersClient>( shared_from_this(), "controller_manager" );

  // 3) Handle e‑stop logic
  if ( estop_topic_.empty() ) {
    released_ = true;
  } else {
    estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        estop_topic_, rclcpp::QoS( 1 ).transient_local(),
        std::bind( &MultiSpawner::estopCb, this, std::placeholders::_1 ) );
    RCLCPP_INFO( get_logger(), "Waiting for e‑stop topic '%s' to become false…",
                 estop_topic_.c_str() );
  }
}

void MultiSpawner::estopCb( const std_msgs::msg::Bool::SharedPtr msg )
{
  if ( !in_progress_ && !msg->data ) {
    RCLCPP_INFO( get_logger(), "E‑stop released — commencing startup sequence." );
    done_ = false;
  }
  released_ = !msg->data;
}

void MultiSpawner::start_sequence( bool initial_init )
{
  in_progress_ = true;

  const auto sleep_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>( retry_delay_ ) );

  // ===== Controller Manager Availability =================================
  while ( rclcpp::ok() && !list_ctrl_client_->wait_for_service( sleep_ns ) ) {
    RCLCPP_WARN( get_logger(), "Controller Manager is not yet available %.1fs", retry_delay_ );
  }

  if ( initial_init ) {
    // ===== Copy Parameters ==================================================
    // replicateParamsToCM();

  } else {
    // ===== Restart Necessary ?  ==============================================
    // test if hardware interfaces are available -> if not redo start sequence
    auto list_hw_fut = list_hardware_ctrl_client_->async_send_request(
        std::make_shared<controller_manager_msgs::srv::ListHardwareComponents::Request>() );
    if ( rclcpp::spin_until_future_complete( shared_from_this(), list_hw_fut ) !=
         rclcpp::FutureReturnCode::SUCCESS ) {
      RCLCPP_WARN( get_logger(), "Failed to list hardware components" );
    }
    auto list_hw_resp = list_hw_fut.get();
    size_t active_hw_interfaces = 0;
    for ( const auto &hw : list_hw_resp->component ) {
      if ( hw.state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE &&
           std::find( hw_interfaces_.begin(), hw_interfaces_.end(), hw.name ) !=
               hw_interfaces_.end() ) {
        active_hw_interfaces++;
      }
    }
    if ( active_hw_interfaces == hw_interfaces_.size() ) {
      RCLCPP_INFO( get_logger(),
                   "Hardware interfaces are still available after e-stop deactivation. No need to "
                   "redo hw interface and controller start sequence." );
      in_progress_ = false;
      done_ = true;
      return;
    }
    RCLCPP_INFO( get_logger(), "Hardware Interface must be reactivated after estop deactivation" );
  }

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
      parseControllerInfo( *resp, current_state );
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
    const bool active = present && ( it->second == "active" );

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
    if ( state == "active" ) {
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

  // 2.5) Re-request current state -> chained info only after configuring available----------------
  if ( !to_load.empty() && list_ctrl_client_->wait_for_service( 2s ) ) {
    current_state.clear();
    auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    auto fut = list_ctrl_client_->async_send_request( req );
    if ( rclcpp::spin_until_future_complete( shared_from_this(), fut ) ==
         rclcpp::FutureReturnCode::SUCCESS ) {
      auto resp = fut.get();
      parseControllerInfo( *resp, current_state );
    }
  }

  // 3) Group switch_controller calls --------------------------------------
  auto switch_controllers = [&]( const std::vector<std::string> &activate,
                                 const std::vector<std::string> &deactivate ) -> bool {
    if ( !switch_ctrl_client_->wait_for_service( 2s ) ) {
      RCLCPP_ERROR( get_logger(), "switch_controller service unavailable" );
      return false;
    }
    auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    req->activate_controllers = activate;
    req->deactivate_controllers = deactivate;
    req->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    req->timeout = rclcpp::Duration::from_seconds( 5.0 );

    auto fut = switch_ctrl_client_->async_send_request( req );
    return rclcpp::spin_until_future_complete( shared_from_this(), fut ) ==
               rclcpp::FutureReturnCode::SUCCESS &&
           fut.get()->ok;
  };

  // deactivate controllers that are active but not requested
  if ( !to_deactivate.empty() ) {
    std::stringstream ss;
    for ( size_t i = 0; i < to_deactivate.size(); ++i ) {
      ss << to_deactivate[i] << ( i + 1 < to_deactivate.size() ? ", " : "" );
    }
    if ( !switch_controllers( {}, to_deactivate ) ) {
      RCLCPP_ERROR( get_logger(), "Failed to deactivate controllers: %s", ss.str().c_str() );
    } else {
      RCLCPP_INFO( get_logger(), "Deactivated controllers: %s", ss.str().c_str() );
    }
  }
  // print controller configuration
  std::stringstream ss;
  for ( const auto &[name, cfg] : controller_cfg_ ) {
    ss << "Controller '" << name << "' is "
       << ( controller_cfg_.at( name ).activate ? "ACTIVE" : "INACTIVE" ) << "\n";
  }
  RCLCPP_INFO( get_logger(), "%s", ss.str().c_str() );
  for ( const auto &group : controller_groups_ ) {

    /* Determine whether at least one controller in this group should be active. */
    bool group_requested_active = false;
    for ( const auto &m : group ) {
      RCLCPP_INFO( get_logger(), "Controller '%s' in group: %s", m.c_str(),
                   vecToString( group ).c_str() );
      group_requested_active |= controller_cfg_.at( m ).activate;
    }
    if ( !group_requested_active ) {
      continue; // whole group requested inactive
    }

    /* Force any “false” members in the same group to active and warn once. */
    for ( const auto &m : group ) {
      if ( !controller_cfg_.at( m ).activate ) {
        RCLCPP_WARN( get_logger(), "Controller '%s' is in group with ['%s'] → overriding to ACTIVE.",
                     m.c_str(), vecToString( group ).c_str() );
      }
    }

    /* Skip activation if entire group is already active. */
    bool already_active = true;
    for ( const auto &m : group ) {
      auto it = current_state.find( m );
      already_active &= ( it != current_state.end() && it->second == "active" );
    }
    if ( already_active ) {
      continue;
    }

    /* Issue one switch_controller call for this group. */
    if ( !switch_controllers( group, /*deactivate*/ {} ) ) {
      RCLCPP_ERROR( get_logger(), "Failed to activate controller group containing '%s'",
                    vecToString( group ).c_str() );
    } else {

      RCLCPP_INFO( get_logger(), "Activated controller group: %s", vecToString( group ).c_str() );
    }
  }

  // ===== Done =============================================================
  verifyFinalStates();
  RCLCPP_INFO( get_logger(), " Multi Controller Spawner complete !" );
  done_.store( true );
  in_progress_ = false;
}

void MultiSpawner::parseControllerInfo(
    const controller_manager_msgs::srv::ListControllers_Response &resp,
    std::unordered_map<std::string, std::string> &current_state )
{
  RCLCPP_WARN( get_logger(), "Received %zu controllers from list_controllers service.",
               resp.controller.size() );
  // save snapshot of states
  for ( const auto &c : resp.controller ) { current_state[c.name] = c.state; }

  // —— auto-detect chained controllers groups ——
  for ( const auto &c : resp.controller ) {
    for ( const auto &conn : c.chain_connections ) {
      // check if there is a ControllerGroup that includes name or conn.name
      bool found = false;
      for ( auto &group : controller_groups_ ) {
        if ( std::find( group.begin(), group.end(), c.name ) != group.end() ||
             std::find( group.begin(), group.end(), conn.name ) != group.end() ) {
          found = true;
          if ( std::find( group.begin(), group.end(), c.name ) == group.end() ) {
            group.push_back( c.name );
          } else if ( std::find( group.begin(), group.end(), conn.name ) == group.end() ) {
            group.push_back( conn.name );
          }
          break;
        }
      }
      if ( !found ) {
        // create a new group with both names
        controller_groups_.emplace_back( std::vector{ c.name, conn.name } );
      }
      RCLCPP_DEBUG( get_logger(), "Controller Group Member: '%s' <-> '%s'", c.name.c_str(),
                    conn.name.c_str() );
    }
  }
  // add remaining controllers as their own groups
  for ( const auto &c : resp.controller ) {
    bool in_group = false;
    for ( auto &group : controller_groups_ ) {
      if ( std::find( group.begin(), group.end(), c.name ) != group.end() ) {
        in_group = true;
        break;
      }
    }
    if ( !in_group ) {
      controller_groups_.emplace_back( std::vector<std::string>{ c.name } );
      RCLCPP_DEBUG( get_logger(), "Controller Group Member: '%s' (single)", c.name.c_str() );
    }
  }
  // check if all controllers have a controller cfg, if not add them with current state
  for ( const auto &c : resp.controller ) {
    if ( controller_cfg_.find( c.name ) == controller_cfg_.end() ) {
      ControllerCfg cfg;
      cfg.activate = ( c.state == "active" );
      controller_cfg_[c.name] = cfg;
      controllers_.push_back( c.name );
      RCLCPP_DEBUG( get_logger(), "Adding controller '%s' with state '%s' to config.",
                    c.name.c_str(), c.state.c_str() );
    }
  }
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
  const auto resp = fut.get();
  for ( const auto &c : resp->controller ) state[c.name] = c.state;

  size_t ok_cnt = 0, fail_cnt = 0;
  std::stringstream report;
  report << "\nFinal controller states:\n";

  for ( const auto &name : controllers_ ) {
    std::string current = state.count( name ) ? state.at( name ) : "missing";
    bool should_be_active = controller_cfg_[name].activate;
    bool success = ( should_be_active && ( current == "active" ) ) ||
                   ( !should_be_active && ( current == "inactive" || current == "configured" ) );

    if ( success ) {
      ++ok_cnt;
      report << "  " << GREEN << "✔ " << name << " → " << current << RESET << "\n";
    } else {
      ++fail_cnt;
      report << "  " << RED << "✘ " << name << " → " << current << RESET << "\n";
    }
  }
  report << ( ( fail_cnt > 0 ) ? RED : GREEN ) << "Summary: " << ok_cnt << " OK / " << fail_cnt
         << " failed." << RESET;
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
  bool initial_init = true;
  using namespace std::chrono_literals;
  while ( rclcpp::ok() ) {
    rclcpp::spin_some( node );
    if ( node->estop_released_and_not_in_progress() ) {
      node->start_sequence( initial_init ); // safe – not inside another callback
      if ( !node->is_tracking_estop() || !node->restart_after_estop_deactivation() )
        break;
      initial_init = false;
    }
    std::this_thread::sleep_for( 50ms );
  }

  node.reset();
  rclcpp::shutdown();
  return 0;
}
