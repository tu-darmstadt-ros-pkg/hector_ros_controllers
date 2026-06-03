#include "input_shaping_controller/input_shaping_controller.hpp"

#include <limits>
#include <sstream>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace input_shaping_controller
{

InputShapingController::InputShapingController()
    : controller_interface::ChainableControllerInterface()
{
}

controller_interface::CallbackReturn InputShapingController::on_init()
{
  const auto node = get_node();
  if ( !node ) {
    return controller_interface::CallbackReturn::ERROR;
  }

  try {
    param_listener_ = std::make_shared<ParamListener>( get_node() );
    params_ = param_listener_->get_params();
  } catch ( const std::exception &e ) {
    RCLCPP_WARN( get_node()->get_logger(), "Exception thrown during init stage with message: %s",
                 e.what() );
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
InputShapingController::on_configure( const rclcpp_lifecycle::State & )
{
  if ( params_.joints.empty() ) {
    RCLCPP_ERROR( get_node()->get_logger(), "'joints' parameter must not be empty." );
    return controller_interface::CallbackReturn::ERROR;
  }

  const size_t n = params_.joints.size();

  if (params_.shaper_type.size() != n ||
      params_.shaping_frequency.size() != n ||
      params_.shaping_damping_ratio.size() != n) {
    RCLCPP_ERROR( get_node()->get_logger(), "Input shaping parameter arrays must have the same size as 'joints'." );
    return controller_interface::CallbackReturn::ERROR;
  }

  reference_interfaces_.assign( n, std::numeric_limits<double>::quiet_NaN() );

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
InputShapingController::on_activate( const rclcpp_lifecycle::State & )
{
  shapers_initialized_ = false;
  for ( auto &shaper : shapers_ ) {
    shaper.history_initialized = false;
  }

  for ( auto &ref : reference_interfaces_ ) {
    ref = std::numeric_limits<double>::quiet_NaN();
  }
  
  // check order of command interfaces
  for ( size_t i = 0; i < params_.joints.size(); ++i ) {
    if ( command_interfaces_[i].get_name() != params_.joints[i] + "/position" ) {
      RCLCPP_ERROR( get_node()->get_logger(), "Command interfaces are not in the expected order." );
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO( get_node()->get_logger(), "InputShapingController activated with configuration:" );
  for ( size_t i = 0; i < params_.joints.size(); ++i ) {
    RCLCPP_INFO( get_node()->get_logger(),
                 "  Joint %zu (%s): type='%s', freq=%.2f, zeta=%.2f",
                 i, params_.joints[i].c_str(),
                 params_.shaper_type[i].c_str(),
                 params_.shaping_frequency[i],
                 params_.shaping_damping_ratio[i] );
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
InputShapingController::on_deactivate( const rclcpp_lifecycle::State & )
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
InputShapingController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for ( const auto &j : params_.joints ) {
    conf.names.emplace_back( j + "/position" );
  }
  return conf;
}

controller_interface::InterfaceConfiguration
InputShapingController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  return conf;
}

std::vector<hardware_interface::CommandInterface>
InputShapingController::on_export_reference_interfaces()
{
  const size_t n = params_.joints.size();
  std::vector<hardware_interface::CommandInterface> refs;
  refs.reserve( n );

  const std::string controller_name = get_node()->get_name();
  for ( size_t i = 0; i < n; ++i ) {
    refs.emplace_back( controller_name + "/" + params_.joints[i],
                       hardware_interface::HW_IF_POSITION, &reference_interfaces_[i] );
  }
  return refs;
}

controller_interface::return_type
InputShapingController::update_reference_from_subscribers( const rclcpp::Time &,
                                                           const rclcpp::Duration & )
{
  return controller_interface::return_type::OK;
}

controller_interface::return_type
InputShapingController::update_and_write_commands( const rclcpp::Time &, const rclcpp::Duration &period )
{
  if ( !is_chained_ ) {
    return controller_interface::return_type::OK;
  }

  if ( param_listener_->is_old( params_ ) ) {
    params_ = param_listener_->get_params();
    RCLCPP_INFO( get_node()->get_logger(), "InputShapingController parameters updated:" );
    for ( size_t i = 0; i < params_.joints.size(); ++i ) {
    RCLCPP_INFO( get_node()->get_logger(),
                 "  Joint %zu (%s): type='%s', freq=%.2f, zeta=%.2f",
                 i, params_.joints[i].c_str(),
                 params_.shaper_type[i].c_str(),
                 params_.shaping_frequency[i],
                 params_.shaping_damping_ratio[i] );
  }
  }

  if ( !shapers_initialized_ && period.seconds() > 0.0 ) {
    const double dt = period.seconds();
    const size_t n = params_.joints.size();
    shapers_.resize( n );
    for ( size_t i = 0; i < n; ++i ) {
      ShaperType type = ShaperType::NONE;
      if ( params_.shaper_type[i] == "none" ) {
        type = ShaperType::NONE;
      } else if ( params_.shaper_type[i] == "zv" ) {
        type = ShaperType::ZV;
      } else if ( params_.shaper_type[i] == "zvd" ) {
        type = ShaperType::ZVD;
      } else {
        RCLCPP_WARN( get_node()->get_logger(), "Unknown shaper type '%s' for joint %s, disabling shaping.", 
                     params_.shaper_type[i].c_str(), params_.joints[i].c_str() );
      }
      shapers_[i].init( type, params_.shaping_frequency[i], params_.shaping_damping_ratio[i], dt );
    }
    shapers_initialized_ = true;
  }
  
  bool success = true;
  for ( size_t i = 0; i < params_.joints.size(); ++i ) {
    double ref = reference_interfaces_[i];
    
    if ( shapers_initialized_ && shapers_.size() > i ) {
      double shaped_ref = shapers_[i].process( ref );
      if ( params_.enabled ) {
        ref = shaped_ref;
      }
    }
    
    // forward to command interface
    success &= command_interfaces_[i].set_value( ref );
    
  }

  return success ? controller_interface::return_type::OK : controller_interface::return_type::ERROR;
}

bool InputShapingController::on_set_chained_mode( bool chained_mode )
{
  is_chained_ = chained_mode;
  return true;
}

} // namespace input_shaping_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( input_shaping_controller::InputShapingController,
                        controller_interface::ChainableControllerInterface )
