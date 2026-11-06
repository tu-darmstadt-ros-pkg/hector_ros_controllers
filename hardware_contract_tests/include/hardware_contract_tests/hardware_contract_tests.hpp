// Copyright (c) 2026 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#ifndef HARDWARE_CONTRACT_TESTS__HARDWARE_CONTRACT_TESTS_HPP_
#define HARDWARE_CONTRACT_TESTS__HARDWARE_CONTRACT_TESTS_HPP_

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace hardware_contract_tests
{

/**
 * What hector_controller_manager relies on from a hardware plugin, checked on one joint.
 *
 * A plugin's test instantiates the suite with its own Plant type:
 *
 *   INSTANTIATE_TYPED_TEST_SUITE_P( MyPlugin, HardwareContract, MyPlant );
 *
 * The Plant provides
 * - `std::unique_ptr<hardware_interface::SystemInterface> make()`, the plugin to test;
 * - `hardware_interface::HardwareInfo info()`, the component it loads;
 * - `std::string joint()`, a joint with position, velocity and torque_off command interfaces, free
 *   to move one radian either way, in a plant without gravity;
 * - `void step( double period )`, the plant's physics that write() does not run;
 * - `void push( double delta )`, moves the joint by hand.
 *
 * Every call comes from the test's thread, one at a time, as hcm calls a component.
 */
template<typename Plant>
class HardwareContract : public ::testing::Test
{
protected:
  static constexpr double kPeriod = 0.02;
  static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

  void SetUp() override
  {
    if ( !rclcpp::ok() )
      rclcpp::init( 0, nullptr );
    hardware_ = plant_.make();
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = plant_.info();
    params.hardware_info.rw_rate = static_cast<unsigned int>( std::lround( 1.0 / kPeriod ) );
    params.clock = std::make_shared<rclcpp::Clock>( RCL_ROS_TIME );
    params.logger = rclcpp::get_logger( "hardware_contract" );
    ASSERT_EQ( hardware_->init( params ), hardware_interface::CallbackReturn::SUCCESS );
    for ( const auto &state : hardware_->on_export_state_interfaces() )
      states_[state->get_name()] = state;
    for ( const auto &command : hardware_->on_export_command_interfaces() )
      commands_[command->get_name()] = command;
    ASSERT_EQ( hardware_->on_configure( state_ ), hardware_interface::CallbackReturn::SUCCESS );
    ASSERT_EQ( hardware_->on_activate( state_ ), hardware_interface::CallbackReturn::SUCCESS );
    active_ = true;
    cycles( 5 );
  }

  void TearDown() override
  {
    if ( active_ )
      hardware_->on_deactivate( state_ );
  }

  std::string interface( const std::string &name ) const { return plant_.joint() + "/" + name; }

  double position() const { return states_.at( interface( "position" ) )->get_optional().value(); }

  void command( const std::string &name, double value )
  { ASSERT_TRUE( commands_.at( interface( name ) )->set_value( value, true ) ); }

  hardware_interface::return_type prepare( const std::vector<std::string> &start,
                                           const std::vector<std::string> &stop )
  { return hardware_->prepare_command_mode_switch( start, stop ); }

  /// A switch as hcm performs it, which leaves NaN in the started interfaces until a controller writes
  void perform( const std::vector<std::string> &start, const std::vector<std::string> &stop )
  {
    ASSERT_EQ( hardware_->perform_command_mode_switch( start, stop ),
               hardware_interface::return_type::OK );
    for ( const auto &name : start )
      if ( name != interface( "torque_off" ) )
        ASSERT_TRUE( commands_.at( name )->set_value( kNaN, true ) );
  }

  /// Like the stock resource manager, an inactive component is read but not written
  void cycles( int count )
  {
    const rclcpp::Duration period = rclcpp::Duration::from_seconds( kPeriod );
    for ( int i = 0; i < count; ++i ) {
      time_ += period;
      ASSERT_EQ( hardware_->read( time_, period ), hardware_interface::return_type::OK );
      if ( active_ )
        ASSERT_EQ( hardware_->write( time_, period ), hardware_interface::return_type::OK );
      plant_.step( kPeriod );
    }
  }

  /// The joint stays at @p held, and comes back there after a push
  void expectHeldAt( double held, const std::string &context )
  {
    cycles( 25 );
    EXPECT_NEAR( position(), held, 0.02 ) << context << ": the joint did not hold";
    plant_.push( 0.2 );
    cycles( 50 );
    EXPECT_NEAR( position(), held, 0.02 ) << context << ": the joint is limp";
  }

  /// The joint stays where a push leaves it
  void expectLimp( const std::string &context )
  {
    cycles( 5 );
    const double before = position();
    plant_.push( 0.2 );
    cycles( 50 );
    EXPECT_NEAR( position(), before + 0.2, 0.02 ) << context << ": the joint has torque";
  }

  Plant plant_;
  std::unique_ptr<hardware_interface::SystemInterface> hardware_;
  std::unordered_map<std::string, hardware_interface::StateInterface::ConstSharedPtr> states_;
  std::unordered_map<std::string, hardware_interface::CommandInterface::SharedPtr> commands_;
  const rclcpp_lifecycle::State state_;
  rclcpp::Time time_{ 0, 0, RCL_ROS_TIME };
  bool active_{ false };
};

TYPED_TEST_SUITE_P( HardwareContract );

TYPED_TEST_P( HardwareContract, ANaNPositionCommandHoldsWhereTheJointIs )
{
  const double start = this->position();
  this->perform( { this->interface( "position" ) }, {} );
  this->command( "position", start + 1.0 );
  this->cycles( 10 );
  const double at_nan = this->position();
  ASSERT_GT( std::abs( at_nan - start ), 0.2 ) << "the joint was not moving";
  this->command( "position", this->kNaN );
  this->cycles( 25 );
  const double held = this->position();
  // A motor needs its braking distance
  EXPECT_NEAR( held, at_nan, 0.1 ) << "the joint did not stop where the NaN found it";
  this->expectHeldAt( held, "after the NaN" );
}

TYPED_TEST_P( HardwareContract, AStopAloneHoldsAMovingJoint )
{
  const double start = this->position();
  this->perform( { this->interface( "velocity" ) }, {} );
  this->command( "velocity", 0.5 );
  this->cycles( 25 );
  ASSERT_GT( std::abs( this->position() - start ), 0.05 ) << "the joint was not moving";
  this->perform( {}, { this->interface( "velocity" ) } );
  this->cycles( 10 );
  const double stopped = this->position();
  this->expectHeldAt( stopped, "after the stop" );
}

TYPED_TEST_P( HardwareContract, AStartAloneTakesCommands )
{
  const double target = this->position() + 0.3;
  this->perform( { this->interface( "position" ) }, {} );
  this->command( "position", target );
  this->cycles( 100 );
  EXPECT_NEAR( this->position(), target, 0.02 );
}

TYPED_TEST_P( HardwareContract, TorqueOffAloneGoesLimp )
{
  ASSERT_EQ( this->prepare( { this->interface( "torque_off" ) }, {} ),
             hardware_interface::return_type::OK );
  this->perform( { this->interface( "torque_off" ) }, {} );
  this->expectLimp( "with torque_off claimed" );
}

TYPED_TEST_P( HardwareContract, TorqueOffIsRefusedNextToAMotionInterfaceOfTheSameJoint )
{
  const auto torque_off = this->interface( "torque_off" );
  const auto position = this->interface( "position" );
  EXPECT_EQ( this->prepare( { torque_off, position }, {} ), hardware_interface::return_type::ERROR );
  this->perform( { torque_off }, {} );
  EXPECT_EQ( this->prepare( { position }, {} ), hardware_interface::return_type::ERROR );
  EXPECT_EQ( this->prepare( { position }, { torque_off } ), hardware_interface::return_type::OK );
}

// hcm's hold after a fault stops a claim without a successor, which must not energise
TYPED_TEST_P( HardwareContract, AStoppedTorqueOffStaysLimp )
{
  this->perform( { this->interface( "torque_off" ) }, {} );
  this->perform( {}, { this->interface( "torque_off" ) } );
  this->expectLimp( "after torque_off stopped alone" );
}

TYPED_TEST_P( HardwareContract, AMotionStartInTheSameSwitchEnergisesWhereTheJointWasLeft )
{
  this->perform( { this->interface( "torque_off" ) }, {} );
  this->cycles( 5 );
  this->plant_.push( 0.2 );
  this->cycles( 5 );
  const double left = this->position();
  this->perform( { this->interface( "position" ) }, { this->interface( "torque_off" ) } );
  this->expectHeldAt( left, "after the combined switch" );
}

TYPED_TEST_P( HardwareContract, AMotionStartInALaterSwitchEnergisesWhereTheJointWasLeft )
{
  this->perform( { this->interface( "torque_off" ) }, {} );
  this->perform( {}, { this->interface( "torque_off" ) } );
  this->cycles( 5 );
  this->plant_.push( 0.2 );
  this->cycles( 5 );
  const double left = this->position();
  this->perform( { this->interface( "position" ) }, {} );
  this->expectHeldAt( left, "after the later start" );
}

TYPED_TEST_P( HardwareContract, DeactivationStopsAMovingJointAndActivationHoldsIt )
{
  this->perform( { this->interface( "velocity" ) }, {} );
  this->command( "velocity", 0.5 );
  this->cycles( 25 );
  ASSERT_EQ( this->hardware_->on_deactivate( this->state_ ),
             hardware_interface::CallbackReturn::SUCCESS );
  this->active_ = false;
  this->cycles( 10 );
  const double stopped = this->position();
  this->cycles( 25 );
  EXPECT_NEAR( this->position(), stopped, 0.01 ) << "the inactive joint kept moving";

  ASSERT_EQ( this->hardware_->on_activate( this->state_ ),
             hardware_interface::CallbackReturn::SUCCESS );
  this->active_ = true;
  this->expectHeldAt( stopped, "after the activation" );
}

// A fault ends every claim, also torque_off, and the recovery is configure, then activate
TYPED_TEST_P( HardwareContract, AfterAFaultTheRecoveredJointHoldsAndTakesAMotionClaim )
{
  this->perform( { this->interface( "torque_off" ) }, {} );
  this->cycles( 5 );
  ASSERT_EQ( this->hardware_->on_error( this->state_ ), hardware_interface::CallbackReturn::SUCCESS );
  this->active_ = false;
  ASSERT_EQ( this->hardware_->on_configure( this->state_ ),
             hardware_interface::CallbackReturn::SUCCESS );
  ASSERT_EQ( this->hardware_->on_activate( this->state_ ),
             hardware_interface::CallbackReturn::SUCCESS );
  this->active_ = true;
  this->cycles( 5 );
  this->expectHeldAt( this->position(), "after the recovery" );
  EXPECT_EQ( this->prepare( { this->interface( "position" ) }, {} ),
             hardware_interface::return_type::OK );
}

REGISTER_TYPED_TEST_SUITE_P( HardwareContract, ANaNPositionCommandHoldsWhereTheJointIs,
                             AStopAloneHoldsAMovingJoint, AStartAloneTakesCommands,
                             TorqueOffAloneGoesLimp,
                             TorqueOffIsRefusedNextToAMotionInterfaceOfTheSameJoint,
                             AStoppedTorqueOffStaysLimp,
                             AMotionStartInTheSameSwitchEnergisesWhereTheJointWasLeft,
                             AMotionStartInALaterSwitchEnergisesWhereTheJointWasLeft,
                             DeactivationStopsAMovingJointAndActivationHoldsIt,
                             AfterAFaultTheRecoveredJointHoldsAndTakesAMotionClaim );

} // namespace hardware_contract_tests

#endif // HARDWARE_CONTRACT_TESTS__HARDWARE_CONTRACT_TESTS_HPP_
