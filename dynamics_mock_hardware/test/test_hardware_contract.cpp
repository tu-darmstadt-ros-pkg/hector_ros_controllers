// The plugin contract of hector_controller_manager, checked on the dynamics mock

#include <memory>
#include <string>

#include "dynamics_mock_hardware/dynamics_mock_hardware.hpp"
#include "hardware_contract_tests/hardware_contract_tests.hpp"
#include "hardware_interface/component_parser.hpp"

namespace
{

const char *const PENDULUM = R"(<?xml version="1.0"?>
<robot name="pendulum">
  <link name="world"/>
  <joint name="joint1" type="revolute">
    <parent link="world"/>
    <child link="link1"/>
    <axis xyz="0 1 0"/>
    <limit effort="100.0" lower="-3.0" upper="3.0" velocity="10.0"/>
  </joint>
  <link name="link1">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <ros2_control name="pendulum" type="system">
    <hardware>
      <plugin>dynamics_mock_hardware/DynamicsMockHardware</plugin>
      <param name="gravity_z">0.0</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <command_interface name="torque_off"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
</robot>
)";

// The mock's state is its plant, so setting the position is what a hand does to the joint
class PushableMock : public dynamics_mock_hardware::DynamicsMockHardware
{
public:
  void push( const std::string &joint, double delta )
  { set_state( joint + "/position", get_state<double>( joint + "/position" ) + delta ); }
};

struct DynamicsMockPlant {
  std::unique_ptr<hardware_interface::SystemInterface> make()
  {
    auto mock = std::make_unique<PushableMock>();
    mock_ = mock.get();
    return mock;
  }
  hardware_interface::HardwareInfo info() const
  { return hardware_interface::parse_control_resources_from_urdf( PENDULUM ).at( 0 ); }
  std::string joint() const { return "joint1"; }
  void step( double /*period*/ ) { }
  void push( double delta ) { mock_->push( joint(), delta ); }

  PushableMock *mock_{ nullptr };
};

} // namespace

namespace hardware_contract_tests
{
INSTANTIATE_TYPED_TEST_SUITE_P( DynamicsMock, HardwareContract, DynamicsMockPlant );
} // namespace hardware_contract_tests
