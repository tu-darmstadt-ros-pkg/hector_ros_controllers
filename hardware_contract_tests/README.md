# hardware_contract_tests

A gtest suite that checks a ros2_control hardware plugin against what `hector_controller_manager`
relies on. These parts of the plugin contract and the torque_off decisions are checked:

- a stop alone holds a moving joint, and a start alone takes commands;
- a NaN position command stops a moving joint within its braking distance (0.1 rad) and holds it
  there, and a NaN velocity command stops it within 0.05 rad;
- `on_error` returns SUCCESS, and the component recovers through configure and activate with no
  claim left;
- a `torque_off` claim alone leaves the joint limp, it is refused next to a motion interface of
  the same joint, a stop alone keeps the joint limp, and a motion start energises it where it was
  left;
- deactivation stops a moving joint, and activation holds it. A plant that moves only in `write()`
  stands still while inactive, so on the dynamics mock only the activation half can fail.

The suite does not check a started interface that never gets a command, the timing rows, or
`write()` returning ERROR, which is a demand on hcm.

The suite drives one component directly from the test's thread, one call at a time, as hcm does.

## Use

Add `<test_depend>hardware_contract_tests</test_depend>`, link the gtest against
`hardware_contract_tests::hardware_contract_tests`, and give the suite a plant:

```cpp
#include <hardware_contract_tests/hardware_contract_tests.hpp>

struct MyPlant {
  std::unique_ptr<hardware_interface::SystemInterface> make();
  hardware_interface::HardwareInfo info();  // the component to load
  std::string joint();                      // has position, velocity and torque_off
  void step( double period );               // physics that write() does not run
  void push( double delta );                // moves the joint by hand
};

namespace hardware_contract_tests
{
INSTANTIATE_TYPED_TEST_SUITE_P( MyPlugin, HardwareContract, MyPlant );
}
```

The joint must be free to move one radian either way, without gravity. See
`dynamics_mock_hardware/test/test_hardware_contract.cpp` and Dynamixel's `test_hw_contract.cpp`.
