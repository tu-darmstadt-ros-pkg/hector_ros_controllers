#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <string>
#include <cmath>

#include "input_shaping_controller/input_shaping_controller.hpp"
#include "hardware_interface/loaned_command_interface.hpp"

class InputShapingControllerTest : public ::testing::Test
{
public:
  using ControllerType = input_shaping_controller::InputShapingController;

  std::vector<std::string> joints_{ "joint1", "joint2" };
  std::shared_ptr<ControllerType> controller_;
  
  std::vector<double> hw_cmd_values_;
  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> cmd_ifaces_;

  void SetUp() override {
    rclcpp::init(0, nullptr);
    controller_ = std::make_shared<ControllerType>();
  }

  void TearDown() override {
    controller_.reset();
    rclcpp::shutdown();
  }

  void initController(const std::vector<std::string>& shaper_type,
                      const std::vector<double>& frequency,
                      const std::vector<double>& damping,
                      bool enabled = true)
  {
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "test_input_shaping";
    params.update_rate = 1000;
    params.controller_manager_update_rate = 1000;
    params.node_namespace = "";

    rclcpp::NodeOptions opts;
    opts.parameter_overrides( {
        rclcpp::Parameter( "enabled", enabled ),
        rclcpp::Parameter( "joints", joints_ ),
        rclcpp::Parameter( "shaper_type", shaper_type ),
        rclcpp::Parameter( "shaping_frequency", frequency ),
        rclcpp::Parameter( "shaping_damping_ratio", damping ),
    } );
    params.node_options = opts;

    auto result = controller_->init( params );
    ASSERT_EQ( result, controller_interface::return_type::OK );
  }

  void configureController()
  {
    rclcpp_lifecycle::State unconfigured( lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured" );
    auto cb = controller_->on_configure( unconfigured );
    ASSERT_EQ( cb, controller_interface::CallbackReturn::SUCCESS );

    controller_->on_export_reference_interfaces(); // sizing
  }

  void setupHardwareInterfaces()
  {
    hw_cmd_values_.assign( joints_.size(), 0.0 );
    cmd_ifaces_.clear();

    std::vector<hardware_interface::LoanedCommandInterface> loaned_cmds;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    for ( size_t i = 0; i < joints_.size(); ++i ) {
      cmd_ifaces_.push_back( std::make_shared<hardware_interface::CommandInterface>(
          joints_[i], "position", &hw_cmd_values_[i] ) );
      loaned_cmds.emplace_back( *cmd_ifaces_.back() );
    }
#pragma GCC diagnostic pop

    controller_->assign_interfaces( std::move(loaned_cmds), {} );
  }

  void activateController()
  {
    rclcpp_lifecycle::State inactive( lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive" );
    auto cb = controller_->on_activate( inactive );
    ASSERT_EQ( cb, controller_interface::CallbackReturn::SUCCESS );
  }

  void setReference(size_t index, double value) {
    // Actually we can just export reference interfaces to get pointers
    auto refs = controller_->export_reference_interfaces();
    refs[index]->set_value(value);
  }
};

TEST_F(InputShapingControllerTest, ConfigureFailsSizeMismatch)
{
  initController( {"zv"}, {10.0, 10.0}, {0.1, 0.1} ); // Mismatch size
  rclcpp_lifecycle::State unconfigured( lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured" );
  auto cb = controller_->on_configure( unconfigured );
  EXPECT_EQ( cb, controller_interface::CallbackReturn::ERROR );
}

TEST_F(InputShapingControllerTest, PassthroughWithoutShaping)
{
  initController( {"none", "none"}, {10.0, 10.0}, {0.1, 0.1} );
  configureController();
  setupHardwareInterfaces();
  activateController();

  setReference(0, 1.0);
  setReference(1, 2.0);

  rclcpp::Time time(0, 0);
  rclcpp::Duration period(0, 1000000); // 1ms

  auto ret = controller_->update_and_write_commands(time, period);
  EXPECT_EQ(ret, controller_interface::return_type::OK);

  EXPECT_DOUBLE_EQ(hw_cmd_values_[0], 1.0);
  EXPECT_DOUBLE_EQ(hw_cmd_values_[1], 2.0);
}

TEST_F(InputShapingControllerTest, ZVShaperBehavior)
{
  // joint1: use shaping, freq=10Hz, zeta=0.0
  initController( {"zv", "none"}, {10.0, 10.0}, {0.0, 0.0} );
  configureController();
  setupHardwareInterfaces();
  activateController();

  rclcpp::Time time(0, 0);
  rclcpp::Duration period(0, 1000000); // 1ms

  setReference(0, 1.0);
  setReference(1, 0.0);

  auto ret = controller_->update_and_write_commands(time, period);
  EXPECT_EQ(ret, controller_interface::return_type::OK);

  EXPECT_DOUBLE_EQ(hw_cmd_values_[0], 1.0);
}

TEST_F(InputShapingControllerTest, ZVShaperStepResponse)
{
  // joint1: use shaping, freq=10Hz, zeta=0.0
  // delay_time = 0.5 * period = 0.05s = 50 steps
  initController( {true, false}, {"zv", "zv"}, {10.0, 10.0}, {0.0, 0.0} );
  configureController();
  setupHardwareInterfaces();
  activateController();

  rclcpp::Time time(0, 0);
  rclcpp::Duration period(0, 1000000); // 1ms

  setReference(0, 0.0);
  controller_->update_and_write_commands(time, period);
  EXPECT_DOUBLE_EQ(hw_cmd_values_[0], 0.0);

  setReference(0, 1.0);
  
  controller_->update_and_write_commands(time, period);
  EXPECT_NEAR(hw_cmd_values_[0], 0.5, 1e-6);

  for (int i=0; i<48; i++) {
    controller_->update_and_write_commands(time, period);
    EXPECT_NEAR(hw_cmd_values_[0], 0.5, 1e-6);
  }

  controller_->update_and_write_commands(time, period);
  EXPECT_NEAR(hw_cmd_values_[0], 1.0, 1e-6);
}

TEST_F(InputShapingControllerTest, ZVDShaperStepResponse)
{
  initController( {"zvd", "none"}, {10.0, 10.0}, {0.0, 0.0} );
  configureController();
  setupHardwareInterfaces();
  activateController();

  rclcpp::Time time(0, 0);
  rclcpp::Duration period(0, 1000000); // 1ms

  setReference(0, 0.0);
  controller_->update_and_write_commands(time, period);
  EXPECT_DOUBLE_EQ(hw_cmd_values_[0], 0.0);

  setReference(0, 1.0);
  
  controller_->update_and_write_commands(time, period);
  EXPECT_NEAR(hw_cmd_values_[0], 0.25, 1e-6);

  for (int i=0; i<49; i++) {
    controller_->update_and_write_commands(time, period);
  }
  EXPECT_NEAR(hw_cmd_values_[0], 0.75, 1e-6);

  for (int i=0; i<50; i++) {
    controller_->update_and_write_commands(time, period);
  }
  EXPECT_NEAR(hw_cmd_values_[0], 1.0, 1e-6);
}

TEST_F(InputShapingControllerTest, EnabledBypassShaping)
{
  // joint1: use shaping, freq=10Hz, zeta=0.0, but enabled=false
  initController( {"zv", "none"}, {10.0, 10.0}, {0.0, 0.0}, false );
  configureController();
  setupHardwareInterfaces();
  activateController();

  rclcpp::Time time(0, 0);
  rclcpp::Duration period(0, 1000000); // 1ms

  setReference(0, 0.0);
  controller_->update_and_write_commands(time, period);
  EXPECT_DOUBLE_EQ(hw_cmd_values_[0], 0.0);

  setReference(0, 1.0);
  
  // Since enabled=false, it should immediately pass 1.0 through
  controller_->update_and_write_commands(time, period);
  EXPECT_DOUBLE_EQ(hw_cmd_values_[0], 1.0);
}
