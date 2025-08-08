import os
import unittest
import time
from ament_index_python.packages import get_package_share_directory
import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.actions
from lifecycle_msgs.msg import State
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from controller_manager_msgs.srv import (
    ListControllers,
    ListHardwareComponents,
    SwitchController,
    SetHardwareComponentState,
)
from rclpy.qos import QoSProfile, DurabilityPolicy


def generate_test_description():
    pkg_share = get_package_share_directory("hector_controller_spawner")

    controller_config = os.path.join(pkg_share, "test", "config", "controllers.yaml")
    spawner_config = os.path.join(
        pkg_share, "test", "config", "controller_spawner_with_estop.yaml"
    )
    robot_description_file = os.path.join(pkg_share, "test", "config", "athena.urdf")

    for path in [controller_config, spawner_config, robot_description_file]:
        if not os.path.isfile(path):
            raise FileNotFoundError(f"Missing test file: {path}")

    with open(robot_description_file, "r") as f:
        robot_description = f.read()

    robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[controller_config],
    )

    # Launch the spawner alongside the controller manager
    spawner_node = launch_ros.actions.Node(
        package="hector_controller_spawner",
        executable="hector_controller_spawner",
        output="screen",
        parameters=[spawner_config],
    )

    return (
        launch.LaunchDescription(
            [
                robot_state_publisher,
                controller_manager,
                launch.actions.TimerAction(period=3.0, actions=[spawner_node]),
                launch.actions.TimerAction(
                    period=5.0, actions=[launch_testing.actions.ReadyToTest()]
                ),
            ]
        ),
        {"controller_manager": controller_manager, "spawner_node": spawner_node},
    )


class TestEStopFunctionality(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_estop_functionality")

        # Create e-stop publisher
        estop_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Match spawner's QoS
        )
        cls.estop_pub = cls.node.create_publisher(
            Bool, "estop_board/hard_estop", estop_qos
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_complete_estop_workflow(self):
        """Test complete e-stop workflow: block loading, allow loading, block again, allow again"""

        # Step 1: Send e-stop true (active/stopped)
        self.node.get_logger().info("Step 1: Activating e-stop")
        self._publish_estop(True)
        time.sleep(2.0)  # Allow time for e-stop to be processed

        # Step 2: Check that hardware interfaces and controllers are not active
        self.node.get_logger().info(
            "Step 2: Checking that hardware/controllers are not active"
        )
        hardware_components = self._get_hardware_components()
        controllers = self._get_controller_list()

        # Verify no hardware components are active
        active_hardware = [
            hw.name for hw in hardware_components if hw.state.label == "active"
        ]
        self.assertEqual(
            len(active_hardware),
            0,
            "No hardware interfaces should be active while e-stop is active",
        )

        # Verify no controllers are active
        active_controllers = [c.name for c in controllers if c.state == "active"]
        self.assertEqual(
            len(active_controllers),
            0,
            "No controllers should be active while e-stop is active",
        )

        # Step 3: Send e-stop false (inactive/running) and wait
        self.node.get_logger().info("Step 3: Releasing e-stop")
        self._publish_estop(False)
        time.sleep(8.0)  # Allow time for spawner to proceed and load everything

        # Step 4: Check that controllers and hardware interfaces are loaded and active
        self.node.get_logger().info(
            "Step 4: Checking that hardware/controllers are active"
        )
        hardware_components = self._get_hardware_components()
        controllers = self._get_controller_list()

        # Verify expected hardware components are active
        expected_hardware = ["athena_flipper_interface", "athena_arm_interface"]
        active_hardware = [
            hw.name for hw in hardware_components if hw.state.label == "active"
        ]

        for hw_name in expected_hardware:
            self.assertIn(
                hw_name,
                active_hardware,
                f"Hardware interface {hw_name} should be active after e-stop release",
            )

        # Verify expected controllers are active
        expected_active_controllers = [
            "joint_state_broadcaster",
            "flipper_velocity_controller",
            "gripper_trajectory_controller",
            "arm_trajectory_controller",
            "vel_to_pos_controller",
        ]

        active_controllers = [c.name for c in controllers if c.state == "active"]
        for controller in expected_active_controllers:
            self.assertIn(
                controller,
                active_controllers,
                f"Controller {controller} should be active after e-stop release",
            )

        # Step 5: Send e-stop true and unload hardware interfaces
        self.node.get_logger().info(
            "Step 5: Activating e-stop again (should deactivate hardware)"
        )
        self._publish_estop(True)
        self._deactivate_all_controllers()
        self._unload_hardware_interfaces()
        time.sleep(3.0)  # Allow time for deactivation

        # Step 6: Verify hardware interfaces are deactivated
        hardware_components = self._get_hardware_components()
        controllers = self._get_controller_list()

        # Check that hardware is no longer active
        active_hardware_after_estop = [
            hw.name for hw in hardware_components if hw.state.label == "active"
        ]
        self.assertEqual(
            len(active_hardware_after_estop),
            0,
            "Hardware interfaces should be deactivated when e-stop is activated again",
        )

        # Check that controllers are no longer active
        active_controllers_after_estop = [
            c.name for c in controllers if c.state == "active"
        ]
        self.assertEqual(
            len(active_controllers_after_estop),
            0,
            "Controllers should be deactivated when e-stop is activated again",
        )

        # Step 7: Send e-stop false and wait some time
        self.node.get_logger().info("Step 6: Releasing e-stop second time")
        self._publish_estop(False)
        time.sleep(8.0)  # Allow time for reactivation

        # Step 8: Check that hardware interfaces and requested controllers are active again
        self.node.get_logger().info(
            "Step 7: Checking final state - should be active again"
        )
        final_hardware_components = self._get_hardware_components()
        final_controllers = self._get_controller_list()

        # Verify hardware interfaces are active again
        final_active_hardware = [
            hw.name for hw in final_hardware_components if hw.state.label == "active"
        ]
        for hw_name in expected_hardware:
            self.assertIn(
                hw_name,
                final_active_hardware,
                f"Hardware interface {hw_name} should be active again after second e-stop release",
            )

        # Verify controllers are active again
        final_active_controllers = [
            c.name for c in final_controllers if c.state == "active"
        ]
        for controller in expected_active_controllers:
            self.assertIn(
                controller,
                final_active_controllers,
                f"Controller {controller} should be active again after second e-stop release",
            )

        self.node.get_logger().info("E-stop workflow test completed successfully")

    def _publish_estop(self, active):
        """Helper to publish e-stop message multiple times for reliability"""
        estop_msg = Bool()
        estop_msg.data = active

        # Publish multiple times to ensure delivery
        for _ in range(10):
            self.estop_pub.publish(estop_msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)

        # Extra time for message propagation
        time.sleep(0.5)

    def _get_hardware_components(self):
        """Helper to get current hardware component list"""
        client = self.node.create_client(
            ListHardwareComponents, "/controller_manager/list_hardware_components"
        )

        if not client.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().warn("Hardware components service not available")
            return []

        request = ListHardwareComponents.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

        response = future.result()
        return response.component if response else []

    def _get_controller_list(self):
        """Helper to get current controller list"""
        client = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )

        if not client.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().warn("Controller manager service not available")
            return []

        request = ListControllers.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

        response = future.result()
        return response.controller if response else []

    def _unload_hardware_interfaces(self):
        """Unload/deactivate hardware interfaces by transitioning them to inactive state"""
        active_hardware = ["athena_flipper_interface", "athena_arm_interface"]
        # Create service client for setting hardware component state
        set_hw_state_client = self.node.create_client(
            SetHardwareComponentState,
            "/controller_manager/set_hardware_component_state",
        )

        if not set_hw_state_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error(
                "Hardware component state service not available"
            )
            return

        # Deactivate each active hardware interface
        for hw_component in active_hardware:
            self.node.get_logger().info(
                f"Deactivating hardware interface: {hw_component}"
            )

            request = SetHardwareComponentState.Request()
            request.name = hw_component
            request.target_state = State()
            request.target_state.id = (
                State.PRIMARY_STATE_INACTIVE
            )  # Transition to inactive
            request.target_state.label = "inactive"

            # Call the service
            future = set_hw_state_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

            response = future.result()
            if response and response.ok:
                self.node.get_logger().info(
                    f"Successfully deactivated hardware interface: {hw_component}"
                )
            else:
                self.node.get_logger().error(
                    f"Failed to deactivate hardware interface: {hw_component}"
                )

    def _deactivate_all_controllers(self):
        """Deactivate all controllers by switching them off"""

        controllers_to_deactivate = [
            "joint_state_broadcaster",
            "flipper_trajectory_controller",
            "flipper_velocity_controller",
            "gripper_trajectory_controller",
            "arm_trajectory_controller",
            "vel_to_pos_controller",
        ]

        # Create service client for switching controllers
        switch_client = self.node.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

        if not switch_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error("Switch controller service not available")
            return

        self.node.get_logger().info("Deactivating all controllers")

        request = SwitchController.Request()
        request.activate_controllers = []  # No controllers to activate
        request.deactivate_controllers = (
            controllers_to_deactivate  # Controllers to deactivate
        )
        request.strictness = (
            SwitchController.Request.BEST_EFFORT
        )  # Strict mode - fail if any controller can't be switched
        request.activate_asap = False
        request.timeout = rclpy.duration.Duration(seconds=5.0).to_msg()

        # Call the service
        future = switch_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)

        response = future.result()
        if response and response.ok:
            self.node.get_logger().info("Successfully deactivated all controllers")
        else:
            self.node.get_logger().error("Failed to deactivate controllers")


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        # Check that the spawner completed successfully
        self.assertEqual(proc_info["hector_controller_spawner"].returncode, 0)
