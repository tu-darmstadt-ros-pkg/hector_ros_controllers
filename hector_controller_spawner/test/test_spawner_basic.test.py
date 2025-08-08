import os
import time
import unittest
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros
import launch_testing.actions
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from std_msgs.msg import Bool
import pytest


def generate_test_description():
    # Get package directory
    pkg_share = get_package_share_directory("hector_controller_spawner")

    # Controller manager configuration
    controller_config = os.path.join(pkg_share, "test", "config", "controllers.yaml")
    spawner_config = os.path.join(
        pkg_share, "test", "config", "controller_spawner.yaml"
    )
    robot_description = os.path.join(pkg_share, "test", "config", "athena.urdf")
    if not os.path.isfile(controller_config):
        raise FileNotFoundError(
            f"Controller config file not found: {controller_config}"
        )
    if not os.path.isfile(spawner_config):
        raise FileNotFoundError(f"Spawner config file not found: {spawner_config}")
    if not os.path.isfile(robot_description):
        raise FileNotFoundError(
            f"Robot description file not found: {robot_description}"
        )

    with open(robot_description, "r") as f:
        robot_description = f.read()
    # Launch controller manager
    controller_manager = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        output="screen",
    )

    # Launch spawner after delay
    spawner_node = launch_ros.actions.Node(
        package="hector_controller_spawner",
        executable="hector_controller_spawner",
        parameters=[spawner_config],
        output="screen",
    )

    robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
            }
        ],
    )

    return (
        launch.LaunchDescription(
            [
                robot_state_publisher,
                controller_manager,
                # Add delay to ensure controller manager is ready
                launch.actions.TimerAction(period=5.0, actions=[spawner_node]),
                # Start tests after another delay
                launch.actions.TimerAction(
                    period=10.0, actions=[launch_testing.actions.ReadyToTest()]
                ),
            ]
        ),
        {"controller_manager": controller_manager, "spawner_node": spawner_node},
    )


class TestControllerSpawner(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_controller_spawner")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_hardware_interfaces_loaded(self):
        """Test that expected hardware interfaces are loaded"""
        client = self.node.create_client(
            ListHardwareComponents, "/controller_manager/list_hardware_components"
        )

        # Wait for service
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))

        # Call service
        request = ListHardwareComponents.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

        response = future.result()
        self.assertIsNotNone(response)

        # Verify expected hardware interfaces
        expected_interfaces = ["athena_flipper_interface", "athena_arm_interface"]
        loaded_interfaces = [iface.name for iface in response.component]
        self.assertGreater(len(loaded_interfaces), 0, "No hardware interfaces loaded")

        for interface in expected_interfaces:
            self.assertIn(interface, loaded_interfaces)

    def test_controllers_loaded_and_activated(self):
        """Test that expected controllers are loaded and activated"""
        client = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )

        self.assertTrue(client.wait_for_service(timeout_sec=10.0))

        request = ListControllers.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

        response = future.result()
        self.assertIsNotNone(response)

        # Check expected controllers and their states
        expected_active = [
            "joint_state_broadcaster",
            "flipper_velocity_controller",
            "gripper_trajectory_controller",
            "arm_trajectory_controller",
            "vel_to_pos_controller",
        ]

        expected_inactive = ["flipper_trajectory_controller"]

        loaded_controllers = {ctrl.name: ctrl.state for ctrl in response.controller}
        self.node.get_logger().info(f"Loaded controllers: {loaded_controllers}")

        for controller in expected_active:
            self.assertIn(controller, loaded_controllers)
            self.assertEqual(loaded_controllers[controller], "active")

        for controller in expected_inactive:
            self.assertIn(controller, loaded_controllers)
            self.assertEqual(loaded_controllers[controller], "inactive")


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        """Test that all processes exit cleanly"""
        launch_testing.asserts.assertExitCodes(proc_info)
