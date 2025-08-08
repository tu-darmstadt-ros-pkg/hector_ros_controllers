import os
import unittest
from ament_index_python.packages import get_package_share_directory
import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.actions

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents


def generate_test_description():
    pkg_share = get_package_share_directory("hector_controller_spawner")

    controller_config = os.path.join(pkg_share, "test", "config", "controllers.yaml")
    spawner_config = os.path.join(
        pkg_share, "test", "config", "controller_spawner.yaml"
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
                launch.actions.TimerAction(period=5.0, actions=[spawner_node]),
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
        client = self.node.create_client(
            ListHardwareComponents, "/controller_manager/list_hardware_components"
        )

        self.assertTrue(client.wait_for_service(timeout_sec=10.0))

        request = ListHardwareComponents.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)

        response = future.result()
        self.assertIsNotNone(response)

        expected_interfaces = ["athena_flipper_interface", "athena_arm_interface"]
        loaded_interfaces = [iface.name for iface in response.component]

        self.assertGreater(len(loaded_interfaces), 0, "No hardware interfaces loaded")
        for iface in expected_interfaces:
            self.assertIn(iface, loaded_interfaces)

    def test_controllers_loaded_and_activated(self):
        client = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )

        self.assertTrue(client.wait_for_service(timeout_sec=10.0))

        request = ListControllers.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)

        response = future.result()
        self.assertIsNotNone(response)

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
        # You can disable this assert if controller_manager fails to shutdown cleanly
        self.assertEqual(proc_info["hector_controller_spawner"].returncode, 0)
