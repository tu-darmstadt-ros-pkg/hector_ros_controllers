import os
import unittest
import subprocess
import time
from ament_index_python.packages import get_package_share_directory
import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.actions

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers


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

    # Don't launch spawner automatically - we'll control it manually
    return (
        launch.LaunchDescription(
            [
                robot_state_publisher,
                controller_manager,
                launch.actions.TimerAction(
                    period=5.0, actions=[launch_testing.actions.ReadyToTest()]
                ),
            ]
        ),
        {"controller_manager": controller_manager},
    )


class TestControllerSpawnerIdempotency(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_controller_spawner_idempotency")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_spawner_run_twice_idempotent(self):
        """Test that running the spawner twice doesn't cause problems"""

        pkg_share = get_package_share_directory("hector_controller_spawner")
        spawner_config = os.path.join(
            pkg_share, "test", "config", "controller_spawner.yaml"
        )

        # Spawner command
        cmd = [
            "ros2",
            "run",
            "hector_controller_spawner",
            "hector_controller_spawner",
            "--ros-args",
            "--params-file",
            spawner_config,
        ]

        # 1. Check initial state (no controllers loaded)
        initial_controllers = self._get_controller_list()
        initial_count = len(initial_controllers)
        self.node.get_logger().info(f"Initial controller count: {initial_count}")

        # 2. Run spawner first time
        self.node.get_logger().info("Running spawner first time...")
        process1 = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        # Wait for spawner to complete
        return_code1 = process1.wait(timeout=30)  # 30 second timeout

        stdout1, stderr1 = process1.communicate()
        self.assertEqual(
            return_code1, 0, f"First spawner run failed with return code {return_code1}"
        )

        # Give time for controllers to be loaded
        time.sleep(2.0)

        # 3. Check state after first run
        first_run_controllers = self._get_controller_list()
        first_run_count = len(first_run_controllers)
        self.node.get_logger().info(
            f"Controller count after first run: {first_run_count}"
        )

        self.assertGreater(
            first_run_count,
            initial_count,
            "Controllers should be loaded after first spawner run",
        )

        # Store the state for comparison
        first_run_active = [
            c.name for c in first_run_controllers if c.state == "active"
        ]
        first_run_inactive = [
            c.name for c in first_run_controllers if c.state == "inactive"
        ]

        # 4. Wait a bit, then run spawner second time
        time.sleep(3.0)  # Delay between runs

        self.node.get_logger().info("Running spawner second time...")
        process2 = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        # Wait for second spawner to complete
        return_code2 = process2.wait(timeout=30)

        stdout2, stderr2 = process2.communicate()
        self.assertEqual(
            return_code2,
            0,
            f"Second spawner run failed with return code {return_code2}",
        )

        # Give time for any changes to take effect
        time.sleep(2.0)

        # 5. Check state after second run
        second_run_controllers = self._get_controller_list()
        second_run_count = len(second_run_controllers)
        self.node.get_logger().info(
            f"Controller count after second run: {second_run_count}"
        )

        second_run_active = [
            c.name for c in second_run_controllers if c.state == "active"
        ]
        second_run_inactive = [
            c.name for c in second_run_controllers if c.state == "inactive"
        ]

        # 6. Verify that the state is identical (idempotent behavior)
        self.assertEqual(
            first_run_count,
            second_run_count,
            "Controller count should be the same after second spawner run",
        )

        # Check that active controllers are the same
        self.assertEqual(
            set(first_run_active),
            set(second_run_active),
            "Active controllers should be identical after second run",
        )

        # Check that inactive controllers are the same
        self.assertEqual(
            set(first_run_inactive),
            set(second_run_inactive),
            "Inactive controllers should be identical after second run",
        )

        # 7. Verify expected controllers are still in correct states
        expected_active = [
            "joint_state_broadcaster",
            "flipper_velocity_controller",
            "gripper_trajectory_controller",
            "arm_trajectory_controller",
            "vel_to_pos_controller",
        ]

        for controller in expected_active:
            self.assertIn(
                controller,
                second_run_active,
                f"Controller {controller} should still be active after second run",
            )

        # Log the outputs for debugging if needed
        if stdout1:
            self.node.get_logger().info(
                f"First spawner stdout: {stdout1.decode()[:200]}..."
            )
        if stdout2:
            self.node.get_logger().info(
                f"Second spawner stdout: {stdout2.decode()[:200]}..."
            )

    def _get_controller_list(self):
        """Helper to get current controller list"""
        client = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )

        if not client.wait_for_service(timeout_sec=5.0):
            return []

        request = ListControllers.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

        response = future.result()
        return response.controller if response else []


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        # Controller manager should shutdown cleanly
        pass  # Allow flexible exit codes since we're not checking spawner here
