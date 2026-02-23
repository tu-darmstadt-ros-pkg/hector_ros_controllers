#!/usr/bin/env python3
"""Launch athena with DynamicsMockHardware, controllers, gamepad, and RViz."""

import os
import socket

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = get_package_share_directory("dynamics_mock_hardware")

    # Read URDF from file
    urdf_path = os.path.join(pkg_dir, "config", "athena.urdf")
    with open(urdf_path, "r") as f:
        robot_description_content = f.read()

    # --- Arguments ---
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz")
    )
    ld.add_action(
        DeclareLaunchArgument(
            "gamepad", default_value="true", description="Launch gamepad manager"
        )
    )

    robot_ns = "athena"

    # --- Robot State Publisher ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_ns,
        output="screen",
        parameters=[{"robot_description": robot_description_content}],
    )
    ld.add_action(robot_state_publisher)

    # --- Controller Manager ---
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        namespace=robot_ns,
        output="screen",
        parameters=[
            os.path.join(pkg_dir, "config", "controllers.yaml"),
            {
                "hardware_components_initial_state.unconfigured": [
                    "athena_flipper_interface",
                    "athena_arm_interface",
                ],
            },
        ],
    )
    ld.add_action(controller_manager)

    # --- Controller Spawner ---
    controller_spawner = Node(
        package="hector_controller_spawner",
        executable="hector_controller_spawner",
        name="multi_controller_spawner",
        namespace=robot_ns,
        output="screen",
        parameters=[
            os.path.join(pkg_dir, "config", "controller_spawner.yaml"),
        ],
    )
    ld.add_action(controller_spawner)

    # --- Joy Node + Gamepad Manager ---
    hostname = socket.gethostname()
    hostname_sanitized = hostname.replace("-", "_")
    if not hostname_sanitized[0].isalpha():
        hostname_sanitized = "ns" + hostname_sanitized

    joy_node = GroupAction(
        actions=[
            PushRosNamespace(hostname_sanitized),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[
                    {"device_name": ""},
                    {"deadzone": 0.1},
                    {"autorepeat_rate": 30.0},
                ],
            ),
        ],
    )
    ld.add_action(joy_node)

    gamepad_manager = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("hector_gamepad_manager"),
                    "launch",
                    "hector_gamepad_manager.launch.yaml",
                ]
            )
        ),
        launch_arguments={
            "config_name": "athena",
            "robot_namespace": robot_ns,
            "ocs_namespace": hostname_sanitized,
        }.items(),
    )
    ld.add_action(gamepad_manager)

    # --- RViz ---
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(pkg_dir, "rviz", "simulation.rviz"),
        ],
    )
    ld.add_action(rviz_node)

    return ld
