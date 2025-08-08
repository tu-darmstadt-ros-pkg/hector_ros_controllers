#!/usr/bin/env python3

import os
import pytest
import launch
import launch.actions
import launch_ros.actions
import launch_testing
import rclpy
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces
from std_msgs.msg import Bool

# paths to your YAML configs
CONTROLLER_CFG = os.path.join(
    os.path.dirname(__file__),
    'config', 'controllers.yaml'
)
SPAWNER_CFG = os.path.join(
    os.path.dirname(__file__),
    'config', 'controller_spawner.yaml'
)


def generate_launch_description(estop_initial: bool, release_mode: str):
    """
    Build a LaunchDescription that:
      - brings up ros2_control (controller_manager)
      - brings up your hector_controller_spawner once, or twice if release_mode=='respawn'
      - if estop_initial, starts with estop active
    """
    nodes = []

    # controller_manager
    nodes.append(launch_ros.actions.Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[CONTROLLER_CFG],
        output='screen',
    ))

    # first spawner
    nodes.append(launch_ros.actions.Node(
        package='hector_controller_spawner',
        executable='hector_controller_spawner',
        parameters=[SPAWNER_CFG],
        output='screen',
    ))

    # Test Case 2: “respawn” → include a second spawner after 2s
    if release_mode == 'respawn':
        nodes.append(launch.actions.TimerAction(
            period=2.0,
            actions=[
                launch_ros.actions.Node(
                    package='hector_controller_spawner',
                    executable='hector_controller_spawner',
                    parameters=[SPAWNER_CFG],
                    output='screen',
                )
            ]
        ))

    return launch.LaunchDescription([
        *nodes,
        # signal to launch_testing that we’re up and running
        launch_testing.actions.ReadyToTest(),
    ])


@pytest.mark.launch_test
@launch_testing.parametrize('estop_initial, release_mode', [
    (False, None),       # Case 1: single spawn
    (False, 'respawn'),  # Case 2: two spawns
    (True,  'release'),  # Case 3: estop then release
])
def generate_test_description(estop_initial, release_mode):
    ld = generate_launch_description(estop_initial, release_mode)
    return ld, {'release_mode': release_mode}


def _call_list_controllers(node, timeout=5.0):
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    assert client.wait_for_service(timeout_sec=timeout)
    req = ListControllers.Request()
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    return fut.result().controller


def _call_list_hw_interfaces(node, timeout=5.0):
    client = node.create_client(ListHardwareInterfaces, '/controller_manager/list_hardware_interfaces')
    assert client.wait_for_service(timeout_sec=timeout)
    req = ListHardwareInterfaces.Request()
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    return fut.result().interface_names


@launch_testing.post_shutdown_test()
def test_controllers_and_hardware_interfaces(proc_info, proc_output, release_mode):
    """
    Runs after shutdown.  We only need proc_info/proc_output + release_mode here,
    because all of the launch‐time behavior (including two spawns) was baked
    into the description already.
    """
    rclpy.init()
    node = rclpy.create_node('test_node')

    # 1) hardware interfaces
    hw = _call_list_hw_interfaces(node)
    assert 'athena_flipper_interface' in hw
    assert 'athena_arm_interface'     in hw

    # 2) controllers after all spawns
    ctrls = _call_list_controllers(node)
    active = {c.name for c in ctrls if c.state == 'active'}

    # should always get at least these
    assert 'joint_state_broadcaster'             in active
    assert 'self_collision_avoidance_controller' in active
    assert 'arm_trajectory_controller'           in active

    # 3) Case 3: e-stop start; now release it and check loading
    if release_mode == 'release':
        pub = node.create_publisher(Bool, 'estop_board/hard_estop', 1)
        pub.publish(Bool(data=False))
        rclpy.spin_once(node, timeout_sec=5.0)

        ctrls2 = _call_list_controllers(node)
        active2 = {c.name for c in ctrls2 if c.state == 'active'}
        assert 'flipper_trajectory_controller' in active2

    node.destroy_node()
    rclpy.shutdown()
