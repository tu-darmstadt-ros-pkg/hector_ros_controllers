# Hector Ros Controllers

A set of ROS 2 controllers focused on **safety**, **self-collision avoidance**, and **safe position command handling**.
All controllers are compatible with **ros2_control** and can be inserted as **chain controllers**.

---

## 1. Safety Forward Controller

Forwards incoming commands, but **resets the command to zero** if no message is received within a configured timeout.
Useful for teleoperation and autonomy if the communication is unreliable or in case of node failures.

---

## 2. Self Collision Avoidance Controller

Monitors joint states and **prevents the robot from moving into a configuration close to self-collision**.
Acts as a motion gatekeeper in real-time.

---

## 3. Safety Position Controller

A **chainable safety layer** for joint position commands.

### Features

* **Continuous joints unwrapped** to nearest equivalent angle.
* **Joint limits enforced** directly from URDF.
* **Target-only self-collision check** (Pinocchio + hpp-fcl).
* **Block-if-too-far:** prevents large unsafe jumps and makes collision checking meaningful.
* Optional **compliant / stiff current limiting** per joint.

### Collision Checking Limitation - target-only collision checking

If used **behind a trajectory controller**, target updates are **small steps**, so checking only the target pose is sufficient when using a **small collision padding**.

If the parent controller is a raw position controller (large jumps), **block-if-too-far** ensures targets stay close → prevents missing intermediate collisions.

**Future improvement:** continuous / path collision checking.


## Parameters (Safety Position Controller)

| Parameter                          | Type       | Default    | Description                                                                                          |
| ---------------------------------- | ---------- | ---------- | ---------------------------------------------------------------------------------------------------- |
| `joints`                           | `string[]` | `[]`       | Names of joints controlled. Must match URDF. *(read-only)*                                           |
| `unwrap_continuous_joints`         | `bool`     | `true`     | Unwrap continuous joints to maintain continuity in commanded angle.                                  |
| `enforce_position_limits`          | `bool`     | `true`     | Clamp joint commands to URDF limits.                                                                 |
| `check_self_collisions`            | `bool`     | `true`     | Run self-collision check on the **target** pose before sending commands.                             |
| `collision_padding`                | `double`   | `0.0`      | Minimum allowed link-to-link distance [m] — distances ≤ padding count as collision.                  |
| `collision_cache_epsilon`          | `double`   | `0.000001` | Threshold for reusing last collision check result (skip compute if target change is small).          |
| `block_if_too_far`                 | `bool`     | `true`     | Blocks commands that are too far from current state. Auto-enabled if `check_self_collisions = true`. |
| `block_velocity_scaling`           | `double`   | `1.5`      | Scales maximum per-cycle motion: blocked if `                                                        |
| `debug_visualize_collisions`       | `bool`     | `false`    | Publishes collision debug markers for RViz.                                                          |
| `set_current_limits`               | `bool`     | `false`    | Enables per-joint current limit control. *(read-only)*                                               |
| `current_limits.*.compliant_limit` | `double`   | `3.0`      | Current limit in **compliant** mode [A].                                                             |
| `current_limits.*.stiff_limit`     | `double`   | `5.0`      | Current limit in **stiff** mode [A].                                                                 |

> Note: When using dynamic reconfigure to change the parameters the controller must be deactivated and reactivated for changes to take effect!


## Example Configuration

```yaml
/**:
  controller_manager:
    ros__parameters:
      update_rate: 50

      joint_state_broadcaster:
        type: joint_state_broadcaster/JointStateBroadcaster

      arm_trajectory_controller:
        type: joint_trajectory_controller/JointTrajectoryController

      arm_safety_position_controller:
        type: safety_position_controller/SafetyPositionController

  arm_safety_position_controller:
    ros__parameters:
      joints: [arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4, arm_joint_5, arm_joint_6, arm_joint_7]
      unwrap_continuous_joints: true
      enforce_position_limits: true
      check_self_collisions: true
      collision_padding: 0.01
      debug_visualize_collisions: false
      block_velocity_scaling: 3.0
      set_current_limits: true
      current_limits:
        arm_joint_1: {compliant_limit: 3.0, stiff_limit: 9.0}
        arm_joint_2: {compliant_limit: 5.0, stiff_limit: 9.0}
        arm_joint_3: {compliant_limit: 5.0, stiff_limit: 9.0}
        arm_joint_4: {compliant_limit: 3.0, stiff_limit: 4.9}
        arm_joint_5: {compliant_limit: 2.5, stiff_limit: 5.5}
        arm_joint_6: {compliant_limit: 0.5, stiff_limit: 3.5}
        arm_joint_7: {compliant_limit: 0.5, stiff_limit: 3.5}

  arm_trajectory_controller:
    ros__parameters:
      joints: [arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4, arm_joint_5, arm_joint_6, arm_joint_7]
      command_joints:
      - arm_safety_position_controller/arm_joint_1
      - arm_safety_position_controller/arm_joint_2
      - arm_safety_position_controller/arm_joint_3
      - arm_safety_position_controller/arm_joint_4
      - arm_safety_position_controller/arm_joint_5
      - arm_safety_position_controller/arm_joint_6
      - arm_safety_position_controller/arm_joint_7
      command_interfaces: [position]
      state_interfaces: [position, velocity]
```
