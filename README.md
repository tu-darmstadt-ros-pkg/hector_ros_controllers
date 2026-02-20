# Hector Ros Controllers

A set of ROS 2 controllers focused on **safety**, **self-collision avoidance**, and **safe position command handling**.
All controllers are compatible with **ros2_control** and can be inserted as **chain controllers**.

---

## 1. Safety Forward Controller

The **Safety Forward Controller** is a chainable ROS 2 controller that forwards joint commands while enforcing three
safety mechanisms:

* **Safety timeout:** If no commands arrive for a specified duration (standalone mode only), outputs are set to zero.
* **Emergency Stop (E-Stop):** Immediately stops motion; in position mode the controller holds the current joint
  positions.
* **Joint Limit Enforcement** Makes sure the command value is within the allowed limits. Warning only checks the limit
  of the used interface (e.g. in velocity mode are no position limits checked).
  It supports *position*, *velocity*, and *effort* command interfaces and can operate in:

* **Standalone mode** — commands come from `~/commands`
* **Chained mode** — commands come from an upstream controller via reference interfaces

### Behavior Summary

| Condition / Mode               | Position Interface          | Velocity Interface     | Effort Interface       |
|--------------------------------|-----------------------------|------------------------|------------------------|
| Normal operation               | Forward command             | Forward command        | Forward command        |
| Safety timeout *(not chained)* | (optional) or no-op         | 0.0                    | 0.0                    |
| E-Stop engaged                 | Hold last measured position | 0.0                    | 0.0                    |
| E-Stop released                | Resume forwarding           | Resume forwarding      | Resume forwarding      |
| Chained mode                   | Use upstream reference      | Use upstream reference | Use upstream reference |

**Important:**
The **safety timer is disabled in chained mode** and is only active when the controller receives commands via
`~/commands`.

### Parameters

| Name                       | Type           | Default | Description                                                                                                                       |
|----------------------------|----------------|---------|-----------------------------------------------------------------------------------------------------------------------------------|
| **joints**                 | `string_array` | `[]`    | Names of the joints controlled by this controller. Order defines command order.                                                   |
| **interface_type**         | `string`       | `""`    | Interface to command: `"position"`, `"velocity"`, or `"effort"`.                                                                  |
| **safety_timer_duration**  | `int`          | `500`   | Timeout in milliseconds. If no commands are received for this duration, the controller outputs zero (not active in chained mode). |
| **passthrough_controller** | `string`       | `""`    | Optional prefix of a lower-level controller whose namespace should be mirrored for hardware interfaces.                           |

### Topics

| Topic            | Type                | Description                                                |
|------------------|---------------------|------------------------------------------------------------|
| `~/commands`     | `Float64MultiArray` | Input commands (not used in chained mode).                 |
| `~/safety_estop` | `Bool`              | Engages (`true`) or releases (`false`) the emergency stop. |

---

## 2. Self Collision Avoidance Controller

Monitors joint states and **prevents the robot from moving into a configuration close to self-collision**.
Acts as a motion gatekeeper in real-time.

---

## 3. Velocity-to-Position Command Controller

Converts **velocity references** into **position commands** for joint hardware. Useful when an upstream controller outputs velocity commands but the hardware only accepts position commands.

### Features

* **Feedforward integration with PD velocity tracking:**
    * Integrates velocity commands into a desired position trajectory.
    * A proportional term corrects for velocity tracking error (commanded vs actual).
    * A derivative term damps acceleration to prevent overshoot and oscillation.
* **Desired position tracking:**
    * Maintains an internal `desired_positions` trajectory, providing implicit position error correction.
    * On state transitions (STOPPED/STOPPING → MOVING), the desired position re-syncs to the actual joint position to prevent jumps.
* **State machine** (per joint):
    * **MOVING** – actively integrating velocity commands.
    * **STOPPING** – velocity command is zero but joint is still moving.
    * **STOPPED** – joint is below velocity threshold; holds the last desired position.
* **Joint synchronization:**
    * Joints in the same synchronization group are kept aligned via P-control on position offsets.
* **E-Stop support:**
    * Subscribes to an E-Stop topic; freezes all joints at their current positions when engaged.
* **Chainable controller:**
    * Can receive velocity references from an upstream controller or from a `~/commands` topic.
* **Dynamic PID reconfiguration:**
    * `kp`, `kd`, and `kp_sync` can be changed at runtime via ROS parameter callbacks.

### Control Law

```
desired_pos[i] += vel_cmd * dt                          // feedforward integration
vel_p = kp * (vel_cmd - vel_actual) * dt                // velocity P-term
vel_d = kd * vel_actual * dt                            // velocity D-term (damping)
pos_cmd = desired_pos[i] + vel_p - vel_d + sync_correction
```

### Parameters

| Name                            | Type           | Default                    | Description                                                                    |
|---------------------------------|----------------|----------------------------|--------------------------------------------------------------------------------|
| **joints**                      | `string_array` | `[]`                       | Names of the joints to control.                                                |
| **kp**                          | `double`       | `2.0`                      | Proportional gain for velocity tracking.                                       |
| **kd**                          | `double`       | `0.1`                      | Derivative gain for acceleration damping.                                      |
| **kp_sync**                     | `double`       | `1.0`                      | Proportional gain for synchronization of synced joints.                        |
| **stopping_velocity_threshold** | `double`       | `0.005`                    | Velocity threshold below which a joint is considered stopped.                  |
| **synchronous_groups**          | `string_array` | `[]`                       | Group names per joint for synchronization (empty = no sync).                   |
| **passthrough_controller**      | `string`       | `""`                       | Prefix for the lower-level controller exposing command interfaces.             |
| **e_stop_topic**                | `string`       | `estop_board/hard_estop`   | Topic for emergency stop messages (`std_msgs/Bool`).                           |

### Topics

| Topic          | Type                | Description                                            |
|----------------|---------------------|--------------------------------------------------------|
| `~/commands`   | `Float64MultiArray` | Velocity commands (not used in chained mode).          |
| E-Stop topic   | `Bool`              | Engages (`true`) or releases (`false`) the e-stop.     |

---

## 4. Safety Position Controller

A **safety layer for joint position commands**, usable both

* as a **chainable controller** behind e.g. a trajectory controller, or
* in **standalone mode** as a position controller receiving commands on `~/commands`.

### Features

* **Chainable or standalone**:

    * *Chained mode*: consumes reference interfaces from an upstream controller.
    * *Non-chained mode*: acts as a position controller driven by `Float64MultiArray` on `~/commands`.
* **Continuous joint handling**:

    * Optionally unwraps continuous joints to the nearest equivalent angle.
* **Joint limit enforcement**:

    * Clamps commands to URDF limits (rev/prismatic/other) if enabled.
* **Target-only self-collision check**:

    * Uses URDF + SRDF via `CollisionChecker`.
    * Only the *target pose* is checked; large jumps are prevented via `block_if_too_far`.
* **Block-if-too-far**:

    * Limits per-cycle motion based on URDF velocity limits and controller update rate.
* **E-Stop**:

    * Subscribes to `~/safety_estop` (`std_msgs/Bool`).
    * On activation → records current positions and *holds them* (no limit/collision checks) until E-Stop is released.
* **Safety Bypass Mode** (for folded arm positions):

    * Service `~/bypass_safety_checks` (`std_srvs/SetBool`) to temporarily disable collision checks and relax joint limits.
    * Useful when driving the arm into folded positions where intentional collisions must be made.
    * Adds configurable tolerance to joint limits (default 3%).
    * Auto-disables after configurable timeout (default 60 seconds) for safety.
    * **Note:** Joint wrapping for continuous joints remains **always active** even during bypass.
* **Optional current-limit control**:

    * Per-joint compliant/stiff current limits, written to `<joint>/current`.
* **Debug joint state publishers**:

    * If enabled, publishes incoming command references and outgoing commanded positions as `sensor_msgs/JointState`.


### Parameters (Safety Position Controller)

| Parameter                          | Type       | Default | Description                                                                                                         |
| ---------------------------------- | ---------- | ------- | ------------------------------------------------------------------------------------------------------------------- |
| `joints`                           | `string[]` | `[]`    | Names of joints controlled. Must match URDF. *(effectively read-only; set via params at startup)*                   |
| `unwrap_continuous_joints`         | `bool`     | `true`  | If `true`, continuous joints are unwrapped to stay close to the current angle.                                      |
| `enforce_position_limits`          | `bool`     | `true`  | If `true`, clamps joint commands to URDF position limits.                                                           |
| `check_self_collisions`            | `bool`     | `true`  | If `true`, performs a self-collision check on the **target** pose before sending commands.                          |
| `collision_padding`                | `double`   | `0.0`   | Minimum allowed link-to-link distance [m]; distances ≤ padding are treated as collision.                            |
| `collision_cache_epsilon`          | `double`   | `1e-6`  | Threshold for reusing the previous collision result (skip recomputation if the pose change is below this value).    |
| `block_if_too_far`                 | `bool`     | `true`  | If `true`, blocks/limits commands that are too far from the current state. Auto-enabled if `check_self_collisions`. |
| `block_velocity_scaling`           | `double`   | `1.5`   | Scales maximum per-cycle motion: allowed step ≈ `velocity_limit / update_rate * block_velocity_scaling`.            |
| `debug_visualize_collisions`       | `bool`     | `false` | If `true`, publishes collision debug markers for RViz (via `CollisionChecker`).                                     |
| `set_current_limits`               | `bool`     | `false` | If `true`, enables writing `<joint>/current` limits for compliant/stiff modes. *(configured at startup)*            |
| `current_limits.*.compliant_limit` | `double`   | `3.0`   | Per-joint current limit in **compliant** mode [A].                                                                  |
| `current_limits.*.stiff_limit`     | `double`   | `5.0`   | Per-joint current limit in **stiff** mode [A].                                                                      |
| `publish_debug_joint_states`       | `bool`     | `false` | If `true`, publishes debug `JointState` messages for incoming references and outgoing commands.                     |
| `safety_bypass_timeout`            | `double`   | `60.0`  | Time in seconds after which safety bypass auto-disables. Must be positive.                                          |
| `safety_bypass_joint_limit_tolerance` | `double` | `0.03`  | Tolerance factor (0.0-1.0) added to joint limits during bypass. E.g., 0.03 = 3% beyond normal limits.               |

> **Note:** Parameters are read/updated at `on_activate()`. To apply runtime changes reliably, deactivate and reactivate the controller.

---

### Topics & Services

**Subscriptions**

| Topic                        | Type                                | Used for                                                                     |
| ---------------------------- | ----------------------------------- | ---------------------------------------------------------------------------- |
| `~/commands`                 | `std_msgs/Float64MultiArray`        | **Non-chained mode**: incoming position commands (one per joint).            |
| `~/safety_estop`             | `std_msgs/Bool`                     | E-Stop control. `true` → latch and hold current positions; `false` → resume. |
| `robot_description_semantic` | `std_msgs/String` (transient local) | SRDF XML for self-collision checking.                                        |

**Publications (debug, optional)**

Enabled if `publish_debug_joint_states = true`:

| Topic                      | Type                     | Content                                                                |
| -------------------------- | ------------------------ | ---------------------------------------------------------------------- |
| `~/debug_in_joint_states`  | `sensor_msgs/JointState` | Names = `joints`, positions = current **references** (input commands). |
| `~/debug_out_joint_states` | `sensor_msgs/JointState` | Names = `joints`, positions = final **commanded** joint positions.     |

**Services**

| Service                    | Type               | Description                                                                                          |
| -------------------------- | ------------------ | ---------------------------------------------------------------------------------------------------- |
| `~/enforce_current_limits` | `std_srvs/SetBool` | Enable (`true`) or disable (`false`) compliant mode (switch between compliant/stiff current limits). |
| `~/bypass_safety_checks`   | `std_srvs/SetBool` | Enable (`true`) or disable (`false`) safety bypass mode. Disables collision checks and relaxes joint limits. Auto-disables after `safety_bypass_timeout` seconds. Joint wrapping remains active. |


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
      joints: [ arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4, arm_joint_5, arm_joint_6, arm_joint_7 ]
      unwrap_continuous_joints: true
      enforce_position_limits: true
      check_self_collisions: true
      collision_padding: 0.01
      debug_visualize_collisions: false
      block_velocity_scaling: 3.0
      set_current_limits: true
      current_limits:
        arm_joint_1: { compliant_limit: 3.0, stiff_limit: 9.0 }
        arm_joint_2: { compliant_limit: 5.0, stiff_limit: 9.0 }
        arm_joint_3: { compliant_limit: 5.0, stiff_limit: 9.0 }
        arm_joint_4: { compliant_limit: 3.0, stiff_limit: 4.9 }
        arm_joint_5: { compliant_limit: 2.5, stiff_limit: 5.5 }
        arm_joint_6: { compliant_limit: 0.5, stiff_limit: 3.5 }
        arm_joint_7: { compliant_limit: 0.5, stiff_limit: 3.5 }

  arm_trajectory_controller:
    ros__parameters:
      joints: [ arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4, arm_joint_5, arm_joint_6, arm_joint_7 ]
      command_joints:
        - arm_safety_position_controller/arm_joint_1
        - arm_safety_position_controller/arm_joint_2
        - arm_safety_position_controller/arm_joint_3
        - arm_safety_position_controller/arm_joint_4
        - arm_safety_position_controller/arm_joint_5
        - arm_safety_position_controller/arm_joint_6
        - arm_safety_position_controller/arm_joint_7
      command_interfaces: [ position ]
      state_interfaces: [ position, velocity ]
```
