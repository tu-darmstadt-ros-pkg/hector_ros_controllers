# Hector Ros Controllers

A set of ROS 2 controllers focused on **safety**, **self-collision avoidance**, and **safe position command handling**.
All controllers are compatible with **ros2_control** and can be inserted as **chain controllers**.

---

## 1. Safety Position Controller

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
* **Self-collision check with distance-based velocity scaling**:

    * Uses URDF + SRDF via `CollisionChecker` (Pinocchio + hpp-fcl).
    * Returns minimum pairwise clearance distance, not just a binary collision flag.
    * When `check_self_collisions` is enabled, per-cycle joint motion is automatically limited based on URDF velocity limits (`block_velocity_scaling`) and **scaled down proportionally** as the robot approaches collision geometry.
    * **Safety zone**: Between `collision_padding` and `collision_safety_zone`, velocity is linearly interpolated from 0% to 100%. Beyond `collision_safety_zone`, full velocity is allowed. At or below `collision_padding`, motion is blocked entirely.
    * **Directional scaling** (`directional_collision_scaling`): When enabled, the controller computes collision distance gradients w.r.t. joint velocities for all pairs within the safety zone. Only motions that *decrease* a collision distance are slowed down — motions moving away from all collisions proceed at full speed. This prevents the robot from becoming trapped near collisions. The check is conservative: if *any* safety-zone pair's distance would decrease, the full distance-based scaling is applied.
    * This replaces the former `block_if_too_far` parameter with a mandatory, distance-aware velocity limiting system.
* **Collision debug visualization** (RViz markers):

    * When `debug_visualize_collisions` is enabled, publishes `MarkerArray` on `~/debug_collision_geometry` with separate namespaces for toggling in RViz:
        * `collision_geometry` — all collision geometry objects (gray = safe, red = colliding).
        * `distance_lines_safe` — nearest-point lines for pairs outside the safety zone (gray).
        * `distance_lines_safety_zone` — nearest-point lines for safety-zone pairs, colored per-pair: green = motion increases distance, red = motion decreases distance, yellow = no directional info.
        * `distance_lines_collision` — nearest-point lines for colliding pairs (bright red, thicker).
* **Status topic** (`~/status`):

    * Publishes `SafetyPositionControllerStatus` (latched) with diagnostic fields: `min_collision_distance`, `distance_scale`, `effective_scale`, `worst_directional_derivative`, `num_pairs_in_safety_zone`, plus safety bypass, E-stop, and mode flags.
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
| `check_self_collisions`            | `bool`     | `true`  | If `true`, performs self-collision checks and enables distance-based velocity scaling. *(read-only)*                 |
| `collision_padding`                | `double`   | `0.01`  | Minimum allowed link-to-link distance [m]; distances ≤ padding are treated as collision. Bounds: [0.0, 1.0].       |
| `collision_safety_zone`            | `double`   | `0.05`  | Outer safety zone distance [m]. Between `collision_padding` and this value, velocity is linearly scaled down. Must be > `collision_padding`. |
| `directional_collision_scaling`    | `bool`     | `true`  | If `true`, velocity scaling near collisions is direction-aware: only motions that decrease any collision distance in the safety zone are slowed down. Motions moving away proceed at full speed. Requires `check_self_collisions`. |
| `collision_cache_epsilon`          | `double`   | `1e-6`  | Threshold [rad] for reusing the previous collision result (skip recomputation if the pose change is below this value). |
| `use_broadphase`                   | `bool`     | `true`  | If `true`, uses AABB-tree broadphase acceleration for self-collision distance queries (3–5× speedup). Only relevant when `check_self_collisions` is true. *(read-only)* |
| `block_velocity_scaling`           | `double`   | `1.5`   | Scales maximum per-cycle motion: allowed step = `velocity_limit / update_rate * block_velocity_scaling`. Bounds: [0.01, 15.0]. Only active when `check_self_collisions` is true. |
| `debug_visualize_collisions`       | `bool`     | `false` | If `true`, publishes collision debug markers for RViz (via `CollisionChecker`). Uses separate marker namespaces for toggling. |
| `publish_collision_distances`      | `bool`     | `false` | If `true`, publishes lightweight distance-only markers for safety zone / collision pairs via a realtime publisher. Cheaper than `debug_visualize_collisions`. Ignored when full debug visualization is active. |
| `set_current_limits`               | `bool`     | `false` | If `true`, enables writing `<joint>/current` limits for compliant/stiff modes. *(read-only)*                        |
| `current_limits.*.compliant_limit` | `double`   | `3.0`   | Per-joint current limit in **compliant** mode [A].                                                                  |
| `current_limits.*.stiff_limit`     | `double`   | `5.0`   | Per-joint current limit in **stiff** mode [A].                                                                      |
| `publish_debug_joint_states`       | `bool`     | `false` | If `true`, publishes debug `JointState` messages for incoming references and outgoing commands.                     |
| `safety_bypass_timeout`            | `double`   | `60.0`  | Time in seconds after which safety bypass auto-disables. Bounds: [0.1, 600.0].                                     |
| `safety_bypass_joint_limit_tolerance` | `double` | `0.03`  | Tolerance factor added to joint limits during bypass. E.g., 0.03 = 3% of the total joint range added to both sides. Bounds: [0.0, 1.0]. |

> **Note:** Parameters are read/updated at `on_activate()`. To apply runtime changes reliably, deactivate and reactivate the controller.

---

### Topics & Services

**Subscriptions**

| Topic                        | Type                                | Used for                                                                     |
| ---------------------------- | ----------------------------------- | ---------------------------------------------------------------------------- |
| `~/commands`                 | `std_msgs/Float64MultiArray`        | **Non-chained mode**: incoming position commands (one per joint).            |
| `~/safety_estop`             | `std_msgs/Bool`                     | E-Stop control. `true` → latch and hold current positions; `false` → resume. |
| `robot_description_semantic` | `std_msgs/String` (transient local) | SRDF XML for self-collision checking.                                        |

**Publications**

| Topic                           | Type                                    | Content                                                                | Condition |
| ------------------------------- | --------------------------------------- | ---------------------------------------------------------------------- | --------- |
| `~/status`                      | `SafetyPositionControllerStatus`        | Latched diagnostics: collision distances, scaling factors, safety state. | Always |
| `~/debug_collision_geometry`    | `visualization_msgs/MarkerArray`        | Collision geometry, distance lines, colored by directional scaling.     | `debug_visualize_collisions = true` or `publish_collision_distances = true` |
| `~/debug_in_joint_states`       | `sensor_msgs/JointState`                | Names = `joints`, positions = current **references** (input commands). | `publish_debug_joint_states = true` |
| `~/debug_out_joint_states`      | `sensor_msgs/JointState`                | Names = `joints`, positions = final **commanded** joint positions.     | `publish_debug_joint_states = true` |

**Services**

| Service                    | Type               | Description                                                                                          |
| -------------------------- | ------------------ | ---------------------------------------------------------------------------------------------------- |
| `~/enforce_current_limits` | `std_srvs/SetBool` | Enable (`true`) or disable (`false`) compliant mode (switch between compliant/stiff current limits). Only available when `set_current_limits = true`. |
| `~/bypass_safety_checks`   | `std_srvs/SetBool` | Enable (`true`) or disable (`false`) safety bypass mode. Disables collision checks and relaxes joint limits. Auto-disables after `safety_bypass_timeout` seconds. Joint wrapping remains active. |

---

## 2. Safety Forward Controller

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

## 3. Velocity-to-Position Command Controller

Converts **velocity references** into **position commands** for joint hardware. Useful when an upstream controller outputs velocity commands but the hardware only accepts position commands.

> **Important:** This controller does **not** perform self-collision checking. It should be chained downstream of the **Safety Position Controller** (see section 4) to ensure collision avoidance and joint limit enforcement. A typical chain is:
> `upstream velocity source → flipper_velocity_to_position_controller → safety_position_controller → hardware`

### Features

* **Feedforward integration with PD velocity tracking:**
    * Integrates velocity commands into a desired position trajectory.
    * A proportional term corrects for velocity tracking error (commanded vs actual).
    * A derivative term damps acceleration to prevent overshoot and oscillation.
* **Velocity clamping:**
    * All incoming velocity references are clamped to `[-max_velocity, max_velocity]`.
* **Desired position tracking:**
    * Maintains an internal `desired_positions` trajectory, providing implicit position error correction.
    * On state transitions (STOPPED/STOPPING → MOVING), the desired position re-syncs to the actual joint position to prevent jumps.
* **State machine** (per joint):
    * **MOVING** – actively integrating velocity commands.
    * **STOPPING** – velocity command is zero; joint follows a trapezoidal deceleration profile at `max_deceleration` until stopped.
    * **STOPPED** – joint velocity has reached zero; holds the last desired position.
* **Trapezoidal braking profiles:**
    * When velocity commands go to zero, each joint computes a smooth deceleration-only profile using `max_deceleration`.
    * Per-joint braking always runs, even for joints in a sync pair — each joint independently stops at `current_pos + braking_distance`. Coordinated braking that intentionally preserves a sync offset is available explicitly via `~/sync_flipper_group`.
* **Joint synchronization:**
    * Joints in the same synchronization group are kept aligned via P-control on position offsets while moving.
    * Braking is independent per joint; the offset between sync partners may drift during a stop and is then re-aligned by `~/sync_flipper_group` if needed.
* **Velocity command timeout:**
    * If no new non-zero velocity command arrives within `velocity_command_timeout` seconds, all velocity references are zeroed. Disabled when set to 0.
    * Works in both chained and standalone modes.
* **Drive flipper group action** (`~/drive_flipper_group`):
    * Drives all joints in a synchronous group to a target position using trapezoidal velocity profiles.
    * Configurable target position, max velocity, and max acceleration per goal (defaults to `upright_position`, `max_velocity`, `max_acceleration` parameters).
    * **Immediately cancelled** when a non-zero velocity command arrives for any joint in the group.
    * Goals targeting a group that is already running another goal are **rejected** at goal acceptance — clients must cancel the in-flight goal first.
* **Sync flipper group action** (`~/sync_flipper_group`):
    * Drives each joint in the requested groups to the average position of its group.
    * Supports syncing **multiple groups simultaneously** in a single action call. Empty `group_names` = sync all groups.
    * Uses per-joint trapezoidal profiles (each joint may travel a different distance).
    * **Immediately cancelled** when a non-zero velocity command arrives for any joint in a synced group.
    * Goals are **rejected** if any target group is already executing another goal. If acceptance succeeds but a later group fails to start (e.g. invalid joint state), already-started groups are rolled back so the failed goal result accurately reflects no in-flight motion.
* **URDF position limit clamping:**
    * Position commands are clamped to URDF joint limits for revolute/prismatic joints. Continuous joints are unclamped.
* **E-Stop support:**
    * Subscribes to an E-Stop topic and tracks the latest state. The freeze-on-engage logic in the update loop is currently disabled (TODO); the subscription remains so downstream consumers can read the state.
* **Chainable controller:**
    * Can receive velocity references from an upstream controller or from a `~/commands` topic.
* **Debug joint state publishers:**
    * Optionally publishes incoming velocity references and outgoing position commands as `sensor_msgs/JointState` (dynamically togglable).
* **Dynamic parameter reconfiguration:**
    * `kp`, `kd`, `kp_sync`, `kd_sync`, `sync_velocity_factor`, `sync_velocity_min_threshold`, `max_velocity`, `max_acceleration`, and `max_deceleration` can be changed at runtime via ROS parameter callbacks.

### Control Law

```
desired_pos[i] += vel_cmd * dt                          // feedforward integration
vel_p = kp * (vel_cmd - vel_actual) * dt                // velocity P-term
vel_d = kd * vel_actual * dt                            // velocity D-term (damping)
pos_cmd = desired_pos[i] + vel_p - vel_d + sync_correction
```

### Parameters

| Name                            | Type           | Default                  | Description                                                                                                                                     |
|---------------------------------|----------------|--------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------|
| **joints**                      | `string_array` | `[]`                     | Names of the joints to control.                                                                                                                 |
| **kp**                          | `double`       | `1.0`                    | Proportional gain for velocity tracking.                                                                                                        |
| **kd**                          | `double`       | `0.1`                    | Derivative gain for acceleration damping.                                                                                                       |
| **kp_sync**                     | `double`       | `1.0`                    | Proportional gain for synchronization of synced joints during movement.                                                                         |
| **kd_sync**                     | `double`       | `0.1`                    | Derivative gain for synchronization (damps oscillation using the velocity difference between sync partners).                                    |
| **sync_velocity_factor**        | `double`       | `1.0`                    | Maximum sync correction as a factor of the commanded velocity.                                                                                  |
| **sync_velocity_min_threshold** | `double`       | `0.1`                    | Minimum absolute sync correction (rad/s) applied regardless of commanded velocity. Ensures sync works at low speeds and at rest.                |
| **stopping_velocity_threshold** | `double`       | `0.005`                  | Velocity threshold below which a joint is considered stopped.                                                                                   |
| **max_velocity**                | `double`       | `1.0`                    | Maximum velocity in rad/s. Clamps incoming velocity references and limits velocity during action profiles.                                      |
| **max_acceleration**            | `double`       | `2.0`                    | Maximum acceleration in rad/s². Used for drive actions and sync actions.                                                                        |
| **max_deceleration**            | `double`       | `4.0`                    | Maximum deceleration in rad/s². Used for braking profiles when velocity commands go to zero. Higher than max_acceleration for faster stopping.  |
| **synchronous_groups**          | `string_array` | `[]`                     | Group names per joint for synchronization (empty = no sync).                                                                                    |
| **passthrough_controller**      | `string`       | `""`                     | Prefix for the lower-level controller exposing command interfaces.                                                                              |
| **e_stop_topic**                | `string`       | `estop_board/hard_estop` | Topic for emergency stop messages (`std_msgs/Bool`).                                                                                            |
| **velocity_command_timeout**    | `double`       | `0.0`                    | Timeout in seconds. If no non-zero velocity command arrives within this duration, references are zeroed. 0 disables.                            |
| **upright_position**            | `double`       | `1.517`                  | Default target position for the drive flipper group action (rad).                                                                               |
| **publish_debug_joint_states**  | `bool`         | `false`                  | If `true`, publishes `~/debug_in_joint_states` (velocity refs) and `~/debug_out_joint_states` (position cmds) as `sensor_msgs/JointState`.     |

### Topics

| Topic                      | Type                     | Description                                                            |
|----------------------------|--------------------------|------------------------------------------------------------------------|
| `~/commands`               | `Float64MultiArray`      | Velocity commands (not used in chained mode).                          |
| E-Stop topic               | `Bool`                   | Engages (`true`) or releases (`false`) the e-stop.                     |
| `~/debug_in_joint_states`  | `sensor_msgs/JointState` | Incoming velocity references (only when `publish_debug_joint_states`). |
| `~/debug_out_joint_states` | `sensor_msgs/JointState` | Outgoing position commands (only when `publish_debug_joint_states`).   |
| `~/sync_status`            | `hector_ros_controllers_msgs/SyncStatus` | Per-joint input/output velocities and desired/current sync offsets (only when `publish_debug_joint_states`). |

### Actions

| Action                      | Type                                              | Description                                                                                                                                     |
|-----------------------------|---------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------|
| `~/drive_flipper_group`     | `hector_ros_controllers_msgs/DriveFlipperGroup`   | Drives all joints in a synchronous group to a target position using trapezoidal profiles. Cancelled immediately by velocity commands on the group. |
| `~/sync_flipper_group`      | `hector_ros_controllers_msgs/SyncFlipperGroup`    | Syncs one or more flipper groups to their average positions. Supports simultaneous multi-group sync. Cancelled immediately by velocity commands.  |


---

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
      collision_safety_zone: 0.05
      directional_collision_scaling: true
      debug_visualize_collisions: false
      block_velocity_scaling: 1.5
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
