# Hector Controller Spawner - Multispawner – ROS2 Hardware & Controller Launcher

Multispawner is a lightweight ROS2 node that boots an entire *ros2\_control* setup in a single shot. It resolves the tedium of juggling multiple **spawner** processes by batching every step:

* **Wait‑for‑safety/Wait-for-Hardware:** Optionally blocks on an emergency‑stop (`std_msgs/Bool`) topic before doing anything. The motors may be impossible to activate while the e‑stop is engaged.
* **Hardware first:** Ensures every listed hardware interface is *loaded* **and** *active* (with automatic retries).
* **Smart loading:** Loads only the controllers that are missing (skips those already present).
* **Reduced overhead:** No per‑controller spawner nodes required, just one multispawner node.

---

## Key Parameters

| Name                      | Type       | Default | Purpose                                                             |
| ------------------------- | ---------- | ------- | ------------------------------------------------------------------- |
| `hardware_interfaces`     | `string[]` | —       | Ordered list of hardware interface names to activate.               |
| `controllers`             | `string[]` | —       | Ordered list of controller names under management.                  |
| `<ctrl>.activate`         | `bool`     | `true`  | Activate this controller after loading?                             |
| `<ctrl>.retry_on_failure` | `bool`     | `false` | Keep retrying the *load* step if it fails?                          |
| `retry_delay`             | `double`   | `5.0`   | Seconds between retry attempts.                                     |
| `estop_topic`             | `string`   | ""      | Topic to wait on (false ⇒ proceed). Empty string disables the gate. |

See **athena.yaml** for a full example.

---

## Typical Usage

```bash
ros2 launch hector_controller_spawner hector_controller_spawner_launch.yml
```
Add it to a launch file exactly once—no per‑controller spawner nodes required.
