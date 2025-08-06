# Hector Controller Spawner – **Multispawner**

**ROS2 Hardware & Controller Launcher for `ros2_control`**

**Multispawner** is a minimal ROS2 node that launches an entire `ros2_control` setup in a single coordinated pass.
It robustly manages hardware interfaces and controllers, ensuring everything is loaded, activated (if required), and
ready to go with minimal configuration.

---

## 🚀 Features

* **Wait-for-safety (e-stop):** Optionally blocks on an `std_msgs/Bool` topic (e.g., emergency stop) before starting.
  Useful when motors can't be activated while safety is engaged.
* **Re-activation:** Automatically reactivates hardware interfaces and controllers after releasing the e-stop (if they
  became inactive).
* **Controller Manager Synchronization:** Waits for the controller manager to become available before proceeding.
* **Hardware-first strategy:** Ensures all listed hardware interfaces are both *loaded* and *activated* (with automatic
  retries on failure).
* **Intelligent controller loading:** Skips controllers already present - only loads and activates what’s missing.
* **Automatic chaining:** Automatically detects and starts *chained controllers* together - no additional config
  required.
* **Single-node simplicity:** No need to spawn one spawner per controller - Multispawner handles everything.
* **Robust retry logic:** Retries failed hardware/controller activations with configurable delays.

---

## 🔧 Key Parameters

| Name                  | Type       | Default | Description                                                               |
|-----------------------|------------|---------|---------------------------------------------------------------------------|
| `hardware_interfaces` | `string[]` | -       | Ordered list of hardware interface names to activate.                     |
| `controllers`         | `string[]` | -       | Ordered list of controller names to load and manage.                      |
| `<ctrl>.activate`     | `bool`     | `true`  | Should the controller be activated after loading?                         |
| `retry_delay`         | `double`   | `5.0`   | Delay (in seconds) between retry attempts.                                |
| `estop_topic`         | `string`   | `""`    | Topic to wait on (false ⇒ proceed). Leave empty to disable e-stop gating. |

📄 See [`athena.yaml`](config/athena.yaml) for a complete configuration example.

---

## 🧪 Example Usage

```bash
ros2 launch hector_controller_spawner hector_controller_spawner_launch.yml
```

* Include **only once** in your launch setup.
* No need for individual `spawner` calls per controller.

