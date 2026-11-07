# dynamics_mock_hardware

Mock hardware interface that supports position, velocity, effort (torque) control and uses pinocchio to simulate the robots behavior.

A joint that declares `<command_interface name="torque_off"/>` goes limp while a controller claims
that interface (for example `claim_only_controller/ClaimOnlyController`), as a Dynamixel joint does.
When the claim ends alone the joint stays limp. Torque comes back when a motion command interface
of the joint starts, holding where the joint was left.

A NaN command means no target, as on the Dynamixel driver. A NaN position holds where the joint was
at the first NaN. A NaN velocity commands zero velocity, so a joint under gravity sags slowly, as
the real arm does in velocity mode.

- [dynamics_mock_hardware](#dynamics_mock_hardware)


## `dynamics_mock_hardware`

### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
|  |  |  |

### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
|  |  |  |

### Services

| Service | Type | Description |
| --- | --- | --- |
|  |  |  |

### Actions

| Action | Type | Description |
| --- | --- | --- |
|  |  |  |

### Parameters

| Parameter | Type | Description |
| --- | --- | --- |
|  |  |  |
