# dynamics_mock_hardware

Mock hardware interface that supports position, velocity, effort (torque) control and uses pinocchio to simulate the robots behavior.

A joint that declares `<command_interface name="torque_off"/>` goes limp while a controller claims
that interface (for example `claim_only_controller/ClaimOnlyController`), as a Dynamixel joint does.
When the claim ends the joint holds where it was left.

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
