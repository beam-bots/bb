<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Error Types Reference

Structured error types in BB. All errors implement the `BB.Error.Severity` protocol.

## Severity Levels

| Level | Description |
|-------|-------------|
| `:critical` | Immediate safety response required |
| `:error` | Operation failed, may retry or degrade |
| `:warning` | Unusual condition, operation continues |

## Hardware Errors

Communication failures with physical devices.

**Class:** `:hardware`

### BusError

Communication bus failure.

**Module:** `BB.Error.Hardware.BusError`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `bus` | `string` | Bus identifier (e.g., "i2c-1") |
| `reason` | `term` | Underlying error |

**Severity:** `:error`

### DeviceError

Device-level failure.

**Module:** `BB.Error.Hardware.DeviceError`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `device` | `term` | Device identifier |
| `reason` | `term` | Error details |

**Severity:** `:error`

### Disconnected

Device unexpectedly disconnected.

**Module:** `BB.Error.Hardware.Disconnected`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `device` | `term` | Device identifier |

**Severity:** `:error`

### Timeout

Hardware communication timeout.

**Module:** `BB.Error.Hardware.Timeout`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `device` | `term` | Device identifier |
| `operation` | `atom` | Operation that timed out |

**Severity:** `:error`

## Safety Errors

Safety system violations.

**Class:** `:safety`

**All safety errors have severity `:critical`.**

### CollisionRisk

Collision detected or imminent.

**Module:** `BB.Error.Safety.CollisionRisk`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `link` | `atom` | Link at risk |
| `obstacle` | `term` | Obstacle description |

### DisarmFailed

Disarm callback failed.

**Module:** `BB.Error.Safety.DisarmFailed`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `path` | `[atom]` | Process path |
| `reason` | `term` | Failure reason |

### EmergencyStop

Emergency stop triggered.

**Module:** `BB.Error.Safety.EmergencyStop`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `source` | `term` | What triggered the stop |

### LimitExceeded

Joint limit exceeded.

**Module:** `BB.Error.Safety.LimitExceeded`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `joint` | `atom` | Joint name |
| `limit` | `atom` | `:position`, `:velocity`, or `:effort` |
| `value` | `float` | Actual value |
| `max` | `float` | Maximum allowed |

## Kinematics Errors

Motion planning failures.

**Class:** `:kinematics`

### NoDofs

No degrees of freedom available.

**Module:** `BB.Error.Kinematics.NoDofs`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `chain` | `[atom]` | Kinematic chain |

**Severity:** `:error`

### NoSolution

Inverse kinematics found no solution.

**Module:** `BB.Error.Kinematics.NoSolution`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `target` | `Transform` | Requested pose |
| `reason` | `term` | Why no solution exists |

**Severity:** `:error`

### MultiFailed

Multiple IK attempts failed.

**Module:** `BB.Error.Kinematics.MultiFailed`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `attempts` | `integer` | Number of attempts |
| `errors` | `[term]` | Individual errors |

**Severity:** `:error`

### SelfCollision

Motion would cause self-collision.

**Module:** `BB.Error.Kinematics.SelfCollision`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `link_a` | `atom` | First colliding link |
| `link_b` | `atom` | Second colliding link |

**Severity:** `:warning`

### Singularity

Near kinematic singularity.

**Module:** `BB.Error.Kinematics.Singularity`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `joint` | `atom` | Joint near singularity |
| `manipulability` | `float` | Manipulability measure |

**Severity:** `:warning`

### UnknownLink

Referenced link not found.

**Module:** `BB.Error.Kinematics.UnknownLink`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `link` | `atom` | Unknown link name |

**Severity:** `:error`

### Unreachable

Target is outside workspace.

**Module:** `BB.Error.Kinematics.Unreachable`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `target` | `Transform` | Requested pose |
| `distance` | `float` | Distance outside workspace |

**Severity:** `:error`

## Invalid Errors

Configuration and validation errors.

**Class:** `:invalid`

### Command

Invalid command definition.

**Module:** `BB.Error.Invalid.Command`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `command` | `atom` | Command name |
| `reason` | `term` | Validation failure |

**Severity:** `:error`

### JointConfig

Invalid joint configuration.

**Module:** `BB.Error.Invalid.JointConfig`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `joint` | `atom` | Joint name |
| `field` | `atom` | Invalid field |
| `reason` | `term` | Why invalid |

**Severity:** `:error`

### Parameter

Invalid parameter value.

**Module:** `BB.Error.Invalid.Parameter`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `name` | `atom` | Parameter name |
| `value` | `term` | Invalid value |
| `expected` | `term` | Expected type/range |

**Severity:** `:error`

### Topology

Invalid topology definition.

**Module:** `BB.Error.Invalid.Topology`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `path` | `[atom]` | Location in topology |
| `reason` | `term` | Validation failure |

**Severity:** `:error`

## State Errors

State machine violations.

**Class:** `:state`

### NotAllowed

Command not allowed in current state.

**Module:** `BB.Error.State.NotAllowed`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `current_state` | `atom` | Current robot state |
| `allowed_states` | `[atom]` | States where command is allowed |

**Severity:** `:error`

### Invalid

Invalid state transition.

**Module:** `BB.Error.State.Invalid`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `from` | `atom` | Current state |
| `to` | `atom` | Requested state |

**Severity:** `:error`

### Preempted

Command was preempted by another.

**Module:** `BB.Error.State.Preempted`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `command` | `atom` | Original command |
| `preempted_by` | `atom` | Preempting command |

**Severity:** `:warning`

### Timeout

Command execution timeout.

**Module:** `BB.Error.State.Timeout`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `command` | `atom` | Command that timed out |
| `elapsed` | `integer` | Time elapsed (ms) |

**Severity:** `:error`

### CommandCrashed

Command process crashed.

**Module:** `BB.Error.State.CommandCrashed`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `command` | `atom` | Command that crashed |
| `reason` | `term` | Crash reason |

**Severity:** `:error`

## Category Errors

Command category errors.

**Class:** `:category`

### Full

Command category is at capacity.

**Module:** `BB.Error.Category.Full`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `category` | `atom` | Category name |
| `capacity` | `integer` | Maximum concurrent commands |
| `running` | `integer` | Currently running |

**Severity:** `:error`

## Protocol Errors

Low-level protocol failures.

**Class:** `:protocol`

Used by driver packages (e.g., Robotis Dynamixel protocol errors).

## Creating Errors

Always use `exception/1` to create errors so that Splode can capture backtraces:

```elixir
# Create error with exception/1 (captures backtrace)
error = BB.Error.State.NotAllowed.exception(
  current_state: :disarmed,
  allowed_states: [:idle]
)

# Check severity
BB.Error.Severity.severity(error)  #=> :error

# Get message
BB.Error.message(error)  #=> "Command not allowed in state :disarmed..."
```

Do **not** create error structs directly - this bypasses Splode's backtrace capture:

```elixir
# Avoid - no backtrace captured
error = %BB.Error.State.NotAllowed{
  current_state: :disarmed,
  allowed_states: [:idle]
}
```

## Returning Errors

Prefer structured errors over tuples:

```elixir
# Good - use exception/1
{:error, BB.Error.State.NotAllowed.exception(current_state: :disarmed, allowed_states: [:idle])}

# Avoid - tuple-based errors
{:error, {:not_allowed, :disarmed}}
```
