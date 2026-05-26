<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Message Types Reference

All message payloads in BB PubSub. Messages are wrapped in `BB.Message`:

```elixir
%BB.Message{
  monotonic_time: integer(),  # System.monotonic_time(:nanosecond)
  wall_time: integer(),       # System.system_time(:nanosecond)
  node: node(),               # Node that produced the message
  frame_id: atom(),           # Coordinate frame (typically joint/link name)
  payload: struct()           # One of the message types below
}
```

**Note on timing fields:**
- `monotonic_time` is monotonic nanoseconds from `System.monotonic_time/1`. Use it for ordering events and measuring durations within a single node. It is **not** comparable across nodes or BEAM restarts.
- `wall_time` is system time in nanoseconds from `System.system_time/1`. Use it for correlation with real-world time, log alignment, and recording/playback. Subject to NTP jumps.
- `node` is the originating BEAM node, useful for disambiguating messages in distributed deployments.

## Sensor Messages

### JointState

State of one or more joints.

**Module:** `BB.Message.Sensor.JointState`

**Fields:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `names` | `[atom]` | Yes | Joint names |
| `positions` | `[float]` | No | Positions in radians (revolute) or metres (prismatic) |
| `velocities` | `[float]` | No | Velocities in rad/s or m/s |
| `efforts` | `[float]` | No | Efforts in Nm or N |

**Published to:** `[:sensor, joint_name]`

**Example:**
```elixir
JointState.new!(
  names: [:shoulder, :elbow],
  positions: [0.5, 1.2],
  velocities: [0.1, 0.0]
)
```

### BatteryState

Battery status information.

**Module:** `BB.Message.Sensor.BatteryState`

**Fields:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `voltage` | `float` | Yes | Battery voltage in volts |
| `current` | `float` | No | Current draw in amps |
| `percentage` | `float` | No | Charge percentage (0.0-1.0) |
| `present` | `boolean` | No | Whether battery is present |

**Published to:** `[:sensor, :battery]` or custom path

### IMU

Inertial measurement unit data.

**Module:** `BB.Message.Sensor.Imu`

**Fields:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `orientation` | `Quaternion` | No | Orientation quaternion |
| `angular_velocity` | `Vec3` | No | Angular velocity in rad/s |
| `linear_acceleration` | `Vec3` | No | Linear acceleration in m/s² |
| `orientation_covariance` | `Covariance3 \| nil` | No | 3x3 covariance over orientation. `nil` when unknown. |
| `angular_velocity_covariance` | `Covariance3 \| nil` | No | 3x3 covariance over angular velocity. `nil` when unknown. |
| `linear_acceleration_covariance` | `Covariance3 \| nil` | No | 3x3 covariance over linear acceleration. `nil` when unknown. |

Drivers with known noise characteristics populate the covariance fields; ones that don't leave them `nil`. Algorithms that require covariance check for `nil` and either fall back to a configured default or raise `BB.Error.Estimator.MissingCovariance`. Mirrors the ROS `sensor_msgs/Imu` shape.

**Published to:** `[:sensor, :imu]` or custom path

### LaserScan

2D laser range finder data.

**Module:** `BB.Message.Sensor.LaserScan`

**Fields:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `angle_min` | `float` | Yes | Start angle in radians |
| `angle_max` | `float` | Yes | End angle in radians |
| `angle_increment` | `float` | Yes | Angle between measurements |
| `ranges` | `[float]` | Yes | Range measurements in metres |
| `intensities` | `[float]` | No | Intensity values |

**Published to:** `[:sensor, :lidar]` or custom path

### Range

Single distance measurement.

**Module:** `BB.Message.Sensor.Range`

**Fields:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `range` | `float` | Yes | Measured distance in metres |
| `min_range` | `float` | No | Minimum valid range |
| `max_range` | `float` | No | Maximum valid range |
| `radiation_type` | `atom` | No | `:ultrasound` or `:infrared` |

**Published to:** `[:sensor, sensor_name]`

### Image

Camera image data.

**Module:** `BB.Message.Sensor.Image`

**Fields:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `height` | `integer` | Yes | Image height in pixels |
| `width` | `integer` | Yes | Image width in pixels |
| `encoding` | `string` | Yes | Pixel encoding (e.g., "rgb8", "mono8") |
| `data` | `binary` | Yes | Raw image data |

**Published to:** `[:sensor, :camera]` or custom path

## Actuator Messages

### BeginMotion

Published when an actuator starts moving.

**Module:** `BB.Message.Actuator.BeginMotion`

**Fields:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `initial_position` | `float` | Yes | Starting position |
| `target_position` | `float` | Yes | Target position |
| `expected_arrival` | `integer` | Yes | Expected completion time (monotonic ms) |
| `command_id` | `reference` | No | Correlation ID |
| `command_type` | `atom` | No | `:position`, `:velocity`, `:effort`, `:trajectory` |

**Published to:** `[:actuator, actuator_name]`

**Used by:** `OpenLoopPositionEstimator` for position feedback without encoders.

### EndMotion

Published when an actuator completes a motion.

**Module:** `BB.Message.Actuator.EndMotion`

**Fields:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `final_position` | `float` | Yes | Achieved position |
| `command_id` | `reference` | No | Correlation ID |

**Published to:** `[:actuator, actuator_name]`

## Actuator Command Messages

Commands sent to actuators.

### Command.Position

Position target command.

**Module:** `BB.Message.Actuator.Command.Position`

**Fields:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `target` | `float` | Yes | Target position |

**Published to:** `[:actuator, actuator_name]`

### Command.Velocity

Velocity command.

**Module:** `BB.Message.Actuator.Command.Velocity`

**Fields:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `velocity` | `float` | Yes | Target velocity |

### Command.Effort

Effort (torque/force) command.

**Module:** `BB.Message.Actuator.Command.Effort`

**Fields:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `effort` | `float` | Yes | Target effort |

### Command.Trajectory

Multi-point trajectory command.

**Module:** `BB.Message.Actuator.Command.Trajectory`

**Fields:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `points` | `[TrajectoryPoint]` | Yes | Trajectory waypoints |

### Command.Hold

Hold current position.

**Module:** `BB.Message.Actuator.Command.Hold`

No additional fields.

### Command.Stop

Stop motion immediately.

**Module:** `BB.Message.Actuator.Command.Stop`

No additional fields.

## Geometry Messages

Geometric primitives used as components in other messages.

### Point3D

3D point.

**Module:** `BB.Message.Geometry.Point3D`

**Fields:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `x` | `float` | Yes | X coordinate |
| `y` | `float` | Yes | Y coordinate |
| `z` | `float` | Yes | Z coordinate |

### Pose

Position and orientation.

**Module:** `BB.Message.Geometry.Pose`

**Fields:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `position` | `Point3D` | Yes | Position |
| `orientation` | `Quaternion` | Yes | Orientation |

### Twist

Linear and angular velocity.

**Module:** `BB.Message.Geometry.Twist`

**Fields:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `linear` | `Vec3` | Yes | Linear velocity (m/s) |
| `angular` | `Vec3` | Yes | Angular velocity (rad/s) |

### Accel

Linear and angular acceleration.

**Module:** `BB.Message.Geometry.Accel`

**Fields:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `linear` | `Vec3` | Yes | Linear acceleration (m/s²) |
| `angular` | `Vec3` | Yes | Angular acceleration (rad/s²) |

### Wrench

Force and torque.

**Module:** `BB.Message.Geometry.Wrench`

**Fields:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `force` | `Vec3` | Yes | Force (N) |
| `torque` | `Vec3` | Yes | Torque (Nm) |

## Estimator Messages

Payloads published by `BB.Estimator` implementations. Distinguished from `Geometry.*` by carrying optional uncertainty (covariance) alongside the geometric value. Subscribers wanting fused outputs specifically can filter on these payload types to separate them from waypoints or command goals.

### Estimator.Pose

A 6-DOF pose estimate with optional covariance.

**Module:** `BB.Message.Estimator.Pose`

**Fields:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `transform` | `Transform` | Yes | Pose as a 4x4 homogeneous transform |
| `covariance` | `Covariance6 \| nil` | No | 6x6 covariance over translation (x/y/z) and rotation (r/p/y). `nil` when unknown. |

Canonical output for any estimator publishing a fused pose (AHRS, EKF, visual SLAM front-end).

### Estimator.Odometry

A pose-and-twist odometry estimate with separate optional covariances.

**Module:** `BB.Message.Estimator.Odometry`

**Fields:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `pose` | `Transform` | Yes | Pose component |
| `twist` | `Twist` | Yes | Linear and angular velocity (as `BB.Message.Geometry.Twist`) |
| `pose_covariance` | `Covariance6 \| nil` | No | 6x6 covariance over the pose |
| `twist_covariance` | `Covariance6 \| nil` | No | 6x6 covariance over the twist |

Mirrors ROS `nav_msgs/Odometry`. Canonical output for IMU-plus-wheel-odometry EKFs and any other fusion stage publishing both a pose and a velocity simultaneously.

## System Messages

### StateMachine.Transition

Robot state machine transition.

**Published to:** `[:state_machine]`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `from` | `atom` | Previous state |
| `to` | `atom` | New state |

### Safety.HardwareError

Hardware error report.

**Published to:** `[:safety, :error]`

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `path` | `[atom]` | Path to component |
| `error` | `term` | Error details |

## Creating Messages

All message types support `new/1` and `new!/1`:

```elixir
# Returns {:ok, message} or {:error, reason}
{:ok, msg} = JointState.new(names: [:shoulder], positions: [0.5])

# Raises on validation error
msg = JointState.new!(names: [:shoulder], positions: [0.5])
```

## Message Wrapper

Messages are wrapped in `BB.Message`:

```elixir
%BB.Message{
  payload: %JointState{...},
  monotonic_time: -576_460_748_776_542,
  wall_time: 1_737_201_600_000_000_000,
  node: :nonode@nohost,
  frame_id: :shoulder
}
```

Create wrapped messages:

```elixir
BB.Message.new(JointState, :shoulder, names: [:shoulder], positions: [0.5])
```
