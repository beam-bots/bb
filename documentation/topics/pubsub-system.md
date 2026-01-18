<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Understanding the PubSub System

This document explains Beam Bots' hierarchical publish-subscribe system - how it addresses messages, routes them efficiently, and enables loose coupling between components.

## The Core Idea

BB's PubSub uses hierarchical paths for addressing. Messages are published to paths, and subscribers can match exact paths or entire subtrees:

```elixir
# Publish to a specific path
BB.publish(MyRobot, [:sensor, :shoulder], joint_state)

# Subscribe to exact path
BB.subscribe(MyRobot, [:sensor, :shoulder])

# Subscribe to all sensors (subtree)
BB.subscribe(MyRobot, [:sensor])
```

## Why Hierarchical Addressing?

### Mirrors Robot Structure

Robot components naturally form hierarchies:
- Sensors grouped by type or location
- Actuators organised by kinematic chain
- Controllers managing multiple devices

Hierarchical paths capture this structure:

```
[:sensor, :shoulder]         # Shoulder position sensor
[:sensor, :elbow]            # Elbow position sensor
[:actuator, :shoulder]       # Shoulder servo
[:actuator, :elbow]          # Elbow servo
[:controller, :pca9685]      # PWM controller
[:state_machine]             # State transitions
[:safety]                    # Safety events
[:safety, :error]            # Hardware errors
```

### Flexible Subscription

Different consumers need different granularity:

- **Runtime**: Subscribes to `[:sensor]` - needs all sensor data
- **Logger**: Subscribes to `[:safety]` - only safety events
- **Dashboard**: Subscribes to `[]` (root) - everything
- **Actuator**: Subscribes to `[:actuator, :shoulder]` - just its commands

### Topic Discovery

New components can publish without coordinating with subscribers. The hierarchy provides natural namespacing:

```elixir
# New sensor added - just publish to its path
BB.publish(MyRobot, [:sensor, :wrist], wrist_data)

# Existing [:sensor] subscribers automatically receive it
```

## Message Format

All messages are wrapped in `BB.Message`:

```elixir
%BB.Message{
  payload: %BB.Message.Sensor.JointState{...},
  timestamp: ~U[2025-01-18 12:00:00Z],
  frame_id: "shoulder"
}
```

Subscribers receive:

```elixir
{:bb, path, %BB.Message{} = message}
```

The tuple format lets you pattern match on path:

```elixir
def handle_info({:bb, [:sensor, joint_name], %{payload: joint_state}}, state) do
  # Handle sensor data for any joint
end

def handle_info({:bb, [:safety, :error], %{payload: error}}, state) do
  # Handle safety errors specifically
end
```

## Publishing

### Basic Publishing

Messages are created using the payload module's `new!/2` function, which returns a `BB.Message` struct:

```elixir
# Create a message (returns %BB.Message{payload: %JointState{...}, ...})
message = JointState.new!(:shoulder, name: :shoulder, positions: [0.5])

# Publish the message
BB.publish(MyRobot, [:sensor, :shoulder], message)
```

The first argument to `new!/2` is the `frame_id` (typically the joint or link name), and the second is a keyword list of payload attributes.

### Publish Patterns

Common publishing patterns:

```elixir
# Sensor publishing its readings
BB.publish(robot_module, bb.path, sensor_message)

# Actuator publishing motion start
BB.publish(robot_module, bb.path, begin_motion_message)

# Command publishing events
BB.publish(robot_module, [:command, command_name], progress_message)

# Controller publishing status
BB.publish(robot_module, [:controller, controller_name], status_message)
```

## Subscribing

### Exact Path

```elixir
BB.subscribe(MyRobot, [:sensor, :shoulder])
# Receives: [:sensor, :shoulder] only
```

### Subtree (Prefix)

```elixir
BB.subscribe(MyRobot, [:sensor])
# Receives: [:sensor, :shoulder], [:sensor, :elbow], [:sensor, :wrist], etc.
```

### Root (Everything)

```elixir
BB.subscribe(MyRobot, [])
# Receives: all messages for this robot
```

### Filtering by Message Type

By default, subscriptions receive all message types published to matching paths. Use the `:message_types` option to filter by payload type:

```elixir
# Only receive JointState messages from sensors
BB.subscribe(MyRobot, [:sensor], message_types: [BB.Message.Sensor.JointState])

# Only receive IMU data from a specific sensor
BB.subscribe(MyRobot, [:sensor, :imu], message_types: [BB.Message.Sensor.Imu])

# Multiple types
BB.subscribe(MyRobot, [:sensor], message_types: [
  BB.Message.Sensor.JointState,
  BB.Message.Sensor.Imu
])
```

An empty list (the default) means no filtering - receive all message types at matching paths.

### Unsubscribing

```elixir
BB.unsubscribe(MyRobot, [:sensor, :shoulder])
```

## Routing Mechanics

Under the hood, BB uses Elixir's `Registry` with `keys: :duplicate`:

1. Each robot has its own Registry (started with duplicate keys mode)
2. Subscriptions register the calling process with a path
3. On publish, `Registry.dispatch/3` sends to all processes registered at matching paths
4. BB publishes to the exact path and all ancestor paths (prefix matching)

This is efficient:
- O(1) dispatch per path (Registry handles fan-out)
- No central broker process
- Messages delivered directly to subscribers

## Common Paths

BB uses consistent paths for standard message types:

| Path Pattern | Purpose |
|--------------|---------|
| `[:sensor, name]` | Sensor readings |
| `[:actuator, name]` | Actuator commands |
| `[:controller, name]` | Controller events |
| `[:state_machine]` | State transitions |
| `[:safety]` | Safety events |
| `[:safety, :error]` | Hardware errors |
| `[:param]` | Parameter updates |
| `[:param, name]` | Specific parameter |

## Message Types

BB provides typed message payloads. Key types:

### Sensor Messages

```elixir
%BB.Message.Sensor.JointState{
  name: :shoulder,
  positions: [0.5],
  velocities: [0.1],
  efforts: [0.0]
}
```

### Actuator Messages

```elixir
%BB.Message.Sensor.JointCommand{
  name: :shoulder,
  target: 0.5
}

%BB.Message.Actuator.BeginMotion{
  name: :shoulder,
  initial: 0.0,
  target: 0.5,
  velocity: 1.0
}
```

### Safety Messages

```elixir
%BB.Safety.HardwareError{
  path: [:actuator, :shoulder],
  error: {:overheating, 85.0}
}
```

### State Machine Messages

```elixir
%BB.StateMachine.Transition{
  from: :disarmed,
  to: :idle
}
```

## Design Patterns

### Sensor → Runtime → State

The standard flow for position feedback:

```
Sensor ──publish──→ [:sensor, :name] ──subscribe──→ Runtime
                                                        │
                                                        ▼
                                              Update joint state
```

### Actuator → Sensor (via OpenLoop)

For servos without feedback:

```
Command ──publish──→ [:actuator, :name] ──subscribe──→ Actuator
                                                           │
                                                    send to hardware
                                                           │
                          publish BeginMotion ─────────────┘
                                  │
                                  ▼
                        OpenLoopPositionEstimator
                                  │
                          publish JointState ──→ [:sensor, :name]
```

### Dashboard Aggregation

Dashboards subscribe broadly:

```elixir
def mount(_params, _session, socket) do
  BB.subscribe(robot_module, [:sensor])
  BB.subscribe(robot_module, [:state_machine])
  BB.subscribe(robot_module, [:safety])
  ...
end
```

### Command Feedback

Commands subscribe to relevant sensors:

```elixir
def handle_command(goal, context, state) do
  BB.subscribe(context.robot_module, [:sensor, goal.joint])
  ...
end

def handle_info({:bb, [:sensor, _joint], %{payload: joint_state}}, state) do
  # Check if target reached
end
```

## Performance Considerations

### High-Frequency Messages

Sensors might publish at 100Hz+. Subscribers should:
- Process quickly or buffer
- Consider throttling if display-only
- Use async handling if processing is slow

### Many Subscribers

With many processes subscribing to the same path:
- Each gets a copy of the message
- Consider a single aggregator if processing is identical
- Registry dispatch is efficient but not free

### Large Messages

The PubSub system copies messages to each subscriber. For large payloads:
- Consider reference-passing (ETS, :persistent_term)
- Publish only changed data
- Compress if over network

## Debugging

### See All Messages

```elixir
BB.subscribe(MyRobot, [])
# In iex, you'll see all {:bb, path, message} tuples
```

### Message Counts

```elixir
# In a GenServer
def init(_) do
  BB.subscribe(MyRobot, [])
  {:ok, %{counts: %{}}}
end

def handle_info({:bb, path, _msg}, %{counts: counts} = state) do
  key = Enum.take(path, 2) |> Enum.join(".")
  {:noreply, %{state | counts: Map.update(counts, key, 1, &(&1 + 1))}}
end
```

### Path Discovery

```elixir
# List all paths that have been published (requires custom tracking)
# Or use the Event Stream widget in bb_liveview/bb_kino
```

## Related Documentation

- [Sensors and PubSub](../tutorials/03-sensors-and-pubsub.md) - Tutorial
- [Reference: Message Types](../reference/message-types.md) - All message types
