<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Reactive Controllers

## Overview

Reactive controllers monitor PubSub messages and trigger actions when conditions are met. They provide a declarative way to implement common reactive patterns like threshold monitoring and event-driven responses without writing custom controller code.

## Controller Types

BB provides two reactive controller types:

| Controller | Purpose |
|------------|---------|
| `BB.Controller.PatternMatch` | Triggers when a message matches a predicate function |
| `BB.Controller.Threshold` | Triggers when a numeric field exceeds min/max bounds |

`Threshold` is a convenience wrapper around `PatternMatch` - internally it generates a match function from the field and bounds configuration.

## Actions

When a condition is met, the controller executes an action. Two action types are available:

### Command Action

Invokes a robot command:

```elixir
action: command(:disarm)
action: command(:move_to, target: :home)
```

### Callback Action

Calls an arbitrary function with the triggering message and context:

```elixir
action: handle_event(fn msg, ctx ->
  Logger.warning("Threshold exceeded: #{inspect(msg.payload)}")
  # ctx contains: robot_module, robot, robot_state, controller_name
  :ok
end)
```

The callback receives:
- `msg` - The `BB.Message` that triggered the action
- `ctx` - A `BB.Controller.Action.Context` struct with robot references

## Configuration

### PatternMatch Options

| Option | Type | Required | Description |
|--------|------|----------|-------------|
| `:topic` | `[atom]` | Yes | PubSub topic path to subscribe to |
| `:match` | `fn msg -> boolean` | Yes | Predicate that returns true when action should trigger |
| `:action` | action | Yes | Action to execute (see Actions above) |
| `:cooldown_ms` | integer | No | Minimum ms between triggers (default: 1000) |

### Threshold Options

| Option | Type | Required | Description |
|--------|------|----------|-------------|
| `:topic` | `[atom]` | Yes | PubSub topic path to subscribe to |
| `:field` | atom or `[atom]` | Yes | Field path to extract from message payload |
| `:min` | float | One required | Minimum acceptable value |
| `:max` | float | One required | Maximum acceptable value |
| `:action` | action | Yes | Action to execute when threshold exceeded |
| `:cooldown_ms` | integer | No | Minimum ms between triggers (default: 1000) |

At least one of `:min` or `:max` must be provided for Threshold.

## Examples

### Current Limiting

Disarm the robot if servo current exceeds safe limits:

```elixir
defmodule MyRobot do
  use BB

  controller :over_current, {BB.Controller.Threshold,
    topic: [:sensor, :servo_status],
    field: :current,
    max: 1.21,
    action: command(:disarm)
  }
end
```

### Collision Detection

React to proximity sensor readings:

```elixir
controller :collision, {BB.Controller.PatternMatch,
  topic: [:sensor, :proximity],
  match: fn msg -> msg.payload.distance < 0.05 end,
  action: command(:disarm)
}
```

### Temperature Monitoring with Callback

Log warnings when temperature is outside safe range:

```elixir
controller :temp_monitor, {BB.Controller.Threshold,
  topic: [:sensor, :temperature],
  field: :value,
  min: 10.0,
  max: 45.0,
  cooldown_ms: 5000,
  action: handle_event(fn msg, ctx ->
    Logger.warning("[#{ctx.controller_name}] Temperature out of range: #{msg.payload.value}°C")
    :ok
  end)
}
```

### Nested Field Access

Access nested fields in message payloads:

```elixir
controller :voltage_monitor, {BB.Controller.Threshold,
  topic: [:sensor, :power],
  field: [:battery, :voltage],  # Accesses msg.payload.battery.voltage
  min: 11.0,
  action: command(:disarm)
}
```

## Cooldown Behaviour

The `:cooldown_ms` option prevents rapid repeated triggering. After an action executes, the controller ignores matching messages until the cooldown period elapses. This is useful for:

- Preventing command spam from noisy sensors
- Allowing time for the triggered action to take effect
- Reducing log noise from callback actions

The first matching message always triggers immediately (no initial delay).

## Integration with Commands

Reactive controllers work alongside the command system. When a controller triggers `command(:disarm)`, it's equivalent to calling `MyRobot.disarm([])` - the command goes through the normal command execution flow with state machine validation.

This means:
- Commands are logged via telemetry
- State machine rules apply (can't disarm if already disarmed)
- Command results are returned (but typically ignored by the controller)

## When to Use Reactive Controllers

**Good use cases:**
- Safety limits (current, temperature, force thresholds)
- Event-driven responses (collision detection, limit switches)
- Monitoring and alerting (logging unusual conditions)

**Consider alternatives when:**
- You need complex logic spanning multiple messages (use a custom controller)
- You need to modify robot state directly (use a custom controller with `handle_info`)
- You need request/response patterns (use commands instead)
