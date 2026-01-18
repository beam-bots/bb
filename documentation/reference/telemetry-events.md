<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Telemetry Events Reference

BB emits `:telemetry` events for performance monitoring and diagnostics.

## Event Types

### Span Events

Span events use `:telemetry.span/3` which automatically emits:
- `[:prefix, :start]` - When operation begins
- `[:prefix, :stop]` - When operation completes successfully
- `[:prefix, :exception]` - When operation raises

### Diagnostic Events

Single events emitted for component health reporting.

## Motion Events

### `[:bb, :motion, :solve]`

IK solver execution.

**Start Measurements:**

| Key | Type | Description |
|-----|------|-------------|
| `system_time` | `integer` | System time at start |

**Stop Measurements:**

| Key | Type | Description |
|-----|------|-------------|
| `duration` | `native_time` | Execution duration |
| `monotonic_time` | `integer` | Monotonic time at completion |

**Metadata:**

| Key | Type | Description |
|-----|------|-------------|
| `robot` | `atom` | Robot module |
| `target_link` | `atom` | End-effector link |
| `solver` | `module` | IK solver module used |

**Stop Metadata (additional):**

| Key | Type | Description |
|-----|------|-------------|
| `iterations` | `integer` | Solver iterations |
| `residual` | `float` | Final position error |
| `reached` | `boolean` | Whether target was reached |

### `[:bb, :motion, :move_to]`

Full move operation (solve + send positions).

**Stop Measurements:**

| Key | Type | Description |
|-----|------|-------------|
| `duration` | `native_time` | Total operation duration |

**Metadata:**

| Key | Type | Description |
|-----|------|-------------|
| `robot` | `atom` | Robot module |
| `target_link` | `atom` | End-effector link |

### `[:bb, :motion, :send_positions]`

Sending positions to actuators.

**Stop Measurements:**

| Key | Type | Description |
|-----|------|-------------|
| `duration` | `native_time` | Send duration |

**Metadata:**

| Key | Type | Description |
|-----|------|-------------|
| `robot` | `atom` | Robot module |
| `joint_count` | `integer` | Number of joints updated |
| `delivery` | `atom` | `:pubsub`, `:direct`, or `:sync` |

## Kinematics Events

### `[:bb, :kinematics, :forward]`

Forward kinematics computation.

**Stop Measurements:**

| Key | Type | Description |
|-----|------|-------------|
| `duration` | `native_time` | Computation duration |

**Metadata:**

| Key | Type | Description |
|-----|------|-------------|
| `robot` | `atom` | Robot module |
| `target_link` | `atom` | Link to compute pose for |

## Command Events

### `[:bb, :command, :execute]`

Command execution span.

**Stop Measurements:**

| Key | Type | Description |
|-----|------|-------------|
| `duration` | `native_time` | Total command duration |

**Metadata:**

| Key | Type | Description |
|-----|------|-------------|
| `robot` | `atom` | Robot module |
| `command` | `atom` | Command name |
| `execution_id` | `reference` | Unique execution ID |

## Diagnostic Events

### `[:bb, :diagnostic]`

Component health diagnostic.

**Measurements:** Empty (`%{}`)

**Metadata:** `BB.Diagnostic` struct

See `BB.Diagnostic` module for diagnostic event details.

## Subscribing to Events

### Single Event

```elixir
:telemetry.attach(
  "my-handler",
  [:bb, :motion, :solve, :stop],
  &MyApp.handle_solve_complete/4,
  nil
)
```

### Multiple Events

```elixir
:telemetry.attach_many(
  "my-perf-handler",
  [
    [:bb, :motion, :solve, :stop],
    [:bb, :motion, :move_to, :stop],
    [:bb, :command, :execute, :stop]
  ],
  &MyApp.handle_perf_event/4,
  nil
)
```

## Handler Example

```elixir
defmodule MyApp.TelemetryHandler do
  require Logger

  def handle_event([:bb, :motion, :solve, :stop], measurements, metadata, _config) do
    duration_ms = System.convert_time_unit(measurements.duration, :native, :millisecond)

    Logger.info(
      "IK solve for #{metadata.robot} completed in #{duration_ms}ms " <>
      "(#{metadata.iterations} iterations, residual: #{metadata.residual})"
    )
  end

  def handle_event([:bb, :motion, :solve, :exception], _measurements, metadata, _config) do
    Logger.error("IK solve for #{metadata.robot} failed")
  end
end
```

## Converting Duration

Durations are in native time units. Convert for display:

```elixir
# To milliseconds
duration_ms = System.convert_time_unit(duration, :native, :millisecond)

# To microseconds (for high-precision)
duration_us = System.convert_time_unit(duration, :native, :microsecond)
```

## Emitting Custom Events

Use `BB.Telemetry` helpers:

```elixir
# Span (start/stop automatically)
BB.Telemetry.span([:bb, :custom, :operation], %{robot: MyRobot}, fn ->
  result = do_work()
  {result, %{items_processed: 10}}
end)

# Single event
BB.Telemetry.emit([:bb, :custom, :event], %{count: 1}, %{robot: MyRobot})
```

## Metrics Collection

Example with `telemetry_metrics` and `telemetry_poller`:

```elixir
# In your application supervision tree
children = [
  {Telemetry.Metrics.ConsoleReporter, metrics: metrics()}
]

defp metrics do
  [
    Telemetry.Metrics.distribution(
      "bb.motion.solve.duration",
      event_name: [:bb, :motion, :solve, :stop],
      measurement: :duration,
      unit: {:native, :millisecond},
      tags: [:robot, :solver]
    ),
    Telemetry.Metrics.counter(
      "bb.command.execute.count",
      event_name: [:bb, :command, :execute, :stop],
      tags: [:robot, :command]
    )
  ]
end
```

## Event Naming Convention

BB follows the telemetry naming convention:

```
[:bb, :subsystem, :operation]
[:bb, :subsystem, :operation, :start]
[:bb, :subsystem, :operation, :stop]
[:bb, :subsystem, :operation, :exception]
```

Subsystems:
- `motion` - Motion planning and execution
- `kinematics` - Kinematic computations
- `command` - Command system
- `diagnostic` - Health diagnostics
