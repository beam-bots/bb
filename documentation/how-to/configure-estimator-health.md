<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# How to Configure Estimator Health

Wire an estimator's `latency_budget` / `lost_after` / `recover_after` timing constraints up to robot commands so the rest of the system reacts when estimation degrades. This guide is task-oriented — for the design rationale behind health-as-commands, see [Understanding Estimators](../topics/understanding-estimators.md).

## Prerequisites

- An estimator declared in your DSL (see [State Estimation](../tutorials/13-state-estimation.md)).
- Familiarity with the [Commands and State Machine](../tutorials/05-commands.md) tutorial.

## The state model

`BB.Estimator.Server` runs a small state machine for each estimator:

```
:healthy  ⇄  :degraded
   │            │
   └─→ :lost ←──┘
```

| Trigger | From → To | Notes |
|---|---|---|
| `handle_input/2` exceeds `latency_budget` | `:healthy → :degraded` | Reason `:latency_overrun` |
| `sync_miss` on a multi-input dispatch | `:healthy → :degraded` | Reason `:sync_miss` |
| No input for `lost_after` | any → `:lost` | Reason `:lost`; reset on every input |
| First input after `:lost` | `:lost → :degraded` | Reason `:recovered`; counter resets to 1 |
| `recover_after` consecutive in-budget dispatches | `:degraded → :healthy` | Reason `:recovered`; hysteresis prevents flapping |

Transitions emit `[:bb, :estimator, :transition]` telemetry whether or not a command is configured.

## Step 1: Declare commands for the transitions you care about

Health policy is a robot-specific decision, so it lives in your `commands do … end` section. Each transition has its own command slot — wire only the ones you need.

```elixir
defmodule MyRobot.Robot do
  use BB

  commands do
    command :pose_degraded do
      handler MyRobot.Commands.SwitchToSlowMode
      allowed_states [:idle, :executing]
    end

    command :pose_lost do
      handler MyRobot.Commands.EmergencyStop
      allowed_states [:idle, :executing, :degraded]
    end

    command :pose_recovered do
      handler MyRobot.Commands.ResumeNormalOperation
      allowed_states [:degraded]
    end
  end

  # ... topology below ...
end
```

The command names are arbitrary — point them at handlers that encode your policy. `allowed_states` works exactly as it does for any other command; if a transition fires while the robot is in a state that disallows the command, the dispatch is rejected by the runtime but the estimator's internal state still moves.

## Step 2: Attach the timing constraints and commands to the estimator

Health options live on the `estimator` DSL entity. The same shape works for both sensor-nested and link-nested estimators:

```elixir
topology do
  link :base_link do
    sensor :imu, BB.Sensor.SomeImu, ... do
      estimator :orientation, {BB.Ahrs.Madgwick, beta: 0.1} do
        latency_budget ~u(20 millisecond)
        lost_after ~u(500 millisecond)
        recover_after 10

        on_degraded :pose_degraded
        on_lost :pose_lost
        on_recovered :pose_recovered
      end
    end
  end
end
```

The verifier checks at compile time that each `on_*` name matches a declared command. A typo produces:

```
estimator :orientation at [:sensor, :base_link, :imu, :orientation]:
  on_degraded references unknown command :pose_degardd.
  Declare it under `commands do ... end` first.
```

## Step 3 (optional): Tune `recover_after` to suppress flapping

`recover_after` (default `1`) is the number of consecutive in-budget completions required before `:degraded → :healthy`. For an estimator that runs at 100 Hz:

```elixir
recover_after 10   # ~100 ms of clean operation before declaring recovery
```

Set it higher for jitter-prone inputs, lower for low-rate estimators that can't afford the recovery delay. A useful rule of thumb: pick a value that corresponds to a few times the natural timescale of whatever transient caused the degradation in the first place.

## Step 4: Use the metadata in your command handler

The configured command receives a structured args map:

```elixir
defmodule MyRobot.Commands.SwitchToSlowMode do
  use BB.Command

  @impl BB.Command
  def handle_command(%{estimator: name, reason: reason, source_path: path}, _ctx, state) do
    Logger.warning(
      "Estimator #{inspect(name)} degraded (reason=#{inspect(reason)}, source=#{inspect(path)})"
    )

    # ... slow down motion, switch control mode, etc. ...

    {:stop, :normal, %{state | result: {:ok, :slowed}}}
  end
end
```

The args shape is the same for all three transition commands:

| Key | Type | Description |
|---|---|---|
| `estimator` | `atom` | The estimator's name (final atom in its path) |
| `reason` | `atom` | `:latency_overrun`, `:sync_miss`, `:lost`, or `:recovered` |
| `source_path` | `[atom] \| nil` | The pubsub path that triggered the transition (when relevant) |
| `previous_state` | `:healthy \| :degraded \| :lost` | State before the transition |
| `new_state` | `:healthy \| :degraded \| :lost` | State after the transition |

## Step 5 (optional): Observe transitions via telemetry

Even with no commands configured, every transition emits `[:bb, :estimator, :transition]`. Useful for logging and dashboards regardless of policy:

```elixir
:telemetry.attach(
  "estimator-transitions",
  [:bb, :estimator, :transition],
  fn _event, _measurements, meta, _config ->
    Logger.info(
      "Estimator #{inspect(meta.estimator)} on #{inspect(meta.robot)}: " <>
        "#{meta.from} → #{meta.to} (reason: #{meta.reason})"
    )
  end,
  nil
)
```

See the [Telemetry Events reference](../reference/telemetry-events.md#bb-estimator-transition) for the full event schema.

## Common patterns

### Block motion while perception is degraded

Define a `:degraded` operational state, transition into it from `on_degraded`, and gate motion commands on the state machine. The state-machine integration is what makes "perception status" a first-class thing rather than a custom global flag:

```elixir
states do
  state :degraded, doc: "Estimation degraded — slower / safer behaviour"
end

commands do
  command :pose_degraded do
    handler MyRobot.Commands.EnterDegradedState
    allowed_states [:idle, :executing]
  end

  command :move_to_target do
    handler MyRobot.Commands.MoveTo
    # Only allowed when perception is healthy
    allowed_states [:idle, :executing]
  end
end
```

`MyRobot.Commands.EnterDegradedState`'s `handle_command/3` returns `{:stop, :normal, %{state | result: {:ok, nil, next_state: :degraded}}}` to flip the robot into `:degraded`. Then `:move_to_target` becomes inadmissible until `on_recovered` flips it back.

### Detect `:lost` without acting on it

Wiring up just `on_lost` (no `on_degraded` or `on_recovered`) is fine — useful when degradation is recoverable in-place but a lost estimator is your "page someone" condition.

### No commands, telemetry only

Omit all three `on_*` slots. Transitions still happen internally and still fire telemetry, but no policy is enforced. Useful during early bring-up when you're observing how an estimator behaves but haven't decided yet what the failure response should be.

## Common gotchas

### `allowed_states` rejection is silent

If `on_lost: :emergency_stop` fires but the robot is in a state where `:emergency_stop` isn't admissible, the command system rejects the dispatch. The estimator's state still moves to `:lost` and `[:bb, :estimator, :transition]` still fires — but `:emergency_stop` doesn't run. Add the relevant states to the command's `allowed_states`, or wire up an intermediate command that's admissible in more states.

### `latency_budget` measures dispatch duration, not message age

`latency_budget` is the time spent inside `handle_input/2`. If the budget is set to `~u(20 millisecond)` and your algorithm takes 25 ms to complete, the transition fires regardless of whether the input arrived "on time". This is intentional — it's the algorithm's response time that matters for downstream consumers. To detect *stale inputs* arriving late, write a `BB.Controller` that monitors `monotonic_time` on the relevant topic.

### `lost_after` is reset on every input, even non-driver

For multi-input estimators the lost timer resets whenever *any* declared input arrives — even ones that aren't the driver. If you want lost detection to depend only on the driver, set `lost_after` only after considering whether a non-driver-only stream should count as "alive enough".

## See also

- [Understanding Estimators](../topics/understanding-estimators.md) — design discussion behind the command-as-policy choice.
- [State Estimation tutorial](../tutorials/13-state-estimation.md) — building the estimator itself.
- [Telemetry Events reference](../reference/telemetry-events.md) — exact event schemas.
