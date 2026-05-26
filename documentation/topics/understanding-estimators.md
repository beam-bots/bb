<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Understanding Estimators

This document explains the design of Beam Bots' state-estimation abstraction — what `BB.Estimator` is for, why it has the shape it has, and how it differs from the closely-related `BB.Sensor` and `BB.Controller`.

## Overview

`BB.Estimator` is a behaviour, parallel to `BB.Sensor` / `BB.Actuator` / `BB.Controller`, for processes that consume one or more input message streams and publish derived state. The same contract covers two distinct problems:

- **Within-sensor fusion** — combining the modalities of a single physical sensor. A 6-DOF IMU has a gyro (fast, drifts) and an accelerometer (drift-free gravity reference, noisy under linear acceleration). Madgwick / Mahony / complementary filters fuse those two channels into a drift-free orientation. Output is in the sensor's own frame.
- **Cross-sensor fusion** — combining different physical sensors into an estimate of some target frame's state. Two co-located IMUs averaged for redundancy. An IMU + wheel odometry running through an EKF for 2D base pose. An IMU + GPS for global pose. Output is in a chosen target frame; inputs must be transformed from their source frames before fusing.

Both problems share a structural shape — declared inputs, declared outputs, a supervised process, telemetry, health transitions — but differ in their reference-frame story and number of inputs. A single behaviour covers both; the DSL distinguishes the two cases.

## Why not extend BB.Controller?

`BB.Controller` is the catch-all "any process that runs alongside the robot" abstraction. Estimators *could* be written as controllers, and historically the `BB.Sensor.OpenLoopPositionEstimator` was. But estimators have a tighter contract:

- Inputs are explicitly declared and resolved against the topology.
- The framework owns subscription and fan-in.
- Outputs are routed automatically based on the entity's placement.
- Health transitions are first-class with hysteresis and command dispatch.
- The reply shape is structured (`{:reply, [{name, message}], state}`) so the user code never touches `BB.PubSub.publish/3`.

That extra structure is too specific to overload onto `BB.Controller` without weakening Controller's general-purpose role. So `BB.Estimator` is its own behaviour with its own DSL entity, server, and supervisor wiring.

## The two DSL forms

The `estimator` DSL keyword has two different schemas depending on where you nest it.

### Sensor-nested

```elixir
sensor :imu, BB.Sensor.Bmi232, ... do
  estimator :orientation, {BB.Ahrs.Madgwick, beta: 0.1}
end
```

- **Frame**: inherited from the parent sensor.
- **Input**: implicit — the parent sensor's published messages.
- **Output path**: `[:sensor, link_name, ..., sensor_name, estimator_name]`.

The verifier rejects `input` blocks here because they'd be meaningless.

### Link-nested

```elixir
link :base_link do
  sensor :imu, BB.Sensor.Bmi232, ...
  sensor :wheels, BB.Sensor.WheelOdom, ...

  estimator :pose, BB.Fusion.Complementary do
    input :imu, [:sensor, :base_link, :imu, :orientation], driver?: true
    input :odom, [:sensor, :base_link, :wheels]
    sync_tolerance ~u(20 millisecond)
  end
end
```

- **Frame**: the parent link's frame.
- **Inputs**: explicitly declared. Multi-input estimators must mark exactly one input as the driver.
- **Output path**: `[:estimator, link_name, ..., estimator_name]`. The `:estimator` prefix distinguishes link-level estimator outputs from sensor outputs.

Same DSL keyword, two entity definitions in the underlying Spark extension. Trying to declare `input` inside a sensor-nested estimator is a compile error before the verifier even runs.

## Frame semantics

The output frame is determined by *where the estimator sits in the topology*, not declared separately. This is the single most important property of the design.

For sensor-nested estimators the output is in the parent sensor's frame, so the algorithm receives samples already in the frame it publishes to — no frame transforms are needed.

For link-nested estimators the output is in the parent link's frame. Inputs have their own frames; the framework provides each input's static transform-to-target-frame at init time via the `BB.Estimator.Context` struct, and the estimator applies them as part of its algorithm. For co-located sensors (same parent link) those transforms are identity; for sensors on different links across a fixed joint the framework precomputes the chain.

Sensors on different links across a *moving* joint (rare in v1 use cases) require dynamic transforms that depend on current joint angles. Algorithms that opt in to this case query `BB.Robot.Kinematics` at message-handling time. The framework does not pre-resolve dynamic transforms — the compile-time verifier rejects cross-sensor estimators whose inputs span moving joints unless the algorithm module declares it handles dynamic transforms.

## Why the reply shape

`handle_input/2` and the other GenServer-style callbacks return `{:reply, outputs, state}` rather than calling `BB.PubSub.publish/3` directly. Three reasons:

1. **Output paths aren't the user's problem.** The framework knows where each output should go (auto-derived from the entity's placement, or from an explicit `output :name, path: ...` block). The user names the output, the framework routes it.
2. **Telemetry hooks at the right boundary.** Latency from input-arrival to output-emission, output counts, payload types — all observable in one place rather than scattered through user code.
3. **Multiple outputs are uniform.** A Kalman filter that emits both a pose and a velocity returns `{:reply, [pose: pose_msg, velocity: vel_msg], state}` — no different in shape from a single-output estimator returning `{:reply, [out: msg], state}`.

`{:reply, [], state}` is legal and useful — accumulators that consume many inputs before producing one output emit nothing on the in-between dispatches. `{:noreply, state}` is also accepted and behaves the same.

The same shape is allowed from `handle_info/2`, `handle_cast/2`, `handle_continue/2`, and `handle_call/3` (the call form is `{:reply, reply, outputs, state}` so the call response and the output list are distinguishable). An AHRS that wants to emit on a fixed-rate timer regardless of input cadence can do so from `handle_info(:tick, state)`.

## Multi-input fan-in

Multi-input estimators need a strategy for "I have a new message on input A — what does my algorithm see for inputs B and C?". The framework provides a deterministic answer:

1. **Driver-triggered dispatch.** Exactly one input is marked `driver?: true`. The driver's arrival triggers `handle_input/2`.
2. **Last-known fan-in.** Non-driver inputs are sourced from a per-input "last known" cache the server maintains. On each driver arrival, the server snapshots that cache and builds a `%{input_name => message}` map.
3. **Sync tolerance.** If any non-driver input's `monotonic_time` is older than the driver's by more than `sync_tolerance`, the dispatch is dropped instead of fired with a stale snapshot. The framework emits `[:bb, :estimator, :dropped]` telemetry with reason `:sync_miss`.

The driver choice is a policy decision. Pick the input with the most reliable cadence (an IMU sampling at 200 Hz is a better driver than an odom topic that may stall when wheels stop). The non-driver inputs are then implicitly interpolated by "use the most recent reading you have" — a deliberately simple choice. Algorithms that need cleverer interpolation (linear interpolation between samples bracketing the driver's timestamp, for example) can implement that themselves on top of the raw inputs.

## Health as commands

The proposal explicitly avoided introducing a structured "health" payload or a separate health-monitoring process. Instead, three configurable transition commands fire on hysteresis-debounced state changes:

- `on_degraded` — fires when the estimator transitions from `:healthy` to `:degraded` (latency overrun, sync miss, stale input, algorithm-reported divergence).
- `on_lost` — fires when no input arrives within `lost_after`.
- `on_recovered` — fires after `recover_after` consecutive in-budget completions return the estimator to `:healthy`.

This shape is deliberate. Two reasons:

### Developer-defined policy

What to do when an estimator degrades is robot-specific. Some robots should stop. Some should switch to a slower control loop. Some should emit a status message and continue. Hard-coding any of these into the framework is wrong; surfacing the transition as a command lets the developer encode their policy using existing BB primitives — the same `BB.Command` machinery used for everything else.

### Existing state-machine integration

Commands already integrate with the robot's state machine via `allowed_states`. Want certain operations blocked during degraded perception? Have `on_degraded` invoke a command that transitions the state machine into a `:degraded` state, and configure other commands' `allowed_states` accordingly. No new mechanism required — the framework reuses what's already there.

Transitions still emit telemetry (`[:bb, :estimator, :transition]`) whether or not a command is configured, so observability is independent of policy.

The configured command receives the transition context as its args:

```elixir
%{
  estimator: :pose,
  reason: :latency_overrun | :sync_miss | :lost | :recovered,
  source_path: [atom] | nil,
  previous_state: :healthy | :degraded | :lost,
  new_state: :healthy | :degraded | :lost
}
```

If the command's `allowed_states` rejects the dispatch (e.g. `on_lost` tries to fire `:emergency_stop` but the robot is in a state that disallows it), the transition still happens internally and the telemetry still fires — but the command doesn't run. Estimator transitions can fail to dispatch; that's a policy concern handled by the state machine, not a framework error.

## Where estimators sit in supervision

Sensor-nested estimators are started by the same supervisor as the parent sensor — the link supervisor (for link-attached sensors), the joint supervisor (for joint-attached sensors), or the robot-level sensor supervisor (for `sensors do … end` declarations). They appear as siblings to the parent sensor in the supervision tree, not as children.

Link-nested estimators are started by the link supervisor.

This means an estimator crash isolates the same way a sensor crash isolates: the supervisor restarts it, the rest of the robot keeps running. An estimator whose algorithm encounters something pathological (NaN in the input, for example) can `{:stop, reason, state}` and OTP will bring it back to a clean initial state.

## What estimators are not

A few things `BB.Estimator` deliberately does *not* try to be:

- **A SLAM front-end.** SLAM has its own scope (loop closure, map management, place recognition) that doesn't fit a "consume streams, emit derived state" shape. A future `bb_slam` package could compose estimators as building blocks but would carry its own contract on top.
- **A trajectory optimiser.** Trajectory generation is the `BB.Motion` / `BB.IK.Solver` story; estimators feed motion planning but aren't part of it.
- **A controller.** An estimator publishes derived state; a controller consumes state (and possibly commands) to drive actuators. Many "estimator → controller" chains are natural in BB topologies — the abstractions stay separate so each can evolve independently.

## See also

- The [State Estimation tutorial](../tutorials/13-state-estimation.md) walks through building one from scratch.
- The [Configure Estimator Health how-to](../how-to/configure-estimator-health.md) is the recipe for hooking up `latency_budget` / `lost_after` / `on_*` commands.
- The [`bb_ahrs`](https://github.com/beam-bots/bb_ahrs) package ports three IMU fusion algorithms onto `BB.Estimator` and is the largest worked example.
- [Proposal 0018](https://github.com/beam-bots/proposals/blob/main/accepted/0018-bb-estimator.md) documents the design discussion that led to the current shape.
