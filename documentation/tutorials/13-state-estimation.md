<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# State Estimation

In this tutorial, you'll write a `BB.Estimator` from scratch — a small sensor-nested filter that consumes raw accelerometer data and republishes it with a fused tilt estimate. By the end you'll understand the behaviour, the DSL entity, and how messages flow through an estimator.

## Prerequisites

Complete [Sensors and PubSub](03-sensors-and-pubsub.md). You should know how a sensor publishes `BB.Message.Sensor.Imu` (or similar) into pubsub and how to subscribe.

## The estimator's job

An estimator consumes one or more input message streams and publishes derived state. The same contract covers two distinct cases:

- **Within-sensor fusion** — combining the modalities of a single sensor. AHRS combining gyro + accelerometer from one IMU into orientation is the canonical case.
- **Cross-sensor fusion** — combining different physical sensors into an estimate of some target frame's state. An EKF blending IMU + wheel odometry into a base pose is the canonical case.

This tutorial focuses on the within-sensor form because it's the simplest place to start. The cross-sensor form is covered in the [understanding-estimators](../topics/understanding-estimators.md) topic.

## The example we'll build

A `TiltSmoother` that nests inside an IMU sensor, consumes its `Imu` messages, and republishes them with the `:orientation` field set to a low-pass tilt estimate computed from the accelerometer. (A real AHRS would also use the gyro — `bb_estimator_ahrs` does — but a single-axis low-pass keeps the example small.)

## Step 1: Declare the estimator in your DSL

`estimator` is a DSL entity that nests inside either a `sensor` (sensor-nested form — what we want) or a `link` (cross-sensor form — for later).

```elixir
defmodule MyRobot.Robot do
  use BB

  topology do
    link :base_link do
      sensor :imu, MyImuSensor, bus: "i2c-1", address: 0x68 do
        estimator :tilt, {TiltSmoother, alpha: 0.95}
      end
    end
  end
end
```

The estimator declaration takes the same shape as `sensor` and `actuator` — a name (`:tilt`) and a child spec (`{TiltSmoother, alpha: 0.95}`).

Because `:tilt` is nested *inside* `sensor :imu`, the framework wires the parent sensor's published messages as its implicit input. No `input` blocks are needed.

## Step 2: Write the callback module

Like sensors and controllers, your module is **not** a GenServer — the framework provides a wrapper (`BB.Estimator.Server`) that delegates to your callbacks. You implement `BB.Estimator`.

```elixir
defmodule TiltSmoother do
  use BB.Estimator,
    options_schema: [
      alpha: [type: :float, default: 0.95, doc: "Low-pass weight"]
    ]

  alias BB.Math.{Quaternion, Vec3}
  alias BB.Message
  alias BB.Message.Sensor.Imu

  @impl BB.Estimator
  def init(opts) do
    {:ok, %{alpha: Keyword.fetch!(opts, :alpha), q: Quaternion.identity()}}
  end

  @impl BB.Estimator
  def handle_input(%Message{payload: %Imu{} = imu} = msg, state) do
    {ax, ay, az} = {Vec3.x(imu.linear_acceleration),
                    Vec3.y(imu.linear_acceleration),
                    Vec3.z(imu.linear_acceleration)}

    # Tilt from gravity (roll about X, pitch about Y), ignoring yaw.
    roll = :math.atan2(ay, az)
    pitch = :math.atan2(-ax, :math.sqrt(ay * ay + az * az))

    q_accel = Quaternion.from_euler(roll, pitch, 0.0)
    q_blended = Quaternion.slerp(state.q, q_accel, 1.0 - state.alpha)

    {:ok, out} =
      Imu.new(msg.frame_id,
        orientation: q_blended,
        angular_velocity: imu.angular_velocity,
        linear_acceleration: imu.linear_acceleration
      )

    {:reply, [out: out], %{state | q: q_blended}}
  end

  def handle_input(_other, state), do: {:noreply, state}
end
```

Two things are worth pausing on.

### The reply shape

Sensors and actuators publish by calling `BB.PubSub.publish/3` directly. Estimators don't — they return outputs from their callback:

```elixir
{:reply, [out: message], new_state}
```

The framework routes each `{output_name, message}` tuple to that output's pubsub path. `:out` is the conventional single-output name and the framework synthesises a path for it automatically — your estimator's own path, in this case `[:sensor, :base_link, :imu, :tilt]`. Multi-output estimators declare each output explicitly with an `output :name` block; this tutorial doesn't need them.

You can return `{:noreply, state}` to consume an input without emitting anything — useful for accumulators that need several updates before producing one output.

### The input shape

For sensor-nested estimators (or link-nested estimators with one declared `input`), `handle_input/2` receives a single `%BB.Message{}` envelope. Multi-input estimators receive a map `%{input_name => message}` keyed by their declared input names. The same callback handles both — pattern-match on whichever shape your estimator expects.

> **For Roboticists:** This is similar in spirit to a ROS node that subscribes to `/imu/data_raw` and republishes on `/imu/data`. The difference is that BB's framework owns subscription, fan-in, dt tracking, and output routing — your module is pure logic.

> **For Elixirists:** Think of `BB.Estimator` as a behaviour that gives you a GenServer with structured input/output semantics — `handle_input/2` is your message handler, the `{:reply, outputs, state}` return shape is the publish side. You can also handle arbitrary messages via `handle_info/2`, `handle_call/3`, `handle_cast/2`, and any of them can return the `{:reply, outputs, state}` shape too — handy for estimators that emit on a timer.

## Step 3: Subscribe to the estimator's output

The estimator publishes to its natural path. For sensor-nested estimators that's the parent sensor's path with the estimator name appended:

```elixir
{:ok, _} = BB.subscribe(MyRobot, [:sensor, :base_link, :imu, :tilt])

# ... in handle_info ...
def handle_info({:bb, _path, %BB.Message{payload: %BB.Message.Sensor.Imu{} = imu}}, state) do
  {roll, pitch, _yaw} = BB.Math.Quaternion.to_euler(imu.orientation)
  IO.puts("roll=#{Float.round(roll, 3)} pitch=#{Float.round(pitch, 3)}")
  {:noreply, state}
end
```

Subscribers to `[:sensor, :base_link, :imu]` still receive the raw sensor output — the estimator publishes alongside, not in place of, the parent sensor.

> **For Roboticists:** The two are siblings in pubsub-space. If a downstream consumer wants raw IMU data and another wants the fused output, both subscribe to the paths they care about.

## Step 4: Test the estimator

Algorithms are easier to test as pure functions. Expose a stateless `step/N` helper so tests can drive it without spinning up a GenServer:

```elixir
defmodule TiltSmoother do
  # ... as before ...

  @doc "Run one step against an `{ax, ay, az}` accel tuple."
  def step(state, {ax, ay, az}) do
    # same body, just refactored out of handle_input
  end
end
```

Then in your tests:

```elixir
defmodule TiltSmootherTest do
  use ExUnit.Case, async: true

  test "stationary gravity along +Z stays near identity" do
    state = %{alpha: 0.9, q: BB.Math.Quaternion.identity()}

    final =
      Enum.reduce(1..100, state, fn _, s ->
        TiltSmoother.step(s, {0.0, 0.0, 9.81})
      end)

    assert_in_delta BB.Math.Quaternion.angular_distance(
                      final.q,
                      BB.Math.Quaternion.identity()
                    ),
                    0.0,
                    1.0e-3
  end
end
```

For integration tests, spin up the whole robot:

```elixir
test "estimator publishes when sensor input arrives" do
  start_supervised!(MyRobot.Robot)
  {:ok, _} = BB.subscribe(MyRobot.Robot, [:sensor, :base_link, :imu, :tilt])

  # Publish a fake IMU message at the parent sensor's path:
  {:ok, msg} = BB.Message.Sensor.Imu.new(:imu,
    orientation: BB.Math.Quaternion.identity(),
    angular_velocity: BB.Math.Vec3.zero(),
    linear_acceleration: BB.Math.Vec3.new(0.0, 0.0, 9.81)
  )

  BB.publish(MyRobot.Robot, [:sensor, :base_link, :imu], msg)

  assert_receive {:bb, [:sensor, :base_link, :imu, :tilt],
                  %BB.Message{payload: %BB.Message.Sensor.Imu{}}}, 500
end
```

## Common gotchas

### Path conventions

- **Sensor-nested estimators** publish on `[:sensor | sensor_path] ++ [estimator_name]`. For the example above that's `[:sensor, :base_link, :imu, :tilt]`.
- **Link-nested estimators** publish on `[:estimator | link_path] ++ [estimator_name]` — the `:estimator` prefix distinguishes them from sensor outputs in subscriptions.

When the verifier rejects an estimator with "input :foo references unknown path …" it usually means a typo in the path or that the referenced sensor lives at a different level of the topology than you think.

### dt tracking

Each incoming message carries `monotonic_time` in nanoseconds. An estimator that needs dt should store the previous message's `monotonic_time` and compute the delta on each `handle_input/2`. The framework doesn't compute dt for you — that decision is per-algorithm (some don't need it, some prefer a fixed-rate approximation).

### Unit conventions

BB uses SI everywhere: rad/s for angular velocity, m/s² for linear acceleration, radians for orientation, metres for translation. Algorithms that traditionally work in other units (e.g. `accel_threshold` expressed as "fraction of 1 g") should convert at the boundary. Don't push unit awareness into sensor drivers — they should publish SI.

## Next steps

- The [understanding-estimators](../topics/understanding-estimators.md) topic explains the design choices behind the behaviour and DSL — single vs multi-input, frame semantics, why the reply shape, where estimators sit in the supervision tree.
- The [configure estimator health](../how-to/configure-estimator-health.md) how-to covers `latency_budget`, `lost_after`, and the `on_degraded`/`on_lost`/`on_recovered` command-as-policy mechanism.
- The [`bb_estimator_ahrs`](https://github.com/beam-bots/bb_estimator_ahrs) sibling package is a real-world example: three IMU fusion algorithms (Madgwick, Mahony, Complementary), each implemented as a `BB.Estimator`. Skim the source for patterns to copy.
