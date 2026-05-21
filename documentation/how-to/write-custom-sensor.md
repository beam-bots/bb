<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# How to Write a Custom Sensor

Build a sensor that publishes data into the BB framework. This guide is task-oriented — for the broader concepts behind sensors and PubSub, see [Sensors and PubSub](../tutorials/03-sensors-and-pubsub.md).

## Prerequisites

- Familiarity with the BB DSL (see [First Robot](../tutorials/01-first-robot.md))
- Comfort with GenServer-style callback modules
- A picture of the hardware (or virtual source) you're reading from

## Three flavours

Sensors in BB can attach at three different levels. The level determines which information the wrapper makes available:

| Attachment | Examples | Has a transmission? |
|---|---|---|
| **Robot-level** (`sensors do … end`) | Battery monitor, ambient temperature, top-level IMU | No |
| **Link-level** (inside a link) | IMU on a specific link, end-of-arm camera | No |
| **Joint-level** (inside a joint) | Encoder, load cell, joint-mounted thermometer | Yes (optional) |

Joint-level sensors are the only kind that participate in transmissions. The framework injects a `:sensor_profile` into their opts containing the resolved transmission, so a sensor reading raw motor-space encoder counts can publish joint-space state without doing the maths itself.

## Step 1: Implement the sensor module

A sensor is a `BB.Sensor` callback module. `BB.Sensor.Server` is the actual GenServer — your module just supplies callbacks. Don't `use GenServer`.

```elixir
defmodule MySensor do
  use BB.Sensor,
    options_schema: [
      pin: [type: :non_neg_integer, required: true, doc: "GPIO pin"],
      poll_interval_ms: [type: :pos_integer, default: 100]
    ]

  alias BB.Message
  alias BB.Message.Sensor.Range

  @impl BB.Sensor
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)

    state = %{
      bb: bb,
      pin: Keyword.fetch!(opts, :pin),
      poll_interval_ms: Keyword.fetch!(opts, :poll_interval_ms),
      last_reading: nil
    }

    schedule_poll(state.poll_interval_ms)
    {:ok, state}
  end

  @impl BB.Sensor
  def handle_info(:poll, state) do
    reading = MyHardware.read(state.pin)

    if reading != state.last_reading do
      publish(reading, state)
    end

    schedule_poll(state.poll_interval_ms)
    {:noreply, %{state | last_reading: reading}}
  end

  defp publish(distance_metres, state) do
    {:ok, msg} =
      Message.new(Range, List.last(state.bb.path),
        range: distance_metres,
        min_range: 0.02,
        max_range: 4.0,
        radiation_type: :ultrasound
      )

    BB.publish(state.bb.robot, [:sensor | state.bb.path], msg)
  end

  defp schedule_poll(ms), do: Process.send_after(self(), :poll, ms)
end
```

The `:bb` key in `opts` contains `%{robot: module, path: [atom]}`. Use `state.bb.robot` (not `state.bb.robot_module` — that key doesn't exist) when calling `BB.publish/3`, `BB.subscribe/2`, etc.

## Step 2: Use it in a robot definition

For a robot-level or link-level sensor, no transmission is involved:

```elixir
defmodule MyRobot.Robot do
  use BB

  sensors do
    sensor :ambient, {MySensor, pin: 18}
  end

  topology do
    link :base_link do
      sensor :front_distance, {MySensor, pin: 19, poll_interval_ms: 50}
    end
  end
end
```

For a joint-attached sensor, see step 3.

## Step 3: Joint-attached sensors with transmissions

A sensor sitting on a joint can declare its own `transmission` block — independent of any actuator on the same joint. Common scenarios:

- A magnetic encoder on the joint output shaft (no reduction, `reduction 1.0`)
- An encoder on the motor shaft, before the gearbox (same `reduction` as the actuator)
- A potentiometer with a polarity flip relative to the actuator

```elixir
joint :shoulder do
  type :revolute

  limit do
    lower(~u(-90 degree))
    upper(~u(90 degree))
    velocity(~u(60 degree_per_second))
    effort(~u(10 newton_meter))
  end

  actuator :motor, {MyServo.Actuator, channel: 0} do
    transmission do
      reduction 50.0
      reversed? true
    end
  end

  sensor :encoder, {MyEncoder, address: 0x6B} do
    transmission do
      reduction 1.0
      # Encoder reads the output shaft directly; no offset/reversal.
    end
  end
end
```

### Reading the sensor profile

`BB.Sensor.Server` injects a `:sensor_profile` into the resolved opts when the sensor is joint-attached:

```elixir
%BB.Sensor.SensorProfile{
  joint_name: :shoulder,
  transmission: %{reduction: 1.0, offset: 0.0, reversed?: false}
}
```

Sensors at the robot or link level get a `sensor_profile` with both fields `nil`.

```elixir
@impl BB.Sensor
def init(opts) do
  sensor_profile = Keyword.fetch!(opts, :sensor_profile)
  bb = Keyword.fetch!(opts, :bb)

  state = %{
    bb: bb,
    sensor_profile: sensor_profile,
    address: Keyword.fetch!(opts, :address)
  }

  {:ok, state}
end

@impl BB.Sensor
def handle_options(new_opts, state) do
  {:ok, %{state | sensor_profile: Keyword.fetch!(new_opts, :sensor_profile)}}
end
```

`handle_options/2` is called when a transmission parameter changes at runtime, so the profile stays current without the sensor having to subscribe to anything itself.

### Publishing joint state

For a sensor that reads a joint position, use `BB.Sensor.publish_joint_state/3`. The driver supplies a position in its own (motor or sensor) coordinate space; the helper applies the transmission and publishes joint-space state to `[:sensor | path]`:

```elixir
defp publish_position(state) do
  raw = MyHardware.read_position(state.address)
  motor_radians = encoder_counts_to_motor_radians(raw)

  BB.Sensor.publish_joint_state(state.bb.robot, state.bb.path,
    positions: [motor_radians]
  )
end
```

Subscribers see joint-space positions; the sensor only ever sees its own coordinate space.

### Publishing other message types

For messages that aren't `JointState`, use `BB.Sensor.to_joint_space/3` to translate then publish wherever you like:

```elixir
defp publish_load(state) do
  motor_torque = MyHardware.read_load(state.address)

  {:ok, motor_msg} =
    Message.new(BB.Message.Sensor.JointState, state.sensor_profile.joint_name,
      names: [state.sensor_profile.joint_name],
      efforts: [motor_torque]
    )

  joint_msg = BB.Sensor.to_joint_space(state.bb.robot, state.bb.path, motor_msg)
  BB.publish(state.bb.robot, [:sensor | state.bb.path], joint_msg)
end
```

`to_joint_space/3` does a fresh transmission resolution on every call. If the sensor isn't joint-attached, or has no transmission block, the message is returned unchanged.

## Step 4: Common message shapes

### `JointState` (joint position feedback)

Use `publish_joint_state/3` for the common single-joint case. For multi-joint states (rare for a single sensor), build the message yourself and publish without `to_joint_space/3`, since the same transmission can't sensibly apply to every joint in the list.

### `Imu` (orientation, angular velocity, linear acceleration)

```elixir
defp publish_imu(state) do
  {q, w, a} = MyHardware.read_imu(state.bus)

  {:ok, msg} =
    Message.new(BB.Message.Sensor.Imu, List.last(state.bb.path),
      orientation: q,
      angular_velocity: w,
      linear_acceleration: a
    )

  BB.publish(state.bb.robot, [:sensor | state.bb.path], msg)
end
```

IMUs are typically link-level — they have no transmission, just an `origin` (in the future, see the [proposals repository](https://github.com/beam-bots/proposals) for `origin` on attachments).

### `BatteryState`

```elixir
defp publish_battery(state) do
  {:ok, msg} =
    Message.new(BB.Message.Sensor.BatteryState, :battery,
      voltage: read_voltage(state),
      current: read_current(state),
      percentage: read_percentage(state)
    )

  BB.publish(state.bb.robot, [:sensor | state.bb.path], msg)
end
```

### `Range`, `LaserScan`, `Image`

All follow the same shape — build a `Message`, then `BB.publish/3` it to `[:sensor | state.bb.path]`. See `lib/bb/message/sensor/` for available types.

## Step 5: Closed-loop control example

The original motivating case for sensor-side transmissions: an open-loop PWM servo plus an independent magnetic encoder, with a PID closing the loop in joint-space.

```elixir
defmodule MyArm.Robot do
  use BB

  parameters do
    group :gains do
      param :kp, type: :float, default: 1.0
    end
  end

  controllers do
    controller :pid_shoulder, {BB.PidController,
      input: [:sensor, :shoulder_encoder],
      output: [:actuator, :shoulder_pwm],
      kp: param([:gains, :kp])
    }
  end

  topology do
    link :base do
      joint :shoulder do
        type :revolute

        limit do
          lower(~u(-90 degree))
          upper(~u(90 degree))
          velocity(~u(60 degree_per_second))
          effort(~u(2 newton_meter))
        end

        # Open-loop PWM servo behind a 100:1 reduction.
        actuator :shoulder_pwm, {BB.Servo.Pigpio.Actuator, pin: 17} do
          transmission do
            reduction 100.0
          end
        end

        # Magnetic encoder reading the output shaft directly.
        sensor :shoulder_encoder, {AS5600.Sensor, address: 0x36} do
          transmission do
            reduction 1.0
          end
        end

        link :upper_arm
      end
    end
  end
end
```

Both the PID's input (the encoder's `JointState`) and its output (the actuator's `Command.Position`) are in joint-space. The encoder driver doesn't know about the actuator's reduction; the actuator doesn't know about the encoder's. Each runs through its own transmission and the PID itself doesn't need to know there's a chain.

## Step 6: Testing

Mimic-copy `BB`, `BB.Sensor`, and any hardware modules:

```elixir
# test/test_helper.exs
Mimic.copy(BB)
Mimic.copy(BB.Sensor)
Mimic.copy(MyHardware)
```

For sensors that publish via `publish_joint_state/3`, stub that helper to assert on the sensor-space opts directly:

```elixir
defmodule MyEncoderTest do
  use ExUnit.Case, async: true
  use Mimic

  alias BB.Sensor.SensorProfile
  alias MyEncoder

  defp sensor_profile do
    %SensorProfile{
      joint_name: :shoulder,
      transmission: %{reduction: 1.0, offset: 0.0, reversed?: false}
    }
  end

  test "publishes joint state in motor-space opts" do
    stub(MyHardware, :read_position, fn _ -> 1234 end)

    expect(BB.Sensor, :publish_joint_state, fn _robot, _path, opts ->
      assert is_list(opts[:positions])
      :ok
    end)

    opts = [
      bb: %{robot: TestRobot, path: [:shoulder, :encoder]},
      address: 0x36,
      sensor_profile: sensor_profile()
    ]

    {:ok, state} = MyEncoder.init(opts)
    MyEncoder.handle_info(:poll, state)
  end
end
```

The transmission lookup itself is tested once, in `bb` — you don't need to retest it in every sensor.

## Safety considerations

For sensors whose loss is safety-critical (e.g. the only encoder on a powered joint), report repeated read failures through `BB.Safety.report_error/3` and then crash. The supervision tree decides whether to escalate:

```elixir
@impl BB.Sensor
def handle_info(:poll, state) do
  case MyHardware.read(state.pin) do
    {:ok, value} ->
      publish(value, state)
      schedule_poll(state.poll_interval_ms)
      {:noreply, %{state | errors: 0}}

    {:error, reason} ->
      new_errors = state.errors + 1

      if new_errors >= 3 do
        BB.Safety.report_error(state.bb.robot, state.bb.path,
          {:sensor_failure, reason})

        {:stop, {:sensor_failure, reason}, state}
      else
        schedule_poll(state.poll_interval_ms)
        {:noreply, %{state | errors: new_errors}}
      end
  end
end
```

`BB.Safety.report_error/3` is a notification only — it publishes a `BB.Safety.HardwareError` event but does not change safety state. Escalation happens through the supervision tree: if your process crashes often enough to exhaust the topology supervisor's restart budget, the safety controller force-disarms the robot.

Sensors that also control hardware (e.g. a spinning LIDAR you can switch off) should implement the optional `disarm/1` callback from the `BB.Sensor` behaviour. When that callback is present, `BB.Sensor.Server` automatically registers the sensor with `BB.Safety`.

## Common pitfalls

### Sensor uses `state.bb.robot_module`

That key doesn't exist. The injected map is `%{robot: module, path: [atom]}` — use `state.bb.robot`.

### Sensor reaches into `BB.Robot.sensors` or `BB.Transmission`

If you find yourself doing manual transmission lookups in a sensor driver, push the maths into the framework instead. Use `:sensor_profile` (injected at init), `publish_joint_state/3`, or `to_joint_space/3`. The driver should never see `BB.Transmission.apply_*`.

### Sensor isn't joint-attached but expects a transmission

Robot-level and link-level sensors get a `sensor_profile` with `joint_name: nil` and `transmission: nil`. Calling `publish_joint_state/3` on a non-joint-attached sensor is allowed — the publish path is unchanged — but the value passes through without transformation, which may not be what you want. If the sensor is supposed to be joint-attached, the bug is in the robot DSL, not the driver.

### Subscribers don't receive messages

Verify the topic. Sensors published via `publish_joint_state/3` (and the `[:sensor | state.bb.path]` pattern in general) appear on a hierarchical topic — subscribers can listen at any ancestor. Check:

- The sensor is started (look at the supervision tree)
- The subscriber's path is an ancestor of the published path
- `message_types:` filtering (if used) matches the published message struct

## Next steps

- [Sensors and PubSub](../tutorials/03-sensors-and-pubsub.md) — concepts behind subscription and message routing.
- [How to Integrate a Servo Driver](integrate-servo-driver.md) — for the actuator side, particularly when pairing a sensor with a closed-loop controller.
- [Writing an Actuator](../tutorials/12-writing-an-actuator.md) — concepts behind motor-space ↔ joint-space conversion, much of which mirrors the sensor side.
