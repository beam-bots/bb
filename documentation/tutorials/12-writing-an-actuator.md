<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Writing an Actuator

In this tutorial, you'll learn how to write a `BB.Actuator` driver for a piece of physical hardware, how the framework feeds it motor-space data, and how it publishes feedback in joint-space. By the end you'll have a working actuator skeleton you can adapt to your own hardware.

## Prerequisites

Complete [Sensors and PubSub](03-sensors-and-pubsub.md) and [Commands and State Machine](05-commands.md). You should already know how to subscribe to messages, send commands with `BB.Actuator.set_position/4`, and understand BB's safety/arming flow.

## The actuator's job

An actuator sits between the rest of the robot — which speaks **joint-space** — and a piece of hardware that has its own ideas about which direction is positive, where zero is, and how many turns of the motor it takes to move the joint a radian. Every actuator driver does three jobs:

1. Drive hardware in response to position, velocity, effort, or trajectory commands.
2. Publish a `BeginMotion` message so the rest of the system knows what motion to expect.
3. (Optionally) Publish `JointState` messages with position feedback from encoders.

Everything coming out of the rest of the robot is in joint-space. Everything the hardware reads or writes is in motor-space. The framework owns the conversion between the two so you can write a driver that knows nothing about gearboxes or sign conventions.

## Joint-space vs. motor-space

Joints have logical positions in the DSL — `~u(-90 degree)` to `~u(90 degree)`, say. The hardware behind the joint may rotate fifty times for one degree of joint motion (a 50:1 reduction), may be physically wired backwards (`reversed?`), and may have its electrical zero somewhere other than the joint's logical zero (`offset`). The `transmission` block on a joint captures all three:

```elixir
joint :shoulder do
  type :revolute

  transmission do
    reduction 50.0
    offset ~u(45 degree)
    reversed? true
  end

  limit do
    lower ~u(-90 degree)
    upper ~u(90 degree)
    velocity ~u(60 degree_per_second)
  end

  actuator :motor, {MyDriver, channel: 0}
end
```

The framework resolves the transmission against the runtime parameter store and uses it for **every** translation between joint-space and motor-space — without the driver having to know it exists.

## The two pipelines

### Inbound: joint-space → motor-space → driver

```
BB.Actuator.set_position(MyRobot, [:shoulder, :motor], 1.57)
       │
       │  message published to [:actuator | path]
       ▼
BB.Actuator.Server  ───►  applies transmission via BB.Transmission.apply_to_command
       │
       │  driver receives motor-space command in its callback
       ▼
MyDriver.handle_cast({:command, motor_space_message}, state)
```

By the time a `Command.Position`, `Command.Velocity`, `Command.Effort`, or `Command.Trajectory` arrives in your driver's callback, every numeric value is already in motor-space. Your driver does no joint-to-motor maths.

### Outbound: driver-space (motor) → BB.Actuator → joint-space subscribers

```
MyDriver.handle_cast(…, state)
       │
       │  builds opts in motor-space, calls
       ▼
BB.Actuator.publish_begin_motion(robot, actuator_path, motor_space_opts)
       │
       │  looks up the joint's transmission and applies
       │  BB.Transmission.unapply_to_payload
       ▼
BB.publish(robot, [:actuator | path], joint_space_message)
```

The driver builds messages in the only coordinate space it understands. The publish helper handles the conversion on the way out.

## The motor profile

You still need to know the limits, the velocity ceiling, and a sensible starting position — in motor-space. The wrapper computes these once at init and injects them as `:motor_profile` in your driver's opts:

```elixir
%BB.Actuator.MotorProfile{
  motor_lower: -78.54,                # motor-space radians
  motor_upper: 78.54,
  motor_velocity_limit: 52.36,        # always a positive magnitude
  motor_acceleration_limit: nil,      # may be nil if the joint has no limit
  motor_effort_limit: nil,
  motor_initial_position: 0.0
}
```

The profile is updated whenever a transmission parameter changes — the wrapper recomputes it and calls your driver's `handle_options/2` callback, so you only need to write the "store the new profile" code once.

## Skeleton driver

Here's a minimal actuator that drives a hypothetical PWM hardware. We'll fill it in piece by piece below.

```elixir
defmodule MyDriver do
  use BB.Actuator,
    options_schema: [
      channel: [type: :pos_integer, doc: "PWM channel", required: true]
    ]

  alias BB.Message
  alias BB.Message.Actuator.Command

  @impl BB.Actuator
  def init(opts) do
    motor_profile = Keyword.fetch!(opts, :motor_profile)
    bb = Keyword.fetch!(opts, :bb)

    state = %{
      bb: bb,
      channel: Keyword.fetch!(opts, :channel),
      motor_profile: motor_profile,
      current_motor_angle: motor_profile.motor_initial_position
    }

    {:ok, state}
  end

  @impl BB.Actuator
  def disarm(opts) do
    MyHardware.disable(Keyword.fetch!(opts, :channel))
    :ok
  end

  @impl BB.Actuator
  def handle_options(new_opts, state) do
    {:ok, %{state | motor_profile: Keyword.fetch!(new_opts, :motor_profile)}}
  end

  @impl BB.Actuator
  def handle_cast({:command, %Message{payload: %Command.Position{} = cmd}}, state) do
    if BB.Safety.armed?(state.bb.robot) do
      do_set_position(cmd, state)
    else
      {:noreply, state}
    end
  end

  defp do_set_position(cmd, state) do
    target = clamp(cmd.position, state.motor_profile)

    MyHardware.write(state.channel, target)

    travel_ms = travel_time_ms(state.current_motor_angle, target, state.motor_profile)
    expected_arrival = System.monotonic_time(:millisecond) + travel_ms

    BB.Actuator.publish_begin_motion(state.bb.robot, state.bb.path,
      initial_position: state.current_motor_angle,
      target_position: target,
      expected_arrival: expected_arrival,
      command_type: :position
    )

    {:noreply, %{state | current_motor_angle: target}}
  end

  defp clamp(angle, %{motor_lower: lower, motor_upper: upper}) do
    angle |> max(lower) |> min(upper)
  end

  defp travel_time_ms(from, to, %{motor_velocity_limit: v}) do
    round(abs(from - to) / v * 1000)
  end
end
```

Notice what isn't in there:

- No call to `BB.Robot.get_joint/2`. The wrapper already looked up the joint.
- No reference to `BB.Transmission`. The wrapper already applied it on the way in, and `publish_begin_motion/3` applies it on the way out.
- No code special-casing `reverse?` or asymmetric joint centres. Both are properties of the transmission, which the wrapper handles.

## Validating the motor profile

The motor profile's fields can be `nil` if the joint doesn't define the corresponding limit. A PWM servo that maps motor limits directly to its pulse-width range *needs* a `motor_lower` and `motor_upper` to function, so it's good practice to validate at init:

```elixir
@impl BB.Actuator
def init(opts) do
  with {:ok, state} <- build_state(opts) do
    {:ok, state}
  else
    {:error, reason} -> {:stop, reason}
  end
end

defp build_state(opts) do
  motor_profile = Keyword.fetch!(opts, :motor_profile)
  bb = Keyword.fetch!(opts, :bb)
  [_, joint_name | _] = Enum.reverse(bb.path)

  with :ok <- validate_motor_profile(motor_profile, joint_name) do
    {:ok, %{ … }}
  end
end

defp validate_motor_profile(%{motor_lower: nil}, joint_name),
  do: {:error, %BB.Error.Invalid.JointConfig{
         joint: joint_name, field: :lower,
         message: "Joint must have a lower limit defined for servo control"
       }}

defp validate_motor_profile(%{motor_upper: nil}, joint_name),
  do: {:error, %BB.Error.Invalid.JointConfig{
         joint: joint_name, field: :upper,
         message: "Joint must have an upper limit defined for servo control"
       }}

defp validate_motor_profile(_, _), do: :ok
```

This subsumes the older pattern of refusing `:continuous` joints — if a joint has no position limits, its motor profile will have `nil` for `motor_lower` and `motor_upper`, and the driver can reject it for one clear reason instead of two overlapping ones.

## Publishing JointState from outside the wrapper

Some hardware reports its actual position via an encoder. Drivers that own this read-back loop usually delegate to a long-running controller process (one bus, many servos). That controller publishes `JointState` messages on its own sensor topic, *not* on the actuator's pubsub path.

The publish-and-forget helper from above only fits the actuator case. For the controller case, `BB.Actuator.to_joint_space/3` does the translation but leaves the publishing to you:

```elixir
defmodule MyController do
  alias BB.Message
  alias BB.Message.Sensor.JointState

  defp publish_position(state, actuator_path, motor_position) do
    joint_name = actuator_path |> Enum.reverse() |> Enum.at(1)

    {:ok, motor_msg} =
      Message.new(JointState, joint_name,
        names: [joint_name],
        positions: [motor_position]
      )

    joint_msg = BB.Actuator.to_joint_space(state.bb.robot, actuator_path, motor_msg)
    BB.publish(state.bb.robot, [:sensor, state.name, joint_name], joint_msg)
  end
end
```

`to_joint_space/3` performs a fresh transmission lookup on each call, so the controller doesn't need to subscribe to parameter changes itself — it always sees the current transmission. If the joint has no transmission, the message is returned unchanged.

## How the wrapper holds the boundary

It's worth being explicit about the invariants:

| Boundary | Direction | Who converts | API |
|---|---|---|---|
| User → driver | joint → motor | `BB.Actuator.Server` (inbound transform) | automatic, via `BB.Transmission.apply_to_command/2` |
| Driver → world | motor → joint | `BB.Actuator.publish_begin_motion/3` | call it from your driver |
| Controller → world | motor → joint | `BB.Actuator.to_joint_space/3` + your `BB.publish` | call from outside the wrapper |
| Joint limits → driver | joint → motor, once | `BB.Actuator.Server` builds `MotorProfile` | read from `:motor_profile` in opts |

Drivers never call `BB.Transmission.apply_*` or `unapply_*` directly. If you find yourself writing those calls in a driver, something has slipped through the abstraction — push the conversion back into the wrapper.

## Testing

Mock the publish helper rather than `BB.publish` itself. Mimic-copy `BB.Actuator`:

```elixir
# test/test_helper.exs
Mimic.copy(BB.Actuator)
```

then stub or expect calls in your tests:

```elixir
BB.Actuator
|> expect(:publish_begin_motion, fn TestRobot, [:shoulder, :motor], opts ->
  assert opts[:initial_position] == 0.0
  assert opts[:target_position] == 0.5
  assert opts[:command_type] == :position
  :ok
end)
```

This stays inside your driver's own coordinate space — the test asserts what the driver tried to publish, not what the wrapper translated it into. Test the translation once, in `bb`; assume it works in every driver.

## Summary

- The wrapper translates commands joint → motor before your driver sees them.
- The wrapper builds a `MotorProfile` from joint limits + transmission and hands it to your driver at init (and again on parameter changes via `handle_options/2`).
- Your driver only ever works in motor-space.
- Publish `BeginMotion` via `BB.Actuator.publish_begin_motion/3` — it does the motor → joint translation for you.
- Controller-style publishers use `BB.Actuator.to_joint_space/3` for the translation and publish to whatever topic they like.
- Validate the motor profile in `init/1`. Don't reach for `BB.Robot` or `BB.Transmission` directly.
