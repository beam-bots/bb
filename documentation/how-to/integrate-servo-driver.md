<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# How to Integrate a Servo Driver

Build a package that drives a servo (or class of servos) through Beam Bots' actuator behaviour. This guide is task-oriented — for the concepts behind the API, see [Writing an Actuator](../tutorials/12-writing-an-actuator.md).

## Prerequisites

- Familiarity with the BB DSL (see [First Robot](../tutorials/01-first-robot.md))
- Comfort with GenServer-style callback modules
- Documentation for your servo's communication protocol

## Pick a shape

Two patterns appear in the existing driver packages. Choose whichever fits your hardware:

| Shape | When to use it | Examples |
|---|---|---|
| **Standalone actuator** | One process per joint, no shared bus state. Typical for PWM servos. | `bb_servo_pca9685`, `bb_servo_pigpio` |
| **Controller + actuator** | Shared serial bus, one controller process for many servos. | `bb_servo_feetech`, `bb_servo_robotis` |

The two shapes share the same actuator-side patterns; the controller-plus-actuator shape adds a separate `BB.Controller` process that talks to the bus and exchanges per-servo data with the actuators via ETS or messages.

## Step 1: Create the package

```elixir
# mix.exs
defp deps do
  [
    {:bb, "~> 0.17"}
  ]
end
```

## Step 2: Implement the actuator

The actuator is a `BB.Actuator` callback module. `BB.Actuator.Server` is the actual `GenServer` — your module just supplies callbacks. Don't `use GenServer`.

```elixir
defmodule MyServo.Actuator do
  use BB.Actuator,
    options_schema: [
      channel: [type: :pos_integer, required: true, doc: "Hardware channel"],
      controller: [
        type: :atom,
        required: false,
        doc: "Controller process name (only for shared-bus drivers)"
      ]
    ]

  alias BB.Error.Invalid.JointConfig, as: JointConfigError
  alias BB.Message
  alias BB.Message.Actuator.Command
  alias BB.Process, as: BBProcess

  @impl BB.Actuator
  def init(opts) do
    motor_profile = Keyword.fetch!(opts, :motor_profile)
    bb = Keyword.fetch!(opts, :bb)
    [name, joint_name | _] = Enum.reverse(bb.path)

    with :ok <- validate_motor_profile(motor_profile, joint_name) do
      state = %{
        bb: bb,
        name: name,
        joint_name: joint_name,
        channel: Keyword.fetch!(opts, :channel),
        motor_profile: motor_profile,
        current_motor_angle: motor_profile.motor_initial_position
      }

      {:ok, state}
    else
      {:error, reason} -> {:stop, reason}
    end
  end

  @impl BB.Actuator
  def disarm(opts) do
    # Must work without GenServer state — receives the opts you registered
    # with at init, not the live state.
    MyServo.Hardware.disable(Keyword.fetch!(opts, :channel))
    :ok
  end

  @impl BB.Actuator
  def handle_options(new_opts, state) do
    motor_profile = Keyword.fetch!(new_opts, :motor_profile)
    {:ok, %{state | motor_profile: motor_profile}}
  end

  @impl BB.Actuator
  def handle_cast({:command, %Message{payload: %Command.Position{} = cmd}}, state) do
    if BB.Safety.armed?(state.bb.robot) do
      do_set_position(cmd, state)
    else
      {:noreply, state}
    end
  end

  @impl BB.Actuator
  def handle_info({:bb, _path, %Message{payload: %Command.Position{} = cmd}}, state) do
    if BB.Safety.armed?(state.bb.robot) do
      do_set_position(cmd, state)
    else
      {:noreply, state}
    end
  end

  defp do_set_position(%Command.Position{} = cmd, state) do
    target = clamp(cmd.position, state.motor_profile)
    MyServo.Hardware.write(state.channel, target)

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

  defp validate_motor_profile(%{motor_lower: nil}, joint),
    do: {:error, %JointConfigError{joint: joint, field: :lower}}

  defp validate_motor_profile(%{motor_upper: nil}, joint),
    do: {:error, %JointConfigError{joint: joint, field: :upper}}

  defp validate_motor_profile(%{motor_velocity_limit: nil}, joint),
    do: {:error, %JointConfigError{joint: joint, field: :velocity}}

  defp validate_motor_profile(_, _), do: :ok
end
```

Key things to notice:

- `motor_profile` arrives in `opts` already. The wrapper has applied the joint's transmission to the joint's limits and given you motor-space values directly. Don't fetch the joint or call `BB.Transmission` yourself.
- Position commands arriving in your `handle_*` callbacks are already in motor-space. The wrapper handles the inbound conversion.
- Outgoing `BeginMotion` is published with motor-space values; `publish_begin_motion/3` converts to joint-space.
- `handle_options/2` is called whenever a transmission parameter changes, so the motor profile stays current.

## Step 3: (Optional) Implement the controller

Only needed for shared-bus drivers. The controller is a `BB.Controller` callback module that owns the hardware connection and serves multiple actuators.

```elixir
defmodule MyServo.Controller do
  use BB.Controller,
    options_schema: [
      port: [type: :string, required: true],
      baud_rate: [type: :pos_integer, default: 1_000_000]
    ]

  @impl BB.Controller
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)

    case MyServo.Bus.start_link(opts) do
      {:ok, bus} ->
        servo_table = :ets.new(:servos, [:set, :public])
        state = %{bb: bb, bus: bus, servo_table: servo_table, servo_ids: []}

        BB.Safety.register(__MODULE__,
          robot: bb.robot,
          path: bb.path,
          opts: [bus: bus, servo_ids: []]
        )

        {:ok, state}

      {:error, reason} ->
        {:stop, reason}
    end
  end

  @impl BB.Controller
  def disarm(opts) do
    bus = Keyword.fetch!(opts, :bus)
    servo_ids = Keyword.get(opts, :servo_ids, [])
    MyServo.Bus.disable_all(bus, servo_ids)
    :ok
  end

  @impl BB.Controller
  def handle_call({:register_servo, servo_id, actuator_path}, _from, state) do
    :ets.insert(state.servo_table, {servo_id, actuator_path, nil})
    servo_ids = [servo_id | state.servo_ids] |> Enum.uniq()
    {:reply, {:ok, state.servo_table}, %{state | servo_ids: servo_ids}}
  end
end
```

The actuator registers itself with the controller at init and writes goal positions to the ETS table; the controller's polling loop reads them and flushes to the bus.

### Publishing joint feedback from the controller

If your controller reads encoder positions and publishes `JointState` on behalf of its actuators, use `BB.Actuator.to_joint_space/3` to convert motor-space readings before publishing on a sensor topic:

```elixir
defp publish_position(state, actuator_path, motor_position) do
  joint_name = actuator_path |> Enum.reverse() |> Enum.at(1)

  {:ok, motor_msg} =
    Message.new(BB.Message.Sensor.JointState, joint_name,
      names: [joint_name],
      positions: [motor_position]
    )

  joint_msg = BB.Actuator.to_joint_space(state.bb.robot, actuator_path, motor_msg)
  BB.publish(state.bb.robot, [:sensor, state.name, joint_name], joint_msg)
end
```

`to_joint_space/3` does a fresh transmission resolution on every call, so you don't need to subscribe to parameter changes from the controller process — it always uses the current transmission.

Store the actuator path (not just the joint name) when the actuator registers, so the controller can pass it to `to_joint_space/3`.

## Step 4: Use it in a robot definition

The user's robot module wires up your controller and actuator, and declares any transmission on the actuator itself:

```elixir
defmodule MyRobot.Robot do
  use BB

  controllers do
    controller :my_servo_bus, {MyServo.Controller, port: "/dev/ttyUSB0"}
  end

  topology do
    link :base_link do
      joint :shoulder do
        type :revolute

        limit do
          lower(~u(-90 degree))
          upper(~u(90 degree))
          velocity(~u(60 degree_per_second))
          effort(~u(10 newton_meter))
        end

        actuator :shoulder_servo, {MyServo.Actuator, channel: 1, controller: :my_servo_bus} do
          transmission do
            reduction 50.0
            reversed? true
          end
        end

        link :upper_arm
      end
    end
  end
end
```

The `transmission` block lives inside the actuator's `do/end`, not on the joint. The wrapper reads it, builds the motor profile from the joint limits + this transmission, and hands it to your driver at init.

## Step 5: Provide an upgrader for the `reverse?` migration

If your driver was previously released with a `reverse?` option that's now subsumed by the transmission, ship a `mix my_servo.upgrade` task that lifts the option into a `transmission` block. The shared `BB.Igniter.Transmission` helper does the work:

```elixir
defmodule Mix.Tasks.MyServo.Upgrade do
  use Igniter.Mix.Task

  @impl Igniter.Mix.Task
  def info(_argv, _parent), do: %Igniter.Mix.Task.Info{}

  @impl Igniter.Mix.Task
  def igniter(igniter) do
    BB.Igniter.Transmission.lift_reverse_question(
      igniter,
      MyServo.Actuator,
      lift_offset?: true
    )
  end
end
```

`lift_offset?: true` also derives an `offset` from the joint's `(lower + upper) / 2` to preserve any implicit centring the driver used to do. Set it to `false` if your driver never auto-centred.

## Step 6: Test the integration

Mock the publish helper rather than `BB.publish`, so your tests assert on motor-space opts directly:

```elixir
# test/test_helper.exs
Mimic.copy(BB.Actuator)
Mimic.copy(BB.Safety)
Mimic.copy(MyServo.Hardware)

# in your actuator test
defmodule MyServo.ActuatorTest do
  use ExUnit.Case, async: true
  use Mimic

  alias BB.Actuator.MotorProfile
  alias BB.Message
  alias BB.Message.Actuator.Command
  alias MyServo.Actuator

  defp motor_profile(overrides \\ []) do
    base = %MotorProfile{
      motor_lower: -:math.pi() / 2,
      motor_upper: :math.pi() / 2,
      motor_velocity_limit: 1.0,
      motor_initial_position: 0.0
    }

    struct!(base, overrides)
  end

  test "publishes BeginMotion with motor-space values" do
    stub(BB.Safety, :armed?, fn _ -> true end)
    stub(MyServo.Hardware, :write, fn _, _ -> :ok end)

    expect(BB.Actuator, :publish_begin_motion, fn _robot, _path, opts ->
      assert opts[:target_position] == 0.5
      assert opts[:command_type] == :position
      :ok
    end)

    opts = [
      bb: %{robot: TestRobot, path: [:shoulder, :servo]},
      channel: 1,
      motor_profile: motor_profile()
    ]

    {:ok, state} = Actuator.init(opts)
    msg = %Message{payload: %Command.Position{position: 0.5}}
    Actuator.handle_cast({:command, msg}, state)
  end
end
```

You don't need to construct a real robot to test the driver — the motor profile is just a struct, and stubbing `BB.Actuator.publish_begin_motion/3` keeps the test scoped to your driver's own coordinate space.

## Common pitfalls

### The driver knows about `BB.Transmission` or `BB.Robot.get_joint`

If you find yourself calling `BB.Transmission.apply_*` or `BB.Robot.get_joint`, something has slipped through the abstraction. The wrapper handles inbound transformation; `publish_begin_motion/3` and `to_joint_space/3` handle outbound. The driver should never see the transmission object itself.

### `disarm/1` references GenServer state

It can't — `disarm/1` is called *outside* the actuator process, often when the process has crashed. Use only the opts you passed to `BB.Safety.register/2`:

```elixir
@impl BB.Actuator
def disarm(opts) do
  MyServo.Hardware.disable(Keyword.fetch!(opts, :channel))
end
```

Test by killing the process and verifying the hardware is left in a safe state.

### `BeginMotion`'s `expected_arrival` is wildly wrong

The `OpenLoopPositionEstimator` uses this to interpolate position over time. Get it right by computing `motor_velocity_limit` from the joint limits + transmission (which the wrapper already does via `motor_profile`):

```elixir
travel_ms = round(abs(from - to) / state.motor_profile.motor_velocity_limit * 1000)
```

If the simulated arm overshoots or lags in `bb_liveview`, this calculation is the usual suspect.

## Next steps

- Read [Writing an Actuator](../tutorials/12-writing-an-actuator.md) for the design rationale.
- Read [How to Write a Custom Sensor](write-custom-sensor.md) if your driver provides closed-loop feedback through a separate sensor process.
- Look at the existing servo packages for full-fledged examples:
  - `bb_servo_feetech` — controller + actuator, Feetech bus protocol.
  - `bb_servo_robotis` — controller + actuator, Dynamixel protocol.
  - `bb_servo_pca9685` — standalone actuator over I²C PWM.
  - `bb_servo_pigpio` — standalone actuator over the pigpio daemon.
