<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# How to Integrate a Servo Driver

Create a servo driver package that integrates with Beam Bots' supervision tree and message system.

## Prerequisites

- Familiarity with the BB DSL (see [First Robot](../tutorials/01-first-robot.md))
- Understanding of GenServer
- Access to your servo's communication protocol (I2C, serial, SPI, etc.)

## Overview

A servo driver package consists of two main components:

1. **Controller** - A GenServer managing hardware communication (shared by multiple actuators)
2. **Actuator** - A GenServer per joint that converts position commands to hardware signals

```
Controller (GenServer)
    |
    v wraps
Hardware Driver (I2C/Serial/SPI)
    ^
    | used by
Actuator (GenServer) --publishes--> BeginMotion --> OpenLoopPositionEstimator
                                                        |
                                                        v publishes
                                                    JointState
```

## Step 1: Create the Package

Create a new Elixir package with `bb` as a dependency:

```elixir
# mix.exs
defp deps do
  [
    {:bb, "~> 0.12"}
  ]
end
```

## Step 2: Implement the Controller

The controller manages the hardware connection. Multiple actuators share one controller.

```elixir
defmodule MyServo.Controller do
  use GenServer, restart: :permanent

  @moduledoc """
  Manages hardware communication for MyServo devices.
  """

  @schema [
    bus: [
      type: :string,
      required: true,
      doc: "Hardware bus identifier (e.g., \"i2c-1\", \"/dev/ttyUSB0\")"
    ],
    address: [
      type: :integer,
      required: false,
      doc: "Device address (for I2C devices)"
    ]
  ]

  def schema, do: @schema

  def start_link(init_arg) do
    opts = Spark.Options.validate!(init_arg, @schema)
    GenServer.start_link(__MODULE__, opts)
  end

  @impl GenServer
  def init(opts) do
    case connect_to_hardware(opts) do
      {:ok, device} ->
        {:ok, %{device: device, opts: opts}}

      {:error, reason} ->
        {:stop, reason}
    end
  end

  @doc """
  Set servo position. Called by Actuator processes.
  """
  def set_position(controller, channel, pulse_width) do
    GenServer.call(controller, {:set_position, channel, pulse_width})
  end

  @impl GenServer
  def handle_call({:set_position, channel, pulse_width}, _from, state) do
    result = write_to_hardware(state.device, channel, pulse_width)
    {:reply, result, state}
  end

  defp connect_to_hardware(opts) do
    # Implement your hardware connection logic
    # Return {:ok, device} or {:error, reason}
  end

  defp write_to_hardware(device, channel, pulse_width) do
    # Implement your hardware write logic
    # Return :ok or {:error, reason}
  end
end
```

## Step 3: Implement the Actuator

The actuator receives position commands (in radians), converts them to hardware values, and publishes motion events.

```elixir
defmodule MyServo.Actuator do
  use GenServer, restart: :permanent
  use BB.Actuator

  alias BB.Message.Actuator.BeginMotion
  alias BB.Message.Sensor.JointCommand

  @schema [
    channel: [
      type: :non_neg_integer,
      required: true,
      doc: "Servo channel (0-15)"
    ],
    controller: [
      type: :atom,
      required: true,
      doc: "Name of the controller process"
    ],
    min_pulse: [
      type: :non_neg_integer,
      default: 500,
      doc: "Minimum pulse width in microseconds"
    ],
    max_pulse: [
      type: :non_neg_integer,
      default: 2500,
      doc: "Maximum pulse width in microseconds"
    ]
  ]

  def schema, do: @schema

  def start_link(init_arg) do
    {bb, init_arg} = Keyword.pop!(init_arg, :bb)
    opts = Spark.Options.validate!(init_arg, @schema)
    GenServer.start_link(__MODULE__, {bb, opts})
  end

  @impl GenServer
  def init({bb, opts}) do
    # Get joint limits from the robot topology
    joint = BB.Robot.joint(bb.robot, bb.path)

    # Register with the safety controller
    BB.Safety.register(__MODULE__,
      robot: bb.robot_module,
      path: bb.path,
      opts: [channel: opts[:channel], controller: opts[:controller]]
    )

    {:ok, %{
      bb: bb,
      opts: opts,
      joint: joint,
      current_position: 0.0
    }}
  end

  # Handle position commands via PubSub (BB.Actuator.set_position/4)
  @impl GenServer
  def handle_info({:bb, _path, %{payload: %JointCommand{} = cmd}}, state) do
    {:noreply, execute_move(cmd.target, state)}
  end

  def handle_info(_msg, state), do: {:noreply, state}

  # Handle direct commands (BB.Actuator.set_position!/4)
  @impl GenServer
  def handle_cast({:set_position, position}, state) do
    {:noreply, execute_move(position, state)}
  end

  # Handle synchronous commands (BB.Actuator.set_position_sync/5)
  @impl GenServer
  def handle_call({:set_position, position}, _from, state) do
    {:reply, {:ok, :accepted}, execute_move(position, state)}
  end

  defp execute_move(target_position, state) do
    # Convert radians to pulse width
    pulse = position_to_pulse(target_position, state)

    # Get the controller process
    {:ok, controller} = BB.Process.whereis(state.bb.robot_module, state.opts[:controller])

    # Send to hardware
    :ok = MyServo.Controller.set_position(controller, state.opts[:channel], pulse)

    # Publish BeginMotion for position estimation
    velocity = state.joint.limit.velocity
    motion = BeginMotion.new!(
      name: state.joint.name,
      initial: state.current_position,
      target: target_position,
      velocity: velocity
    )
    BB.publish(state.bb.robot_module, state.bb.path, motion)

    %{state | current_position: target_position}
  end

  defp position_to_pulse(position, state) do
    # Map position (radians) to pulse width (microseconds)
    lower = state.joint.limit.lower
    upper = state.joint.limit.upper
    range = upper - lower

    normalised = (position - lower) / range
    pulse_range = state.opts[:max_pulse] - state.opts[:min_pulse]

    round(state.opts[:min_pulse] + normalised * pulse_range)
  end

  # Safety callback - must work without GenServer state
  @impl BB.Actuator
  def disarm(opts) do
    # Set servo to neutral position
    {:ok, controller} = BB.Process.whereis(opts[:robot], opts[:controller])
    MyServo.Controller.set_position(controller, opts[:channel], 1500)
  end
end
```

## Step 4: Use in Robot Definition

Wire up the controller and actuator in your robot's DSL:

```elixir
defmodule MyRobot do
  use BB

  controllers do
    controller :my_servo, {MyServo.Controller, bus: "i2c-1", address: 0x40}
  end

  topology do
    link :base do
      joint :shoulder, type: :revolute do
        limit lower: ~u(-90 degree), upper: ~u(90 degree), velocity: ~u(60 degree_per_second)

        actuator :servo, {MyServo.Actuator, channel: 0, controller: :my_servo}
        sensor :feedback, {BB.Sensor.OpenLoopPositionEstimator, actuator: :servo}
      end
    end
  end
end
```

## Step 5: Test the Integration

Create tests using Mimic to mock hardware interactions:

```elixir
defmodule MyServo.ActuatorTest do
  use ExUnit.Case
  use Mimic

  setup :verify_on_exit!

  test "converts position to pulse width" do
    # Mock hardware interactions
    expect(MyServo.Controller, :set_position, fn _controller, 0, pulse ->
      assert pulse >= 500 and pulse <= 2500
      :ok
    end)

    # Start robot in simulation
    {:ok, _} = BB.Supervisor.start_link(MyRobot, simulation: :kinematic)

    # Send position command
    BB.Actuator.set_position!(MyRobot, :servo, 0.0)
  end
end
```

## Common Issues

### Actuator not receiving commands

Ensure the actuator is subscribed to its command path. The `BB.Actuator` behaviour handles this, but check that:
- The actuator's `:bb` option contains the correct path
- The controller is registered with the same name used in the actuator config

### Position estimation drift

If `OpenLoopPositionEstimator` shows incorrect positions:
- Verify the velocity in `BeginMotion` matches the actual servo speed
- Check that joint limits in the DSL match the physical servo range

### Safety disarm not working

The `disarm/1` callback must work without GenServer state:
- Don't rely on `self()` or process state
- Use only the options passed to `BB.Safety.register/2`
- Test disarm after crashing the actuator process

## Next Steps

- Implement `BB.Safety` callbacks properly (see [Implement Safety Callbacks](implement-safety-callbacks.md))
- Add support for reading servo position if your hardware supports it
- Consider implementing velocity control for smoother motion
