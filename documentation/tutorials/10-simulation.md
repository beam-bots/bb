<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Simulation Mode

Beam Bots supports running robots in simulation mode, allowing you to develop and test robot behaviour without physical hardware. A single robot definition works for both hardware and simulation - you just change how you start it.

## Prerequisites

Complete [Your First Robot](01-first-robot.md) and [Starting and Stopping](02-starting-and-stopping.md) first.

## Starting in Simulation Mode

Start your robot in kinematic simulation mode by passing the `simulation` option:

```elixir
iex> {:ok, pid} = MyRobot.start_link(simulation: :kinematic)
{:ok, #PID<0.234.0>}
```

The robot is now running entirely in software. Actuators receive commands and publish motion messages, but no hardware communication occurs.

## Checking Simulation Mode

You can check whether a robot is running in simulation mode:

```elixir
iex> BB.Robot.Runtime.simulation_mode(MyRobot)
:kinematic

# Hardware mode returns nil
iex> BB.Robot.Runtime.simulation_mode(MyRobot)
nil
```

## How Simulation Works

In simulation mode:

1. **Actuators are replaced** - Real actuator modules are swapped for `BB.Sim.Actuator`
2. **Controllers are omitted** - By default, hardware controllers don't start
3. **Messages flow normally** - Commands, `BeginMotion`, and `JointState` messages work as usual
4. **Safety system is active** - You must still arm the robot before sending commands

The simulated actuator:

- Receives position commands via the normal API
- Calculates motion timing from joint velocity limits in your DSL
- Publishes `BeginMotion` messages with realistic timing
- Clamps positions to joint limits

The existing `OpenLoopPositionEstimator` sensor works unchanged, estimating position from `BeginMotion` messages.

## Example: Testing Motion

```elixir
# Start in simulation
{:ok, _pid} = MyRobot.start_link(simulation: :kinematic)

# Arm the robot (required even in simulation)
:ok = BB.Safety.arm(MyRobot)

# Send a position command
BB.Actuator.set_position!(MyRobot, :shoulder_motor, 1.57)

# The OpenLoopPositionEstimator will estimate position over time
Process.sleep(500)
position = BB.Robot.Runtime.joint_position(MyRobot, :shoulder)
```

## Controller Behaviour in Simulation

By default, controllers are omitted in simulation mode. You can customise this per-controller using the `simulation` option in the DSL:

```elixir
controllers do
  # Won't start in simulation (default)
  controller :pca9685, {BB.Servo.PCA9685.Controller, bus: "i2c-1"},
    simulation: :omit

  # Starts a mock controller that accepts but ignores commands
  controller :dynamixel, {BB.Servo.Robotis.Controller, port: "/dev/ttyUSB0"},
    simulation: :mock

  # Starts the real controller (for external simulator integration)
  controller :gazebo_bridge, {MyApp.GazeboBridge, url: "localhost:11345"},
    simulation: :start
end
```

The three options are:

| Option | Behaviour |
|--------|-----------|
| `:omit` | Controller not started (default) |
| `:mock` | Mock controller started - accepts commands but does nothing |
| `:start` | Real controller started |

### When to Use Each Option

- **`:omit`** - Most hardware controllers (I2C, serial, GPIO). The simulated actuator doesn't need them.
- **`:mock`** - When actuators query the controller for state during initialisation.
- **`:start`** - For external simulator bridges (Gazebo, MuJoCo) that need to run in simulation.

## Bridge Behaviour in Simulation

Parameter bridges also support the `simulation` option, with the same three modes:

```elixir
parameters do
  # Won't start in simulation (default)
  bridge :mavlink, {BBMavLink.ParameterBridge, conn: "/dev/ttyACM0"},
    simulation: :omit

  # Starts a mock bridge that accepts but ignores operations
  bridge :gcs, {MyApp.GCSBridge, url: "ws://gcs.local/socket"},
    simulation: :mock

  # Starts the real bridge (for external system integration)
  bridge :phoenix, {BBPhoenix.ParameterBridge, url: "ws://localhost:4000/socket"},
    simulation: :start
end
```

| Option | Behaviour |
|--------|-----------|
| `:omit` | Bridge not started (default) |
| `:mock` | Mock bridge started - accepts operations but does nothing |
| `:start` | Real bridge started |

## Kinematic Simulation

The `:kinematic` simulation mode provides position/velocity interpolation without physics:

- Positions are clamped to joint limits (`lower`, `upper`)
- Travel time is calculated from velocity limits
- No acceleration, inertia, or gravity simulation

This is sufficient for:

- Testing control logic and state machines
- Verifying command sequences
- UI development without hardware
- Integration testing

## Future Simulation Modes

The simulation option is an atom to allow future expansion:

```elixir
# Current: kinematic simulation
MyRobot.start_link(simulation: :kinematic)

# Future: external physics engine
MyRobot.start_link(simulation: :external)

# Future: built-in physics
MyRobot.start_link(simulation: :physics)
```

## Environment-Based Mode Selection

To switch between hardware and simulation based on environment:

```elixir
# In your application.ex
defmodule MyApp.Application do
  use Application

  @impl true
  def start(_type, _args) do
    simulation_mode =
      if Application.get_env(:my_app, :simulate, false) do
        :kinematic
      else
        nil
      end

    children = [
      {MyRobot, simulation: simulation_mode}
    ]

    Supervisor.start_link(children, strategy: :one_for_one)
  end
end
```

Then in your config:

```elixir
# config/dev.exs
config :my_app, simulate: true

# config/prod.exs (or target.exs for Nerves)
config :my_app, simulate: false
```

## Testing with Simulation

Simulation mode is useful for integration tests:

```elixir
defmodule MyRobotTest do
  use ExUnit.Case

  test "robot moves to home position" do
    {:ok, pid} = MyRobot.start_link(simulation: :kinematic)

    :ok = BB.Safety.arm(MyRobot)
    :ok = BB.Command.execute(MyRobot, :home)

    # Verify the robot reached home position
    assert_eventually fn ->
      pos = BB.Robot.Runtime.joint_position(MyRobot, :shoulder)
      abs(pos - 0.0) < 0.01
    end

    Supervisor.stop(pid)
  end
end
```

## Subscribing to Simulated Motion

You can subscribe to motion messages from simulated actuators:

```elixir
# Subscribe to actuator messages
BB.PubSub.subscribe(MyRobot, [:actuator, :base, :shoulder, :motor])

# Send a command
BB.Actuator.set_position!(MyRobot, :motor, 1.0)

# Receive the BeginMotion message
receive do
  {:bb, _path, %BB.Message{payload: %BB.Message.Actuator.BeginMotion{} = motion}} ->
    IO.puts("Moving from #{motion.initial_position} to #{motion.target_position}")
    IO.puts("Expected arrival: #{motion.expected_arrival}ms")
end
```

## Limitations

Kinematic simulation doesn't model:

- Physics (gravity, inertia, friction, collisions)
- Sensor noise or latency
- Hardware-specific behaviour
- External disturbances

For high-fidelity simulation, consider integrating with an external physics engine like Gazebo or MuJoCo using `simulation: :start` controllers.

## What's Next?

You now know how to:

- Run robots in simulation mode
- Configure controller behaviour in simulation
- Use simulation for development and testing

For more advanced topics, see:

- [Custom States and Command Categories](11-custom-states.md) - Define operational modes and concurrent commands
- [Safety](../topics/safety.md) - Understanding the safety system
- [Parameters](07-parameters.md) - Runtime-adjustable configuration
