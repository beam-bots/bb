<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Parameters

In this tutorial, you'll learn how to define runtime-adjustable parameters for your robot and modify them while the robot is running.

## Prerequisites

Complete [Commands](05-commands.md). You should understand how BB manages robot state and processes.

## What Are Parameters?

Parameters are configuration values that can be changed at runtime without recompiling your code. They're useful for:

- **Tuning controllers** - Adjust PID gains while testing
- **Configuring behaviour** - Change speed limits or safety thresholds
- **Adapting to conditions** - Modify settings based on environment

> **For Roboticists:** Parameters work similarly to ROS2 parameters or ArduPilot's parameter system. You define schemas, get/set values at runtime, and receive change notifications.

> **For Elixirists:** Parameters are validated key-value pairs stored in ETS with PubSub change notifications. Think of them as a typed, observable configuration system.

## Defining Parameters in the DSL

Add a `parameters` section to your robot definition:

```elixir
defmodule MyRobot do
  use BB

  parameters do
    param :max_speed, type: :float, default: 1.0,
      min: 0.0, max: 10.0, doc: "Maximum velocity in m/s"

    param :safety_enabled, type: :boolean, default: true,
      doc: "Enable collision avoidance"
  end

  topology do
    link :base
  end
end
```

Each `param` declaration takes:
- A name (atom)
- `type` - The value type (`:float`, `:integer`, `:boolean`, `:string`, `:atom`, or `{:unit, unit_type}` for physical quantities)
- `default` - Initial value (required)
- `min`/`max` - Optional bounds for numeric types
- `doc` - Description for documentation

## Organising Parameters with Groups

Use `group` to organise related parameters:

```elixir
parameters do
  group :motion do
    param :max_linear_speed, type: :float, default: 1.0,
      min: 0.0, max: 5.0

    param :max_angular_speed, type: :float, default: 0.5,
      min: 0.0, max: 2.0
  end

  group :safety do
    param :collision_distance, type: :float, default: 0.3,
      min: 0.1, max: 2.0

    param :emergency_stop_enabled, type: :boolean, default: true
  end
end
```

Groups create hierarchical paths: `[:motion, :max_linear_speed]`, `[:safety, :collision_distance]`.

Groups can be nested:

```elixir
parameters do
  group :controller do
    group :pid do
      param :kp, type: :float, default: 1.0
      param :ki, type: :float, default: 0.1
      param :kd, type: :float, default: 0.01
    end
  end
end
```

This creates paths like `[:controller, :pid, :kp]`.

## Reading Parameters

Start your robot and read parameter values:

```elixir
iex> {:ok, _} = BB.Supervisor.start_link(MyRobot)
iex> BB.Parameter.get(MyRobot, [:motion, :max_linear_speed])
{:ok, 1.0}

iex> BB.Parameter.get(MyRobot, [:safety, :collision_distance])
{:ok, 0.3}
```

Use `get!/2` if you want to raise on missing parameters:

```elixir
iex> BB.Parameter.get!(MyRobot, [:motion, :max_linear_speed])
1.0
```

## Listing Parameters

Enumerate all parameters or filter by prefix:

```elixir
iex> BB.Parameter.list(MyRobot)
[
  {[:motion, :max_linear_speed], %{value: 1.0, type: :float, ...}},
  {[:motion, :max_angular_speed], %{value: 0.5, type: :float, ...}},
  {[:safety, :collision_distance], %{value: 0.3, type: :float, ...}},
  ...
]

iex> BB.Parameter.list(MyRobot, prefix: [:motion])
[
  {[:motion, :max_linear_speed], %{value: 1.0, type: :float, ...}},
  {[:motion, :max_angular_speed], %{value: 0.5, type: :float, ...}}
]
```

## Writing Parameters

Change parameter values at runtime:

```elixir
iex> BB.Parameter.set(MyRobot, [:motion, :max_linear_speed], 2.0)
:ok

iex> BB.Parameter.get(MyRobot, [:motion, :max_linear_speed])
{:ok, 2.0}
```

Values are validated against the schema. Invalid values are rejected:

```elixir
iex> BB.Parameter.set(MyRobot, [:motion, :max_linear_speed], -1.0)
{:error, "must be at least 0.0"}

iex> BB.Parameter.set(MyRobot, [:motion, :max_linear_speed], "fast")
{:error, "expected float, got \"fast\""}
```

## Atomic Batch Updates

Update multiple parameters atomically with `set_many/2`:

```elixir
iex> BB.Parameter.set_many(MyRobot, [
...>   {[:controller, :pid, :kp], 2.0},
...>   {[:controller, :pid, :ki], 0.2},
...>   {[:controller, :pid, :kd], 0.05}
...> ])
:ok
```

If any parameter fails validation, none are changed:

```elixir
iex> BB.Parameter.set_many(MyRobot, [
...>   {[:controller, :pid, :kp], 2.0},
...>   {[:controller, :pid, :ki], -0.5}  # Invalid: negative
...> ])
{:error, [{[:controller, :pid, :ki], "must be at least 0.0"}]}
```

## Subscribing to Parameter Changes

Parameter changes are published via PubSub. Subscribe to receive notifications:

```elixir
iex> BB.PubSub.subscribe(MyRobot, [:param])
{:ok, #PID<0.234.0>}

iex> BB.Parameter.set(MyRobot, [:motion, :max_linear_speed], 3.0)
:ok

iex> flush()
{:bb, [:param, :motion, :max_linear_speed], %BB.Message{
  payload: %BB.Parameter.Changed{
    path: [:motion, :max_linear_speed],
    old_value: 2.0,
    new_value: 3.0,
    source: :local
  }
}}
```

Subscribe to specific parameter paths:

```elixir
# All motion parameters
BB.PubSub.subscribe(MyRobot, [:param, :motion])

# Just the max speed
BB.PubSub.subscribe(MyRobot, [:param, :motion, :max_linear_speed])
```

## Parameters in Components

Controllers, sensors, and actuators can define inline parameters:

```elixir
topology do
  link :base do
    joint :shoulder, type: :revolute do
      controller :position, {MyPIDController, []} do
        param :kp, type: :float, default: 1.0, min: 0.0
        param :ki, type: :float, default: 0.1, min: 0.0
        param :kd, type: :float, default: 0.01, min: 0.0
      end
    end
  end
end
```

These parameters are accessible via their full path:

```elixir
BB.Parameter.get(MyRobot, [:base, :shoulder, :position, :kp])
```

## Implementing a Parameterised Controller

Here's a complete PID controller that uses parameters:

```elixir
defmodule MyPIDController do
  use GenServer
  @behaviour BB.Parameter

  # Define the parameter schema
  @impl BB.Parameter
  def param_schema do
    Spark.Options.new!(
      kp: [type: :float, required: true, doc: "Proportional gain"],
      ki: [type: :float, default: 0.0, doc: "Integral gain"],
      kd: [type: :float, default: 0.0, doc: "Derivative gain"]
    )
  end

  def start_link(opts) do
    GenServer.start_link(__MODULE__, opts)
  end

  @impl GenServer
  def init(opts) do
    %{robot: robot, path: path} = Keyword.fetch!(opts, :bb)

    # Register our parameters with the runtime
    BB.Parameter.register(robot, path, __MODULE__)

    # Subscribe to parameter changes for our path
    BB.PubSub.subscribe(robot, [:param | path])

    {:ok, %{robot: robot, path: path, integral: 0.0, last_error: 0.0}}
  end

  @impl GenServer
  def handle_info({:bb, [:param | _], _message}, state) do
    # Parameters changed - gains will be read fresh on next compute
    {:noreply, state}
  end

  def compute(pid, setpoint, measurement) do
    GenServer.call(pid, {:compute, setpoint, measurement})
  end

  @impl GenServer
  def handle_call({:compute, setpoint, measurement}, _from, state) do
    # Read current gains
    {:ok, kp} = BB.Parameter.get(state.robot, state.path ++ [:kp])
    {:ok, ki} = BB.Parameter.get(state.robot, state.path ++ [:ki])
    {:ok, kd} = BB.Parameter.get(state.robot, state.path ++ [:kd])

    error = setpoint - measurement
    integral = state.integral + error
    derivative = error - state.last_error

    output = kp * error + ki * integral + kd * derivative

    {:reply, output, %{state | integral: integral, last_error: error}}
  end
end
```

Key points:

1. Implement `BB.Parameter` behaviour with `param_schema/0`
2. Call `BB.Parameter.register/3` in `init/1` to register the schema
3. Subscribe to `[:param | path]` for change notifications
4. Read parameters when needed (they're fast ETS lookups)

## Unit-Typed Parameters

Parameters can use physical units:

```elixir
parameters do
  group :motion do
    param :max_speed, type: {:unit, :meter_per_second}, default: ~u(1.0 meter_per_second),
      min: ~u(0 m/s), max: ~u(10 m/s)

    param :acceleration, type: {:unit, :meter_per_second_squared},
      default: ~u(0.5 meter_per_second_squared)
  end
end
```

Unit parameters are validated and can be converted:

```elixir
iex> BB.Parameter.set(MyRobot, [:motion, :max_speed], ~u(2.0 m/s))
:ok

# Units are converted to SI base for storage
iex> BB.Parameter.get(MyRobot, [:motion, :max_speed])
{:ok, 2.0}  # metres per second
```

## Complete Example

Here's a robot with a tuneable motion system:

```elixir
defmodule TuneableRobot do
  use BB

  parameters do
    group :motion do
      param :max_linear_speed, type: :float, default: 1.0,
        min: 0.0, max: 5.0, doc: "Maximum forward velocity"

      param :max_angular_speed, type: :float, default: 0.5,
        min: 0.0, max: 2.0, doc: "Maximum rotation rate"

      param :acceleration_limit, type: :float, default: 0.5,
        min: 0.1, max: 2.0, doc: "Acceleration ramp rate"
    end

    group :safety do
      param :obstacle_distance, type: :float, default: 0.5,
        min: 0.1, max: 2.0, doc: "Minimum obstacle clearance"

      param :enabled, type: :boolean, default: true,
        doc: "Enable safety systems"
    end
  end

  topology do
    link :base do
      sensor :lidar, MyLidarSensor

      joint :left_wheel, type: :continuous do
        actuator :motor, MyMotor
      end

      joint :right_wheel, type: :continuous do
        actuator :motor, MyMotor
      end
    end
  end
end
```

Tune it from IEx:

```elixir
iex> {:ok, _} = BB.Supervisor.start_link(TuneableRobot)

# Check current settings
iex> BB.Parameter.list(TuneableRobot, prefix: [:motion])
[
  {[:motion, :max_linear_speed], %{value: 1.0, ...}},
  {[:motion, :max_angular_speed], %{value: 0.5, ...}},
  {[:motion, :acceleration_limit], %{value: 0.5, ...}}
]

# Increase speed limit
iex> BB.Parameter.set(TuneableRobot, [:motion, :max_linear_speed], 2.0)
:ok

# Disable safety for testing (carefully!)
iex> BB.Parameter.set(TuneableRobot, [:safety, :enabled], false)
:ok
```

## What's Next?

You can now configure robots at runtime with validated parameters. In the next tutorial, we'll:

- Connect to remote systems via parameter bridges
- Access parameters from ground control stations
- Implement bidirectional parameter sync

Continue to [Parameter Bridges](08-parameter-bridges.md).
