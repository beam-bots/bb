<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Sensors and PubSub

In this tutorial, you'll learn how to add sensors to your robot and subscribe to their messages using Kinetix's hierarchical PubSub system.

## Prerequisites

Complete [Starting and Stopping](02-starting-and-stopping.md). You should have a `MyRobot` module that you can start.

## Adding a Sensor to the DSL

Sensors are processes that publish data. Add one to your robot:

```elixir
defmodule MyRobot do
  use Kinetix

  topology do
    link :base do
      sensor :imu, MyImuSensor

      joint :pan_joint do
        # ... rest of robot
      end
    end
  end
end
```

The sensor declaration takes:
- A name (`:imu`)
- A child spec (`MyImuSensor` or `{MyImuSensor, options}`)

Sensors can be attached at three levels:
- **Robot level** - in a `sensors do ... end` block
- **Link level** - inside a link definition
- **Joint level** - inside a joint definition

## Implementing a Sensor Process

A sensor is a GenServer that publishes messages. Here's a simple IMU sensor:

```elixir
defmodule MyImuSensor do
  use GenServer

  alias Kinetix.Message.Sensor.Imu
  alias Kinetix.Message.{Vec3, Quaternion}
  alias Kinetix.PubSub

  def start_link(opts) do
    GenServer.start_link(__MODULE__, opts)
  end

  @impl GenServer
  def init(opts) do
    # Kinetix passes robot context in opts
    robot = Keyword.fetch!(opts, :robot)
    path = Keyword.fetch!(opts, :path)

    # Schedule periodic readings
    :timer.send_interval(100, :read_sensor)

    {:ok, %{robot: robot, path: path}}
  end

  @impl GenServer
  def handle_info(:read_sensor, state) do
    # Create an IMU message
    {:ok, message} = Imu.new(:imu,
      orientation: Quaternion.identity(),
      angular_velocity: Vec3.zero(),
      linear_acceleration: Vec3.new(0.0, 0.0, 9.81)
    )

    # Publish to subscribers
    # Path format: [:sensor | location_path]
    PubSub.publish(state.robot, [:sensor | state.path], message)

    {:noreply, state}
  end
end
```

Key points:

- Kinetix passes `:robot` and `:path` in the options
- The path reflects where the sensor is in the topology (e.g., `[:base, :imu]`)
- Publish with `[:sensor | path]` to identify it as a sensor message

> **For Roboticists:** This is similar to ROS publishers. The sensor publishes on a topic (path) and subscribers receive the messages asynchronously.

> **For Elixirists:** The sensor is just a GenServer. Kinetix starts it as part of the supervision tree and provides context about where it sits in the robot topology.

## Subscribing to Messages

Start your robot and subscribe to sensor messages:

```elixir
iex> {:ok, _} = Kinetix.Supervisor.start_link(MyRobot)
iex> Kinetix.PubSub.subscribe(MyRobot, [:sensor])
{:ok, #PID<0.234.0>}
```

Now your IEx process receives sensor messages:

```elixir
iex> flush()
{:kinetix, [:sensor, :base, :imu], %Kinetix.Message{...}}
{:kinetix, [:sensor, :base, :imu], %Kinetix.Message{...}}
```

## Subscription Patterns

The path you subscribe to determines which messages you receive:

```elixir
# All sensor messages from anywhere
Kinetix.PubSub.subscribe(MyRobot, [:sensor])

# Sensors under the base link
Kinetix.PubSub.subscribe(MyRobot, [:sensor, :base])

# Only the specific IMU sensor
Kinetix.PubSub.subscribe(MyRobot, [:sensor, :base, :imu])

# All messages (sensors, actuators, everything)
Kinetix.PubSub.subscribe(MyRobot, [])
```

## Filtering by Message Type

Subscribe only to specific message types:

```elixir
alias Kinetix.Message.Sensor.Imu

Kinetix.PubSub.subscribe(MyRobot, [:sensor],
  message_types: [Imu]
)
```

This is useful when you have many sensors but only care about IMU data.

## Receiving Messages in a Process

In a real application, you'll receive messages in a GenServer:

```elixir
defmodule MyController do
  use GenServer

  def start_link(opts) do
    GenServer.start_link(__MODULE__, opts)
  end

  @impl GenServer
  def init(opts) do
    robot = Keyword.fetch!(opts, :robot)

    # Subscribe to all sensor messages
    Kinetix.PubSub.subscribe(robot, [:sensor])

    {:ok, %{robot: robot}}
  end

  @impl GenServer
  def handle_info({:kinetix, path, message}, state) do
    # Process the sensor message
    IO.inspect(message.payload, label: "Received from #{inspect(path)}")
    {:noreply, state}
  end
end
```

## Message Structure

Messages have a standard envelope structure:

```elixir
%Kinetix.Message{
  timestamp: -576460748776542,  # monotonic nanoseconds
  frame_id: :imu,
  payload: %Kinetix.Message.Sensor.Imu{
    orientation: {:quaternion, 0.0, 0.0, 0.0, 1.0},
    angular_velocity: {:vec3, 0.0, 0.0, 0.0},
    linear_acceleration: {:vec3, 0.0, 0.0, 9.81}
  }
}
```

- `timestamp` - Monotonic time in nanoseconds (from `System.monotonic_time/1`)
- `frame_id` - Coordinate frame for the data (typically the sensor name)
- `payload` - The actual sensor data struct (type depends on message type)

## Available Message Types

Kinetix includes common sensor message types:

| Module | Description |
|--------|-------------|
| `Kinetix.Message.Sensor.Imu` | Accelerometer, gyroscope |
| `Kinetix.Message.Sensor.JointState` | Joint positions, velocities, efforts |
| `Kinetix.Message.Sensor.LaserScan` | Lidar range data |
| `Kinetix.Message.Sensor.Range` | Single distance measurement |
| `Kinetix.Message.Sensor.Image` | Camera images |
| `Kinetix.Message.Sensor.BatteryState` | Battery status |

And geometry types for transforms and motion:

| Module | Description |
|--------|-------------|
| `Kinetix.Message.Geometry.Pose` | Position + orientation |
| `Kinetix.Message.Geometry.Twist` | Linear + angular velocity |
| `Kinetix.Message.Geometry.Wrench` | Force + torque |
| `Kinetix.Message.Geometry.Transform` | Coordinate transform |

## Creating Custom Payload Types

You can define your own payload types for domain-specific sensor data. A payload type must:

1. Implement the `Kinetix.Message` behaviour (provides the schema)
2. Implement the `Kinetix.Message.Payload` protocol (enables runtime introspection)

Here's a complete example for a custom temperature sensor:

```elixir
defmodule MyApp.Message.Temperature do
  @moduledoc "Temperature reading from a thermal sensor."

  @behaviour Kinetix.Message

  defstruct [:celsius, :sensor_id]

  @type t :: %__MODULE__{
          celsius: float(),
          sensor_id: atom()
        }

  # Define the schema using Spark.Options
  @schema Spark.Options.new!(
            celsius: [
              type: :float,
              required: true,
              doc: "Temperature in degrees Celsius"
            ],
            sensor_id: [
              type: :atom,
              required: true,
              doc: "Identifier of the temperature sensor"
            ]
          )

  # Behaviour callback - returns the compiled schema
  @impl Kinetix.Message
  def schema, do: @schema

  # Protocol implementation - enables runtime schema lookup
  defimpl Kinetix.Message.Payload do
    def schema(_payload), do: @for.schema()
  end

  # Convenience constructor (optional but recommended)
  @spec new(atom(), atom(), float()) ::
          {:ok, Kinetix.Message.t()} | {:error, term()}
  def new(frame_id, sensor_id, celsius) do
    Kinetix.Message.new(__MODULE__, frame_id,
      celsius: celsius,
      sensor_id: sensor_id
    )
  end
end
```

Use your custom payload in a sensor:

```elixir
defmodule MyTemperatureSensor do
  use GenServer

  alias MyApp.Message.Temperature
  alias Kinetix.PubSub

  def start_link(opts), do: GenServer.start_link(__MODULE__, opts)

  @impl GenServer
  def init(opts) do
    robot = Keyword.fetch!(opts, :robot)
    path = Keyword.fetch!(opts, :path)

    :timer.send_interval(1000, :read_temperature)

    {:ok, %{robot: robot, path: path}}
  end

  @impl GenServer
  def handle_info(:read_temperature, state) do
    # Read from actual hardware here
    celsius = 23.5 + :rand.uniform() * 2

    {:ok, message} = Temperature.new(:thermal_sensor, :temp_1, celsius)
    PubSub.publish(state.robot, [:sensor | state.path], message)

    {:noreply, state}
  end
end
```

The `Spark.Options` schema validates attributes when creating messages. If validation fails, `Kinetix.Message.new/3` returns `{:error, reason}` with details about what went wrong.

## Unsubscribing

Stop receiving messages:

```elixir
Kinetix.PubSub.unsubscribe(MyRobot, [:sensor])
```

## Debugging Subscriptions

List who's subscribed to a path:

```elixir
iex> Kinetix.PubSub.subscribers(MyRobot, [:sensor])
[{#PID<0.234.0>, []}]  # PID and message type filters
```

## Sensors with Options

Pass configuration to your sensor:

```elixir
topology do
  link :base do
    sensor :imu, {MyImuSensor, sample_rate: 200, bus: :spi0}
  end
end
```

Your sensor receives these in `start_link/1`:

```elixir
def init(opts) do
  robot = Keyword.fetch!(opts, :robot)
  path = Keyword.fetch!(opts, :path)
  sample_rate = Keyword.get(opts, :sample_rate, 100)
  bus = Keyword.get(opts, :bus, :i2c1)

  # ...
end
```

## Robot-Level Sensors

Some sensors aren't attached to a specific link (e.g., GPS, battery monitor). Define them at robot level:

```elixir
defmodule MyRobot do
  use Kinetix

  sensors do
    sensor :gps, GpsSensor
    sensor :battery, BatteryMonitor
  end

  topology do
    # ... links and joints
  end
end
```

These sensors publish with shorter paths: `[:sensor, :gps]` instead of `[:sensor, :base, :gps]`.

## What's Next?

You can now publish and subscribe to sensor data. In the next tutorial, we'll:

- Use sensor data to compute robot state
- Understand forward kinematics
- Calculate link positions from joint angles

Continue to [Forward Kinematics](04-kinematics.md).
