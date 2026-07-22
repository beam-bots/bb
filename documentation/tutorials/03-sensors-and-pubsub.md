<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Sensors and PubSub

In this tutorial, you'll learn how to add sensors to your robot and subscribe to their messages using Beam Bots' hierarchical PubSub system.

## Prerequisites

Complete [Starting and Stopping](02-starting-and-stopping.md). You should have a `MyRobot.Robot` module that you can start.

## Adding a Sensor to the DSL

Sensors are supervised components that publish data. Add one to your robot:

```elixir
defmodule MyRobot.Robot do
  use BB

  topology do
    link :base_link do
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
- A `BB.Sensor` callback module (`MyImuSensor`) or a callback module with options (`{MyImuSensor, options}`)

Sensors can be attached at three levels:
- **Robot level** - in a `sensors do ... end` block
- **Link level** - inside a link definition
- **Joint level** - inside a joint definition

## Implementing a Sensor

A sensor implementation is a `BB.Sensor` callback module. `BB.Sensor.Server` owns the GenServer process and delegates callbacks to your module, so the implementation does not define `start_link/1` or `use GenServer`.

Here's a simple IMU sensor. Its generated reading represents a stationary, upright sensor; replace `read_sensor/1` with the call to your hardware driver:

```elixir
defmodule MyImuSensor do
  use BB.Sensor,
    options_schema: [
      bus: [type: :atom, default: :i2c1, doc: "Hardware bus"],
      sample_rate: [
        type: {:in, 1..1000},
        default: 10,
        doc: "Samples per second"
      ]
    ]

  alias BB.Math.{Quaternion, Vec3}
  alias BB.Message.Sensor.Imu

  @impl BB.Sensor
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)
    sample_rate = Keyword.fetch!(opts, :sample_rate)

    state = %{
      bb: bb,
      bus: Keyword.fetch!(opts, :bus),
      sample_interval_ms: div(1000, sample_rate)
    }

    schedule_read(state.sample_interval_ms)
    {:ok, state}
  end

  @impl BB.Sensor
  def handle_info(:read_sensor, state) do
    {orientation, angular_velocity, linear_acceleration} = read_sensor(state.bus)

    {:ok, message} = Imu.new(List.last(state.bb.path),
      orientation: orientation,
      angular_velocity: angular_velocity,
      linear_acceleration: linear_acceleration
    )

    BB.publish(state.bb.robot, [:sensor | state.bb.path], message)

    schedule_read(state.sample_interval_ms)
    {:noreply, state}
  end

  defp read_sensor(_bus) do
    {Quaternion.identity(), Vec3.zero(), Vec3.new(0.0, 0.0, 9.81)}
  end

  defp schedule_read(interval_ms) do
    Process.send_after(self(), :read_sensor, interval_ms)
  end
end
```

Key points:

- `BB.Sensor.Server` receives the DSL options, validates them against `options_schema`, applies defaults, and injects `:bb` before calling `MyImuSensor.init/1`.
- The injected value is `%{robot: module, path: [atom]}`. For this sensor, `bb.path` is `[:base_link, :imu]`.
- `Process.send_after/3` schedules the next reading only after the current one has been handled, so readings do not accumulate if one takes longer than expected.
- Publish to `[:sensor | bb.path]`, passing `bb.robot` explicitly as the first argument.

> **For Roboticists:** This is similar to ROS publishers. The sensor publishes on a topic (path) and subscribers receive the messages asynchronously.

> **For Elixirists:** `MyImuSensor` supplies GenServer-style callbacks but is not itself a GenServer. BB starts `BB.Sensor.Server` in the robot's supervision tree, and that wrapper delegates to the callback module with the sensor's resolved options and topology context.

## Subscribing to Messages

Your robot is already running (it starts with your application), so subscribe to its sensor messages:

```elixir
iex> BB.subscribe(MyRobot.Robot, [:sensor])
{:ok, #PID<0.234.0>}
```

Now your IEx process receives sensor messages:

```elixir
iex> flush()
{:bb, [:sensor, :base_link, :imu], %BB.Message{robot: MyRobot.Robot, ...}}
{:bb, [:sensor, :base_link, :imu], %BB.Message{robot: MyRobot.Robot, ...}}
```

The delivery tuple contains the full topic path and message envelope. `BB.publish/3` stamps the publishing robot into `message.robot` before dispatching it.

## Subscription Patterns

The path you subscribe to determines which messages you receive:

```elixir
# All sensor messages from anywhere
BB.subscribe(MyRobot.Robot, [:sensor])

# Sensors under the base_link
BB.subscribe(MyRobot.Robot, [:sensor, :base_link])

# Only the specific IMU sensor
BB.subscribe(MyRobot.Robot, [:sensor, :base_link, :imu])

# All messages (sensors, actuators, everything)
BB.subscribe(MyRobot.Robot, [])
```

## Filtering by Message Type

Subscribe only to specific message types:

```elixir
alias BB.Message.Sensor.Imu

BB.subscribe(MyRobot.Robot, [:sensor],
  message_types: [Imu]
)
```

This is useful when you have many sensors but only care about IMU data.

## Receiving Messages in a Process

In a real application, you might receive messages in a standalone GenServer. This process is not a robot component, so BB does not inject any options into it; its caller passes the robot module explicitly:

```elixir
defmodule MySensorSubscriber do
  use GenServer

  def start_link(robot) do
    GenServer.start_link(__MODULE__, robot)
  end

  @impl GenServer
  def init(robot) do
    {:ok, _pid} = BB.subscribe(robot, [:sensor])

    {:ok, %{}}
  end

  @impl GenServer
  def handle_info({:bb, path, %BB.Message{} = message}, state) do
    IO.inspect(message.payload,
      label: "Received from #{inspect(message.robot)} at #{inspect(path)}"
    )

    {:noreply, state}
  end
end
```

Start it with `MySensorSubscriber.start_link(MyRobot.Robot)`. If a process subscribes to more than one robot, `message.robot` identifies which robot published each delivery.

## Message Structure

Messages have a standard envelope structure:

```elixir
%BB.Message{
  monotonic_time: -576460748776542,         # monotonic nanoseconds
  wall_time: 1_737_201_600_000_000_000,     # system nanoseconds (UTC epoch)
  node: :nonode@nohost,
  frame_id: :imu,
  payload: %BB.Message.Sensor.Imu{
    orientation: %BB.Math.Quaternion{
      tensor: #Nx.Tensor<
        f64[4]
        [1.0, 0.0, 0.0, 0.0]
      >
    },
    angular_velocity: %BB.Math.Vec3{
      tensor: #Nx.Tensor<
        f64[3]
        [0.0, 0.0, 0.0]
      >
    },
    linear_acceleration: %BB.Math.Vec3{
      tensor: #Nx.Tensor<
        f64[3]
        [0.0, 0.0, 9.81]
      >
    },
    orientation_covariance: nil,
    angular_velocity_covariance: nil,
    linear_acceleration_covariance: nil
  },
  robot: MyRobot.Robot
}
```

- `monotonic_time` - Monotonic time in nanoseconds (from `System.monotonic_time/1`). Use for ordering and durations within a node.
- `wall_time` - System time in nanoseconds (from `System.system_time/1`). Use for correlation with real-world time, logging, and recording/playback.
- `node` - The BEAM node that produced the message. Useful in distributed deployments.
- `frame_id` - Coordinate frame for the data (typically the sensor name).
- `payload` - The actual sensor data struct (type depends on message type).
- `robot` - The robot module supplied to `BB.publish/3`. This is `nil` until the message is published, then BB fills it before delivery.

## Available Message Types

BB includes common sensor message types:

| Module | Description |
|--------|-------------|
| `BB.Message.Sensor.Imu` | Accelerometer, gyroscope |
| `BB.Message.Sensor.JointState` | Joint positions, velocities, efforts |
| `BB.Message.Sensor.LaserScan` | Lidar range data |
| `BB.Message.Sensor.Range` | Single distance measurement |
| `BB.Message.Sensor.Image` | Camera images |
| `BB.Message.Sensor.BatteryState` | Battery status |

And geometry types for transforms and motion:

| Module | Description |
|--------|-------------|
| `BB.Message.Geometry.Point3D` | 3D point (wraps `BB.Math.Vec3`) |
| `BB.Message.Geometry.Pose` | Position + orientation (wraps `BB.Math.Transform`) |
| `BB.Message.Geometry.Twist` | Linear + angular velocity |
| `BB.Message.Geometry.Wrench` | Force + torque |
| `BB.Message.Geometry.Accel` | Linear + angular acceleration |

## Creating Custom Payload Types

You can define your own payload types for domain-specific sensor data. Use the `use BB.Message` macro with a schema:

```elixir
defmodule MyApp.Message.Temperature do
  @moduledoc "Temperature reading from a thermal sensor."

  defstruct [:celsius, :sensor_id]

  use BB.Message,
    schema: [
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
    ]

  @type t :: %__MODULE__{
          celsius: float(),
          sensor_id: atom()
        }

  # Custom convenience constructor (in addition to generated new/2)
  @spec new(atom(), atom(), float()) ::
          {:ok, BB.Message.t()} | {:error, term()}
  def new(frame_id, sensor_id, celsius) do
    new(frame_id, celsius: celsius, sensor_id: sensor_id)
  end
end
```

The `use BB.Message` macro:
- Sets up the `BB.Message` behaviour
- Compiles the schema via `Spark.Options`
- Generates a `new/2` function: `new(frame_id, attrs)`
- Implements the `schema/0` callback

Note: Define `defstruct` before `use BB.Message`.

Use your custom payload in a sensor:

```elixir
defmodule MyTemperatureSensor do
  use BB.Sensor

  alias MyApp.Message.Temperature

  @impl BB.Sensor
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)

    schedule_read()
    {:ok, %{bb: bb}}
  end

  @impl BB.Sensor
  def handle_info(:read_temperature, state) do
    celsius = 23.5 + :rand.uniform() * 2
    sensor_name = List.last(state.bb.path)

    {:ok, message} = Temperature.new(sensor_name, sensor_name, celsius)
    BB.publish(state.bb.robot, [:sensor | state.bb.path], message)

    schedule_read()
    {:noreply, state}
  end

  defp schedule_read do
    Process.send_after(self(), :read_temperature, 1000)
  end
end
```

The random value keeps the example runnable without hardware; replace it with the temperature driver's read call in a real sensor.

The `Spark.Options` schema validates attributes when creating messages. If validation fails, `BB.Message.new/3` returns `{:error, reason}` with details about what went wrong.

## Unsubscribing

Stop receiving messages:

```elixir
BB.unsubscribe(MyRobot.Robot, [:sensor])
```

## Debugging Subscriptions

List who's subscribed to a path:

```elixir
iex> BB.PubSub.subscribers(MyRobot.Robot, [:sensor])
[{#PID<0.234.0>, []}]  # PID and message type filters
```

## Sensors with Options

The IMU's `options_schema` declares every user option that its wrapper accepts. You can therefore override the defaults in the DSL:

```elixir
topology do
  link :base_link do
    sensor :imu, {MyImuSensor, sample_rate: 200, bus: :spi0}
  end
end
```

Before `init/1` runs, `BB.Sensor.Server` validates these values, applies any schema defaults, and merges in the framework-owned `:bb` and `:sensor_profile` options. The callback reads the validated values directly:

```elixir
def init(opts) do
  bb = Keyword.fetch!(opts, :bb)
  bus = Keyword.fetch!(opts, :bus)
  sample_rate = Keyword.fetch!(opts, :sample_rate)

  state = %{
    bb: bb,
    bus: bus,
    sample_interval_ms: div(1000, sample_rate)
  }

  schedule_read(state.sample_interval_ms)
  {:ok, state}
end
```

Do not add `:bb` or `:sensor_profile` to `options_schema`; they are injected by the wrapper. Unknown user options and values outside the schema, such as a sample rate above 1000 Hz in this example, prevent the sensor from starting with a validation error.

## Robot-Level Sensors

Some sensors aren't attached to a specific link (e.g., GPS, battery monitor). Define them at robot level:

```elixir
defmodule MyRobot.Robot do
  use BB

  sensors do
    sensor :gps, GpsSensor
    sensor :battery, BatteryMonitor
  end

  topology do
    # ... links and joints
  end
end
```

These sensors publish with shorter paths: `[:sensor, :gps]` instead of `[:sensor, :base_link, :gps]`.

## What's Next?

You can now publish and subscribe to sensor data. In the next tutorial, we'll:

- Use sensor data to compute robot state
- Understand forward kinematics
- Calculate link positions from joint angles

Continue to [Forward Kinematics](04-kinematics.md).
