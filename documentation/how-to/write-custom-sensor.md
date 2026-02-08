<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# How to Write a Custom Sensor

Create a sensor module that publishes data to BB's PubSub system.

## Prerequisites

- Familiarity with the BB DSL (see [First Robot](../tutorials/01-first-robot.md))
- Understanding of BB PubSub (see [Sensors and PubSub](../tutorials/03-sensors-and-pubsub.md))
- GenServer knowledge

## Step 1: Create the Sensor Module

A sensor is a GenServer that reads data and publishes messages:

```elixir
defmodule MySensor do
  use GenServer, restart: :permanent

  alias BB.Message.Sensor.Range

  @schema [
    pin: [
      type: :non_neg_integer,
      required: true,
      doc: "GPIO pin for the sensor"
    ],
    poll_interval: [
      type: :pos_integer,
      default: 100,
      doc: "Polling interval in milliseconds"
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
    # Store BB context for publishing
    state = %{
      bb: bb,
      opts: opts,
      last_reading: nil
    }

    # Start polling
    schedule_poll(opts[:poll_interval])

    {:ok, state}
  end

  @impl GenServer
  def handle_info(:poll, state) do
    reading = read_sensor(state.opts[:pin])

    # Publish if changed (optional - can publish every time)
    if reading != state.last_reading do
      publish_reading(reading, state)
    end

    schedule_poll(state.opts[:poll_interval])
    {:noreply, %{state | last_reading: reading}}
  end

  defp schedule_poll(interval) do
    Process.send_after(self(), :poll, interval)
  end

  defp read_sensor(pin) do
    # Your hardware reading logic
    # Returns distance in metres
    0.5
  end

  defp publish_reading(distance, state) do
    message = Range.new!(
      range: distance,
      min_range: 0.02,
      max_range: 4.0,
      radiation_type: :ultrasound
    )

    BB.publish(state.bb.robot_module, state.bb.path, message)
  end
end
```

## Step 2: Use in Robot Definition

Add the sensor to your robot:

```elixir
defmodule MyRobot do
  use BB

  topology do
    link :base do
      sensor :distance, {MySensor, pin: 18, poll_interval: 50}
    end
  end
end
```

## Step 3: Subscribe to Sensor Data

Consume the sensor data elsewhere:

```elixir
# In another process
BB.subscribe(MyRobot, [:sensor, :distance])

# In handle_info
def handle_info({:bb, [:sensor, :distance], %{payload: range}}, state) do
  IO.puts("Distance: #{range.range}m")
  {:noreply, state}
end
```

## Event-Driven Sensors

For sensors with hardware interrupts (not polling):

```elixir
defmodule InterruptSensor do
  use GenServer, restart: :permanent

  def start_link(init_arg) do
    {bb, init_arg} = Keyword.pop!(init_arg, :bb)
    GenServer.start_link(__MODULE__, {bb, init_arg})
  end

  @impl GenServer
  def init({bb, opts}) do
    # Set up interrupt handler
    {:ok, gpio} = Circuits.GPIO.open(opts[:pin], :input)
    Circuits.GPIO.set_interrupts(gpio, :both)

    {:ok, %{bb: bb, gpio: gpio}}
  end

  @impl GenServer
  def handle_info({:circuits_gpio, _pin, _timestamp, value}, state) do
    # Publish on interrupt
    message = create_message(value)
    BB.publish(state.bb.robot_module, state.bb.path, message)

    {:noreply, state}
  end
end
```

## Publishing Different Message Types

### Joint State (Position Feedback)

```elixir
alias BB.Message.Sensor.JointState

def publish_position(position, velocity, state) do
  message = JointState.new!(
    names: [state.joint_name],
    positions: [position],
    velocities: [velocity]
  )

  BB.publish(state.bb.robot_module, state.bb.path, message)
end
```

### IMU Data

```elixir
alias BB.Message.Sensor.Imu

def publish_imu(orientation, angular_vel, linear_accel, state) do
  message = Imu.new!(
    orientation: orientation,
    angular_velocity: angular_vel,
    linear_acceleration: linear_accel
  )

  BB.publish(state.bb.robot_module, state.bb.path, message)
end
```

### Battery State

```elixir
alias BB.Message.Sensor.BatteryState

def publish_battery(voltage, current, percentage, state) do
  message = BatteryState.new!(
    voltage: voltage,
    current: current,
    percentage: percentage
  )

  BB.publish(state.bb.robot_module, [:sensor, :battery], message)
end
```

## Sensor with Calibration

Store calibration data and apply during reading:

```elixir
defmodule CalibratedSensor do
  use GenServer, restart: :permanent

  @schema [
    pin: [type: :non_neg_integer, required: true],
    calibration: [
      type: :map,
      default: %{offset: 0.0, scale: 1.0}
    ]
  ]

  def start_link(init_arg) do
    {bb, init_arg} = Keyword.pop!(init_arg, :bb)
    opts = Spark.Options.validate!(init_arg, @schema)
    GenServer.start_link(__MODULE__, {bb, opts})
  end

  @impl GenServer
  def init({bb, opts}) do
    {:ok, %{bb: bb, opts: opts}}
  end

  defp read_and_calibrate(state) do
    raw = read_raw(state.opts[:pin])
    cal = state.opts[:calibration]

    raw * cal.scale + cal.offset
  end
end
```

## Robot-Level vs Joint-Level Sensors

### Joint-Level (Inside Topology)

```elixir
topology do
  link :arm do
    joint :shoulder do
      sensor :encoder, {EncoderSensor, channel: 0}
    end
  end
end
```

Path: `[:sensor, :encoder]` (relative to joint)

### Robot-Level (Outside Topology)

```elixir
sensors do
  sensor :battery, {BatterySensor, adc_channel: 0}
  sensor :imu, {IMUSensor, bus: "i2c-1", address: 0x68}
end

topology do
  # ...
end
```

Path: `[:sensor, :battery]`, `[:sensor, :imu]`

## Testing Sensors

```elixir
defmodule MySensorTest do
  use ExUnit.Case

  test "publishes range messages" do
    {:ok, _} = BB.Supervisor.start_link(TestRobot, simulation: :kinematic)

    # Subscribe to sensor
    BB.subscribe(TestRobot, [:sensor, :distance])

    # Wait for message
    assert_receive {:bb, [:sensor, :distance], %{payload: %Range{} = range}}, 1000
    assert range.range >= 0.02
    assert range.range <= 4.0
  end
end
```

## Safety Considerations

For sensors that might affect safety decisions:

```elixir
def handle_info(:poll, state) do
  case read_sensor(state.opts[:pin]) do
    {:ok, reading} ->
      publish_reading(reading, state)
      {:noreply, %{state | last_reading: reading, errors: 0}}

    {:error, reason} ->
      new_errors = state.errors + 1

      if new_errors >= 3 do
        # Report persistent error
        BB.Safety.report_error(
          state.bb.robot_module,
          state.bb.path,
          {:sensor_failure, reason}
        )
      end

      {:noreply, %{state | errors: new_errors}}
  end
end
```

## Common Issues

### Messages Not Received

Check that:
- The sensor is started (part of supervision tree)
- Subscribers use the correct path
- The message type is valid

### High CPU Usage

For high-frequency sensors:
- Batch readings before publishing
- Use longer poll intervals if acceptable
- Consider hardware filtering

### Stale Data

If data seems delayed:
- Check poll interval
- Verify no blocking operations in read function
- Consider event-driven approach

## Next Steps

- Add calibration UI with [bb_kino](https://hexdocs.pm/bb_kino)
- Implement sensor fusion for multiple inputs
- Add telemetry for monitoring sensor health
