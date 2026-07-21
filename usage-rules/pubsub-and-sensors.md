<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# PubSub and Sensors

BB routes messages hierarchically by **path** — the list of names locating a
component in the topology. Subscribe to a path to receive that node and
everything beneath it.

## Subscribing

```elixir
BB.PubSub.subscribe(MyRobot.Robot, [:sensor])            # all sensor messages
BB.PubSub.subscribe(MyRobot.Robot, [:sensor, :base_link]) # one subtree
BB.PubSub.subscribe(MyRobot.Robot, [])                    # everything
```

Messages arrive as a three-element tuple whose payload is a `%BB.Message{}`:

```elixir
def handle_info({:bb, path, %BB.Message{payload: payload}}, state) do
  # payload is e.g. %BB.Message.Sensor.Imu{}, %BB.Message.Sensor.JointState{}
  {:noreply, state}
end
```

`BB.Message` wraps every payload with a timestamp and frame id — carry the
whole struct, don't unwrap it early. (`BB.subscribe/3` and `BB.publish/3` are
shortcuts for the `BB.PubSub` functions.)

## Writing a sensor

`use BB.Sensor`, define `init/1` plus GenServer-style callbacks. BB injects
`:bb => %{robot: ..., path: ...}` into your options and supervises the process
for you — you never write a `child_spec`. Publish readings under
`[:sensor | path]`:

```elixir
def handle_info(:read, %{bb: %{robot: robot, path: path}} = state) do
  {:ok, message} = BB.Message.Sensor.Range.new(:range, range: read_hardware())
  BB.PubSub.publish(robot, [:sensor | path], message)
  {:noreply, state}
end
```

Build a payload with its module's generated `new/2` — `Module.new(frame_id,
attrs)` validates against the payload schema and returns
`{:ok, %BB.Message{}}`. The first argument is the frame id (an atom naming the
coordinate frame, conventionally the sensor's own name).

## Message naming

When choosing a payload type under `BB.Message.Sensor.*` / `.Actuator.*`, match
the existing convention rather than inventing a suffix:

- **`*State`** — a multi-field snapshot of an entity's current condition
  (`BatteryState`, `JointState`, `PowerState`).
- **A naked noun** — a single reading that *is* the sample (`Image`, `Range`,
  `LaserScan`, `Imu`).
- **`VerbObject`** — an event or notification (`BeginMotion`).

See [Sensors and PubSub](https://hexdocs.pm/bb/03-sensors-and-pubsub.html) and
the [message-types reference](https://hexdocs.pm/bb/message-types.html).
