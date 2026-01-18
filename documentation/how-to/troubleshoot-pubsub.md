<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# How to Troubleshoot PubSub

Diagnose and fix common issues with BB's publish-subscribe system.

## Prerequisites

- Understanding of BB PubSub (see [Understanding the PubSub System](../topics/pubsub-system.md))
- A running BB robot

## Common Symptoms

| Symptom | Likely Cause |
|---------|--------------|
| No messages received | Wrong path, not subscribed, publisher not running |
| Messages delayed | Slow subscriber, mailbox backlog |
| Duplicate messages | Multiple subscriptions, multiple publishers |
| Messages stop | Publisher crashed, unsubscribed |

## Diagnostic Tools

### See All Messages

Subscribe to the root path to see everything:

```elixir
BB.subscribe(MyRobot, [])

# Messages will print in IEx
# {:bb, [:sensor, :shoulder], %BB.Message{...}}
# {:bb, [:state_machine], %BB.Message{...}}
```

### Count Messages by Path

```elixir
defmodule MessageCounter do
  use GenServer

  def start_link(robot) do
    GenServer.start_link(__MODULE__, robot, name: __MODULE__)
  end

  def init(robot) do
    BB.subscribe(robot, [])
    {:ok, %{counts: %{}, robot: robot}}
  end

  def get_counts, do: GenServer.call(__MODULE__, :get_counts)
  def reset, do: GenServer.cast(__MODULE__, :reset)

  def handle_call(:get_counts, _from, state) do
    {:reply, state.counts, state}
  end

  def handle_cast(:reset, state) do
    {:noreply, %{state | counts: %{}}}
  end

  def handle_info({:bb, path, _msg}, state) do
    key = Enum.take(path, 2) |> Enum.join(".")
    counts = Map.update(state.counts, key, 1, &(&1 + 1))
    {:noreply, %{state | counts: counts}}
  end
end

# Usage
MessageCounter.start_link(MyRobot)
Process.sleep(5000)
MessageCounter.get_counts()
# => %{"sensor.shoulder" => 50, "state_machine" => 2}
```

### Use Event Stream Widget

In Livebook with bb_kino:

```elixir
BB.Kino.events(MyRobot)
```

Or with bb_liveview dashboard - the event stream component shows all messages.

## Issue: Messages Not Received

### Check 1: Is the Publisher Running?

```elixir
# List all processes for the robot
Registry.select(BB.Registry, [{{MyRobot, :"$1", :"$2"}, [], [{{:"$1", :"$2"}}]}])
# => [{[:sensor, :shoulder], #PID<0.456.0>}, ...]
```

If the expected process isn't listed, check:
- Robot topology includes the sensor/actuator
- Process hasn't crashed (check logs)

### Check 2: Correct Path?

Paths are hierarchical. Common mistakes:

```elixir
# WRONG - extra nesting
BB.subscribe(MyRobot, [:joint, :shoulder, :sensor, :encoder])

# RIGHT - sensors are at joint level
BB.subscribe(MyRobot, [:sensor, :encoder])
```

Check what path the publisher uses:

```elixir
# Subscribe to everything, look at actual paths
BB.subscribe(MyRobot, [])
```

### Check 3: Subscription Active?

Subscriptions are process-linked. If your process restarted, you need to resubscribe:

```elixir
# In GenServer init
def init(opts) do
  BB.subscribe(MyRobot, [:sensor])
  {:ok, %{}}
end

# Also resubscribe after reconnection in LiveView
def handle_info(:reconnected, socket) do
  BB.subscribe(socket.assigns.robot, [:sensor])
  {:noreply, socket}
end
```

### Check 4: Message Type Matches?

Ensure you're pattern matching correctly:

```elixir
# This won't match if payload is different type
def handle_info({:bb, [:sensor, _], %{payload: %JointState{}}}, state)

# More permissive
def handle_info({:bb, [:sensor, _], %{payload: payload}}, state) do
  IO.inspect(payload, label: "Received")
  {:noreply, state}
end
```

## Issue: Messages Delayed

### Check 1: Mailbox Size

```elixir
{:message_queue_len, len} = Process.info(self(), :message_queue_len)
IO.puts("Mailbox has #{len} messages")
```

If mailbox is growing:
- Process messages faster
- Add selective receive
- Consider sampling/throttling

### Check 2: Slow Handler

Profile your handler:

```elixir
def handle_info({:bb, path, msg}, state) do
  {time, result} = :timer.tc(fn -> process_message(msg, state) end)

  if time > 10_000 do  # > 10ms
    Logger.warning("Slow message processing: #{time}µs for #{inspect(path)}")
  end

  result
end
```

### Check 3: High-Frequency Publisher

If a sensor publishes too fast:

```elixir
# Throttle in subscriber
def handle_info({:bb, [:sensor, _], msg}, state) do
  now = System.monotonic_time(:millisecond)

  if now - state.last_processed > 50 do  # Max 20Hz
    process_message(msg)
    {:noreply, %{state | last_processed: now}}
  else
    {:noreply, state}  # Skip this message
  end
end
```

## Issue: Duplicate Messages

### Check 1: Multiple Subscriptions

```elixir
# BAD - subscribes twice
def init(opts) do
  BB.subscribe(MyRobot, [:sensor])
  BB.subscribe(MyRobot, [:sensor, :shoulder])  # Also matches!
  {:ok, %{}}
end

# GOOD - subscribe to most specific path
def init(opts) do
  BB.subscribe(MyRobot, [:sensor, :shoulder])
  {:ok, %{}}
end
```

### Check 2: Multiple Publishers

Check if multiple processes publish to the same path:

```elixir
# Find all processes that might publish to [:sensor, :shoulder]
Registry.select(BB.Registry, [{{MyRobot, :"$1", :"$2"}, [], [{{:"$1", :"$2"}}]}])
|> Enum.filter(fn {path, _pid} ->
  List.starts_with?(path, [:sensor, :shoulder])
end)
```

## Issue: Messages Stop

### Check 1: Publisher Crashed

```elixir
# Check if process is alive
case BB.Process.whereis(MyRobot, [:sensor, :shoulder]) do
  {:ok, pid} ->
    if Process.alive?(pid), do: :running, else: :dead

  {:error, _} ->
    :not_found
end
```

Check supervisor logs for crash reasons.

### Check 2: Unsubscribed

If your process called `BB.unsubscribe/2` or restarted without resubscribing.

### Check 3: Robot Stopped

```elixir
# Check if robot supervisor is running
case Process.whereis(MyRobot.Supervisor) do
  nil -> :stopped
  pid -> if Process.alive?(pid), do: :running, else: :stopping
end
```

## Debugging Techniques

### Add Logging

```elixir
def handle_info({:bb, path, msg}, state) do
  Logger.debug("Received: #{inspect(path)} - #{inspect(msg.payload.__struct__)}")
  # ... handle message
end
```

### Trace Publications

```elixir
# Wrap BB.publish temporarily
defmodule DebugPublish do
  def publish(robot, path, msg) do
    IO.puts("PUBLISH: #{inspect(path)}")
    BB.publish(robot, path, msg)
  end
end
```

### Check Registry State

```elixir
# All subscriptions for a robot
Registry.lookup(BB.PubSub.Registry, {MyRobot, []})
# => [{pid, path_prefix}, ...]
```

## Performance Tuning

### Reduce Message Volume

```elixir
# Only publish on significant change
defp maybe_publish(new_value, state) do
  if abs(new_value - state.last_published) > 0.01 do
    BB.publish(...)
    %{state | last_published: new_value}
  else
    state
  end
end
```

### Batch Messages

```elixir
# Collect readings, publish batch
def handle_info(:flush, state) do
  if length(state.buffer) > 0 do
    message = JointState.new!(
      names: Enum.map(state.buffer, & &1.name),
      positions: Enum.map(state.buffer, & &1.position)
    )
    BB.publish(state.robot, [:sensor, :batch], message)
  end

  schedule_flush()
  {:noreply, %{state | buffer: []}}
end
```

### Use Direct Delivery for Low Latency

For actuator commands where PubSub overhead matters:

```elixir
# Instead of
BB.Actuator.set_position(MyRobot, [:actuator, :servo], position)

# Use direct
BB.Actuator.set_position!(MyRobot, :servo, position)
```

## Quick Reference

| Problem | First Check | Solution |
|---------|-------------|----------|
| No messages | Path correct? | Subscribe to `[]`, check paths |
| Delayed | Mailbox size? | Profile handler, throttle |
| Duplicates | Multiple subs? | Use most specific path |
| Stopped | Process alive? | Check supervisor, logs |

## Related Documentation

- [Understanding the PubSub System](../topics/pubsub-system.md) - Architecture explanation
- [Sensors and PubSub](../tutorials/03-sensors-and-pubsub.md) - Tutorial
- [Reference: Message Types](../reference/message-types.md) - All message types
