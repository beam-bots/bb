<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Understanding the Command System

This document explains the design of Beam Bots' command system - why commands are short-lived processes, how they integrate with the state machine, and the patterns they enable.

## The Core Model

Commands in BB are **short-lived GenServers** that execute discrete operations. They're not background services - they start, do something, and stop.

```
execute command
      │
      ▼
┌─────────────────┐
│ Command Process │ ─── receives messages, updates state
│   (GenServer)   │ ─── decides when complete
└─────────────────┘
      │
      ▼
  return result
```

This model is borrowed from action servers in ROS, but adapted for Erlang/OTP semantics.

## Why Processes?

Commands could be simple function calls. Why make them processes?

### Async by Default

Commands can take time. A movement command might wait for motors to reach position. Making commands processes means:

- Callers don't block
- Multiple commands can (potentially) run concurrently
- Timeouts are handled naturally

### Message-Driven

Commands often need to react to external events:

- Sensor feedback (reached position?)
- Hardware errors (servo overheating?)
- User cancellation

As processes, commands can subscribe to PubSub and receive messages.

### Supervision

Command processes are supervised. If a command crashes:

- The robot returns to a safe state
- Awaiting callers receive an error
- Resources are cleaned up

### State Machine Integration

Commands interact with the robot state machine. Being processes lets them:

- Check state before executing
- Hold the robot in a state while running
- Transition state on completion

## The State Machine

Every robot has an operational state:

```
:disarmed ─arm─→ :idle ─disarm─→ :disarmed
```

Commands declare which states they can run in:

```elixir
command :move_to do
  handler MoveCommand
  allowed_states [:idle]
end

command :arm do
  handler BB.Command.Arm
  allowed_states [:disarmed]
end
```

The Runtime enforces these constraints:

```elixir
BB.Robot.Runtime.state(MyRobot)  #=> :disarmed
MyRobot.move_to(position: 1.0)   #=> {:error, %NotAllowed{}}
```

### Custom States

For complex robots, you can define custom states beyond `:idle` using the `states` DSL section:

```elixir
states do
  state :recording do
    doc "Recording trajectory data"
  end

  state :playback do
    doc "Playing back recorded trajectory"
  end

  state :calibrating do
    doc "Running calibration routine"
  end
end

commands do
  command :start_recording do
    handler {BB.Command.SetState, to: :recording}
    allowed_states [:idle]
  end

  command :start_playback do
    allowed_states [:recording]
  end
end
```

The built-in states (`:idle`, `:disarmed`) are always available. Your custom states extend what's possible. See the [Custom States and Categories](../tutorials/11-custom-states.md) tutorial for comprehensive coverage.

## Command Lifecycle

### 1. Execution Request

```elixir
{:ok, cmd_pid} = MyRobot.move_to(target: %{shoulder: 0.5})
```

The DSL generates this function. It calls `BB.Robot.Runtime.execute/3`.

### 2. State Validation

Runtime checks:
- Robot is in an allowed state
- Command category has capacity (for concurrent commands)

### 3. Process Start

A GenServer starts under the Runtime's DynamicSupervisor:

```elixir
DynamicSupervisor.start_child(supervisor, {CommandServer, opts})
```

### 4. Handler Invocation

The CommandServer calls your handler's `handle_command/3`:

```elixir
@impl BB.Command
def handle_command(goal, context, state) do
  # goal: arguments from the execute call
  # context: robot module, struct, state handle
  # state: command's internal state (includes :result)

  # Return GenServer-style tuple
  {:noreply, updated_state}
end
```

### 5. Execution

The command runs as a GenServer:
- Receives messages via `handle_info/2`
- Can make calls with `handle_call/3`
- Subscribes to PubSub for sensor feedback

### 6. Completion

When done, return `{:stop, reason, state}`:

```elixir
def handle_info(:done, state) do
  {:stop, :normal, %{state | result: {:ok, :completed}}}
end
```

### 7. Result Extraction

On termination, `result/1` is called:

```elixir
@impl BB.Command
def result(%{result: result}), do: result
```

The result goes to awaiting callers.

## Awaiting Results

Callers have options:

### Blocking Wait

```elixir
{:ok, cmd} = MyRobot.move_to(target: %{shoulder: 0.5})
{:ok, result} = BB.Command.await(cmd)  # blocks until done
```

### Timeout

```elixir
case BB.Command.await(cmd, 5000) do
  {:ok, result} -> handle_result(result)
  {:error, :timeout} -> handle_timeout()
end
```

### Non-Blocking Check

```elixir
case BB.Command.yield(cmd, 0) do
  {:ok, result} -> done(result)
  nil -> still_running()
end
```

### Fire and Forget

```elixir
{:ok, _cmd} = MyRobot.move_to(target: %{shoulder: 0.5})
# Don't await - let it run
```

## Command Categories

By default, only one command runs at a time. Categories enable concurrency:

```elixir
commands do
  category :motion do
    doc "Physical movement commands"
    concurrency_limit 1
  end

  category :sensing do
    doc "Sensor and data collection commands"
    concurrency_limit 2  # Allow concurrent sensing
  end

  command :move_to do
    category :motion
    allowed_states [:idle]
  end

  command :read_sensor do
    category :sensing
    allowed_states [:idle]
  end
end
```

Each category has a `concurrency_limit` (default: 1). Commands in different categories can run concurrently. Commands in the same category are limited by that category's concurrency limit.

### Cancellation

Commands can declare they cancel others:

```elixir
command :move_to do
  cancel [:motion]  # Cancels running motion commands
end

command :emergency_stop do
  cancel :*  # Cancels everything
end
```

When cancelled, the command process terminates and `result/1` is called with the current state. Awaiting callers receive whatever `result/1` returns. Commands should handle cancellation with a fallback clause:

```elixir
@impl BB.Command
def result(%{result: result}) do
  {:ok, result}
end

def result(_state), do: {:error, :cancelled}
```

## State Transitions

Commands can change robot state:

```elixir
@impl BB.Command
def result(%{result: {:ok, value}}) do
  {:ok, value, next_state: :recording}
end
```

This is how `BB.Command.Arm` works - it returns `{:ok, :armed, next_state: :idle}`.

## Safety Integration

Commands receive safety state changes:

```elixir
@impl BB.Command
def handle_safety_state_change(:disarming, state) do
  # Robot is being disarmed - stop gracefully
  {:stop, :disarmed, %{state | result: {:error, :disarmed}}}
end
```

The default implementation stops on any safety change. Override for commands that should continue (use with care).

## Design Patterns

### Request-Feedback-Result

The canonical pattern for motion commands:

1. **Request**: Send target to actuators
2. **Feedback**: Subscribe to sensors, monitor progress
3. **Result**: Complete when target reached or timeout

```elixir
def handle_command(goal, context, state) do
  # Request
  publish_targets(goal.target, context)
  subscribe_to_sensors(goal.target, context)

  {:noreply, %{state | target: goal.target}}
end

def handle_info({:bb, [:sensor, _], msg}, state) do
  if reached_target?(msg, state.target) do
    # Result
    {:stop, :normal, %{state | result: {:ok, :reached}}}
  else
    # Feedback (continue waiting)
    {:noreply, state}
  end
end
```

### Coordination

For commands coordinating multiple subsystems:

```elixir
def handle_command(goal, context, state) do
  # Start multiple operations
  start_arm_motion(goal, context)
  start_gripper_action(goal, context)

  {:noreply, %{state | arm_done: false, gripper_done: false}}
end

def handle_info({:arm_complete, _}, state) do
  check_completion(%{state | arm_done: true})
end

def handle_info({:gripper_complete, _}, state) do
  check_completion(%{state | gripper_done: true})
end

defp check_completion(%{arm_done: true, gripper_done: true} = state) do
  {:stop, :normal, %{state | result: {:ok, :complete}}}
end

defp check_completion(state), do: {:noreply, state}
```

### Timeout with Progress

For commands that might hang:

```elixir
def handle_command(goal, _context, state) do
  schedule_timeout(5000)
  {:noreply, %{state | last_progress: now()}}
end

def handle_info(:timeout, state) do
  if stale?(state.last_progress) do
    {:stop, :normal, %{state | result: {:error, :timeout}}}
  else
    schedule_timeout(5000)
    {:noreply, state}
  end
end

def handle_info({:progress, _}, state) do
  {:noreply, %{state | last_progress: now()}}
end
```

## Related Documentation

- [Commands and State Machine](../tutorials/05-commands.md) - Tutorial
- [Custom States and Categories](../tutorials/11-custom-states.md) - Advanced usage
- [How to Add a Custom Command](../how-to/add-custom-command.md) - Step-by-step guide
