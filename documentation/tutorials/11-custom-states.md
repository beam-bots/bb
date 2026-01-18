<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Custom States and Command Categories

In this tutorial, you'll learn how to define custom operational states and use command categories to run multiple commands concurrently.

## Prerequisites

Complete [Commands and State Machine](05-commands.md). You should understand the basic state machine and how to define commands.

## Beyond Idle

The default state machine has just two operational states: `:disarmed` and `:idle`. This works well for simple robots, but real applications often need more operational modes:

- A data collection robot might have a **recording** mode
- A reactive robot might have a **reacting** mode where it responds to stimuli
- A robot running learned behaviours might switch between **inference** and **training** modes

Beam Bots lets you define custom operational states that represent these modes.

## Defining Custom States

Add a `states` section to your robot:

```elixir
defmodule DataCollectionRobot do
  use BB

  states do
    initial_state :idle  # Default, can be omitted

    state :recording do
      doc "Recording sensor data for dataset collection"
    end

    state :processing do
      doc "Processing recorded data"
    end
  end

  commands do
    command :arm do
      handler BB.Command.Arm
      allowed_states [:disarmed]
    end

    command :disarm do
      handler BB.Command.Disarm
      allowed_states [:idle, :recording, :processing]
      cancel :*  # Can cancel any running commands
    end
  end

  topology do
    link :base_link
  end
end
```

The built-in states (`:idle`, `:disarmed`) are always available. Your custom states extend what's possible.

## Transitioning Between States

States can only change via commands - there's no direct API to set the state. This ensures all state transitions are tracked, auditable, and follow the command lifecycle.

### Simple State Transitions with SetState

For straightforward state changes, use the built-in `BB.Command.SetState` handler:

```elixir
commands do
  command :enter_recording do
    handler {BB.Command.SetState, to: :recording}
    allowed_states [:idle]
  end

  command :exit_recording do
    handler {BB.Command.SetState, to: :idle}
    allowed_states [:recording]
  end

  command :start_processing do
    handler {BB.Command.SetState, to: :processing}
    allowed_states [:recording]  # Can only process after recording
  end
end
```

Use these commands like any other:

```elixir
iex> {:ok, _} = BB.Supervisor.start_link(DataCollectionRobot)
iex> {:ok, cmd} = DataCollectionRobot.arm()
iex> {:ok, :armed, _} = BB.Command.await(cmd)

iex> BB.Robot.Runtime.state(DataCollectionRobot)
:idle

iex> {:ok, cmd} = DataCollectionRobot.enter_recording()
iex> {:ok, :recording, _} = BB.Command.await(cmd)

iex> BB.Robot.Runtime.state(DataCollectionRobot)
:recording
```

### State Transitions During Command Execution

Commands that do work over time can transition through multiple states using `BB.Command.transition_state/2`:

```elixir
defmodule DataPipelineCommand do
  use BB.Command

  @impl BB.Command
  def handle_command(_goal, context, state) do
    # Start in :recording state
    :ok = BB.Command.transition_state(context, :recording)

    # Begin recording
    send(self(), :start_recording)
    {:noreply, Map.put(state, :context, context)}
  end

  @impl BB.Command
  def handle_info(:start_recording, state) do
    # ... record data ...
    Process.send_after(self(), :finish_recording, 5000)
    {:noreply, state}
  end

  def handle_info(:finish_recording, state) do
    # Transition to processing
    :ok = BB.Command.transition_state(state.context, :processing)

    # Process the data
    send(self(), :process_data)
    {:noreply, state}
  end

  def handle_info(:process_data, state) do
    # ... process data ...
    {:stop, :normal, Map.put(state, :result, {:ok, :pipeline_complete})}
  end

  @impl BB.Command
  def result(%{result: result}) do
    # Return to :idle when complete
    {:ok, result, next_state: :idle}
  end

  def result(_state), do: {:error, :cancelled}
end
```

## Querying State

Use `BB.Robot.Runtime` to query the current state:

```elixir
# Get the operational state (what mode the robot is in)
BB.Robot.Runtime.operational_state(MyRobot)
# => :idle | :recording | :processing | ...

# Get the "classic" state (backwards compatible)
BB.Robot.Runtime.state(MyRobot)
# => :disarmed | :idle | :executing | :recording | ...
```

The difference between `state/1` and `operational_state/1`:
- `operational_state/1` returns the actual operational mode
- `state/1` returns `:executing` when in `:idle` with commands running (for backwards compatibility)

For custom states, both return the actual state regardless of whether commands are running.

## Command Categories

By default, only one command runs at a time. But some robots need concurrent operations:

- Move the arm **while** recording sensor data
- Blink an LED **while** executing a motion
- Run multiple sensing operations in parallel

Command categories let you define groups of commands with independent concurrency.

### Defining Categories

Add categories to your `commands` section:

```elixir
commands do
  category :motion do
    doc "Physical movement commands"
    concurrency_limit 1  # Only one motion at a time (default)
  end

  category :sensing do
    doc "Sensor and recording commands"
    concurrency_limit 2  # Up to 2 concurrent sensing operations
  end

  category :auxiliary do
    doc "LEDs, sounds, indicators"
    concurrency_limit 3  # Multiple concurrent auxiliary commands
  end

  # Commands specify their category
  command :move_to do
    handler MyMoveCommand
    category :motion
    allowed_states [:idle]
    cancel [:motion]  # Can cancel previous motion commands
  end

  command :record_frame do
    handler MyRecordCommand
    category :sensing
    allowed_states [:idle]
    # No cancel - concurrent sensing up to limit
  end

  command :set_led do
    handler MyLedCommand
    category :auxiliary
    allowed_states [:idle]
    # No cancel - concurrent auxiliary up to limit
  end
end
```

### How Categories Work

- Each category has a `concurrency_limit` (default: 1)
- Commands in a category run concurrently up to that limit
- Commands in **different** categories can run concurrently
- Commands without an explicit category use the `:default` category (limit: 1)

```elixir
# Start a motion command
{:ok, move_cmd} = MyRobot.move_to(target: position)

# While moving, start recording (different category - runs concurrently)
{:ok, record_cmd} = MyRobot.record_frame(sensor: :camera)

# Both commands are now running
BB.Robot.Runtime.executing_commands(MyRobot)
# => [
#   %{name: :move_to, category: :motion, pid: #PID<...>},
#   %{name: :record_frame, category: :sensing, pid: #PID<...>}
# ]
```

### Category Full Behaviour

When a category is at capacity, the behaviour depends on the `cancel` option:

1. If the command has `cancel` that includes the full category, it cancels commands to make room
2. Otherwise, the new command is rejected with `{:error, %BB.Error.Category.Full{}}`

```elixir
# Start a motion command
{:ok, cmd1} = MyRobot.move_to(target: pos1)

# Start another motion (same category, at limit)
{:ok, cmd2} = MyRobot.move_to(target: pos2)

# cmd1 is cancelled, cmd2 runs
# Because :move_to has cancel: [:motion]
```

The `cancel` option accepts:
- `:*` - cancels all categories
- `[:motion, :sensing]` - cancels specific categories
- `[]` (default) - cannot cancel, errors if category is full

## Introspection APIs

Query the execution state:

```elixir
# Is anything executing?
BB.Robot.Runtime.executing?(MyRobot)
# => true | false

# Is a specific category occupied?
BB.Robot.Runtime.executing?(MyRobot, :motion)
# => true | false

# List all running commands
BB.Robot.Runtime.executing_commands(MyRobot)
# => [%{name: :move_to, category: :motion, pid: #PID<...>, ...}]

# Get category availability
BB.Robot.Runtime.category_availability(MyRobot)
# => %{motion: {1, 1}, sensing: {0, 2}, default: {0, 1}}
#    Format: {current_count, limit}
```

## Compile-Time Validation

The DSL validates your state and category references at compile time:

```elixir
# This will produce a warning:
command :bad_cmd do
  handler MyHandler
  allowed_states [:nonexistent_state]  # Warning: undefined state
end

# This will also produce a warning:
command :bad_cmd do
  handler MyHandler
  category :nonexistent_category  # Warning: undefined category
end
```

## A Complete Example

Here's a robot that collects data while moving:

```elixir
defmodule DataCollectorArm do
  use BB

  states do
    state :recording do
      doc "Actively recording sensor data"
    end
  end

  commands do
    category :motion do
      concurrency_limit 1
    end

    category :data do
      concurrency_limit 1
    end

    command :arm do
      handler BB.Command.Arm
      allowed_states [:disarmed]
    end

    command :disarm do
      handler BB.Command.Disarm
      allowed_states [:idle, :recording]
      cancel :*  # Can cancel any running commands
    end

    command :enter_recording do
      handler {BB.Command.SetState, to: :recording}
      allowed_states [:idle]
    end

    command :exit_recording do
      handler {BB.Command.SetState, to: :idle}
      allowed_states [:recording]
    end

    command :move_to do
      handler MoveToCommand
      category :motion
      allowed_states [:idle, :recording]
      cancel [:motion]  # Can cancel previous motion commands
    end

    command :capture_frame do
      handler CaptureFrameCommand
      category :data
      allowed_states [:recording]
      cancel [:data]  # Can cancel previous capture commands
    end
  end

  topology do
    link :base do
      joint :shoulder do
        type :revolute
        axis do
        end
        limit do
          effort(~u(50 newton_meter))
          velocity(~u(2 radian_per_second))
        end
        link :arm
      end
    end
  end
end
```

Using it:

```elixir
# Start and arm
{:ok, _} = BB.Supervisor.start_link(DataCollectorArm)
{:ok, cmd} = DataCollectorArm.arm()
{:ok, :armed, _} = BB.Command.await(cmd)

# Enter recording mode
{:ok, cmd} = DataCollectorArm.enter_recording()
{:ok, :recording, _} = BB.Command.await(cmd)

# Now we can move AND capture frames concurrently
{:ok, move_cmd} = DataCollectorArm.move_to(position: 0.5)
{:ok, capture_cmd} = DataCollectorArm.capture_frame(sensor: :camera)

# Both commands run in parallel (different categories)
BB.Robot.Runtime.executing_commands(DataCollectorArm)
# => [%{name: :move_to, category: :motion}, %{name: :capture_frame, category: :data}]

# Wait for both
BB.Command.await(move_cmd)
BB.Command.await(capture_cmd)

# Exit recording mode
{:ok, cmd} = DataCollectorArm.exit_recording()
{:ok, :idle, _} = BB.Command.await(cmd)
```

## Best Practices

1. **Use states for operational modes**, not for tracking progress. A state like `:recording` is good; a state like `:step_3_of_5` is probably better handled inside a command.

2. **Keep category limits low**. High concurrency limits can make reasoning about robot behaviour difficult. Most categories should have limit 1.

3. **Validate state transitions**. Use `allowed_states` to ensure commands can only run in appropriate modes.

4. **Consider safety implications**. Can your robot safely run concurrent motions? Usually not - keep motion commands in a single category with limit 1.

5. **Use SetState for simple transitions**. Only implement custom command handlers when you need to do work during the transition.

## What's Next?

You now understand custom states and command categories. Continue exploring:

- [Parameters](07-parameters.md) for runtime-adjustable configuration
- [Safety](../topics/understanding-safety.md) for implementing safe hardware control
