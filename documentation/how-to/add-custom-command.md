<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# How to Add a Custom Command

Create a command handler that integrates with the robot state machine and provides structured feedback.

## Prerequisites

- Familiarity with the BB DSL (see [First Robot](../tutorials/01-first-robot.md))
- Understanding of the command system (see [Commands and State Machine](../tutorials/05-commands.md))

## Step 1: Define the Command in DSL

Add the command to your robot's `commands` block:

```elixir
defmodule MyRobot do
  use BB

  commands do
    command :arm do
      handler BB.Command.Arm
      allowed_states [:disarmed]
    end

    command :disarm do
      handler BB.Command.Disarm
      allowed_states [:idle]
    end

    command :move_to do
      handler MyRobot.MoveToCommand
      allowed_states [:idle]

      argument :target, {:map, :atom, :float} do
        required true
        doc "Target joint positions in radians"
      end

      argument :velocity, :float do
        required false
        default 1.0
        doc "Movement velocity multiplier"
      end
    end
  end

  topology do
    # ... your robot topology
  end
end
```

## Step 2: Create the Handler Module

Create a module using `BB.Command`:

```elixir
defmodule MyRobot.MoveToCommand do
  use BB.Command

  alias BB.Message.Sensor.JointCommand
  alias BB.PubSub

  @impl BB.Command
  def handle_command(goal, context, state) do
    target = Map.fetch!(goal, :target)
    velocity = Map.get(goal, :velocity, 1.0)

    # Subscribe to sensor feedback
    for {joint_name, _position} <- target do
      PubSub.subscribe(context.robot_module, [:sensor, joint_name])
    end

    # Send commands to actuators
    for {joint_name, position} <- target do
      command = JointCommand.new!(name: joint_name, target: position)
      PubSub.publish(context.robot_module, [:actuator, joint_name], command)
    end

    # Store target and wait for completion
    {:noreply, Map.merge(state, %{
      target: target,
      velocity: velocity,
      positions: %{}
    })}
  end

  @impl BB.Command
  def handle_info({:bb, [:sensor, joint_name], %{payload: joint_state}}, state) do
    current = hd(joint_state.positions)
    target = Map.get(state.target, joint_name)

    if target && close_enough?(current, target) do
      new_positions = Map.put(state.positions, joint_name, current)

      if all_complete?(new_positions, state.target) do
        {:stop, :normal, %{state | result: {:ok, new_positions}}}
      else
        {:noreply, %{state | positions: new_positions}}
      end
    else
      {:noreply, state}
    end
  end

  def handle_info(_msg, state), do: {:noreply, state}

  @impl BB.Command
  def result(%{result: result}), do: result

  defp close_enough?(current, target), do: abs(current - target) < 0.01

  defp all_complete?(positions, target) do
    Enum.all?(target, fn {joint, _} -> Map.has_key?(positions, joint) end)
  end
end
```

## Step 3: Use the Command

The DSL generates a convenience function on your robot module:

```elixir
# Start the robot
{:ok, _} = BB.Supervisor.start_link(MyRobot)

# Arm first (commands need :idle state)
{:ok, cmd} = MyRobot.arm()
{:ok, :armed, _} = BB.Command.await(cmd)

# Execute your command
{:ok, cmd} = MyRobot.move_to(target: %{shoulder: 0.5, elbow: 1.0})

# Wait for completion
case BB.Command.await(cmd, 10_000) do
  {:ok, positions} ->
    IO.puts("Moved to: #{inspect(positions)}")

  {:error, reason} ->
    IO.puts("Movement failed: #{inspect(reason)}")
end
```

## Adding Timeout Handling

For commands that might hang, add timeout logic:

```elixir
defmodule MyRobot.MoveToCommand do
  use BB.Command

  @timeout_ms 5000

  @impl BB.Command
  def handle_command(goal, context, state) do
    # ... setup code ...

    # Schedule timeout
    timer_ref = Process.send_after(self(), :timeout, @timeout_ms)

    {:noreply, Map.put(state, :timer_ref, timer_ref)}
  end

  @impl BB.Command
  def handle_info(:timeout, state) do
    {:stop, :normal, %{state | result: {:error, :timeout}}}
  end

  def handle_info({:bb, [:sensor, _], _} = msg, state) do
    # Cancel timeout on any progress
    if state[:timer_ref] do
      Process.cancel_timer(state.timer_ref)
    end

    # ... existing sensor handling ...

    {:noreply, Map.put(state, :timer_ref, new_timer_ref)}
  end
end
```

## Handling Safety State Changes

React to safety transitions during execution:

```elixir
@impl BB.Command
def handle_safety_state_change(:disarming, state) do
  # Robot is being disarmed - stop gracefully
  {:stop, :disarmed, %{state | result: {:error, :disarmed}}}
end

def handle_safety_state_change(_new_state, state) do
  # Continue execution (use with care!)
  {:continue, state}
end
```

The default implementation stops with `:disarmed` on any safety state change.

## Command Cancellation

Allow your command to be cancelled by other commands:

```elixir
command :move_to do
  handler MyRobot.MoveToCommand
  allowed_states [:idle]
  cancel [:default]  # Can be cancelled by other :default commands
end

command :emergency_stop do
  handler MyRobot.EmergencyStopCommand
  allowed_states :*     # Run in any state
  cancel :*             # Cancel all running commands
end
```

When cancelled, awaiting callers receive `{:error, :cancelled}`.

## State Transitions

Commands can transition the robot to a new state:

```elixir
@impl BB.Command
def result(%{result: {:ok, value}, next_state: next_state}) do
  {:ok, value, next_state: next_state}
end
```

This is how `BB.Command.Arm` and `BB.Command.Disarm` work - they set `next_state` to `:idle` and `:disarmed` respectively.

## Structured Errors

Return structured errors from `BB.Error`:

```elixir
alias BB.Error.State.NotAllowed

@impl BB.Command
def handle_command(goal, context, state) do
  case validate_goal(goal, context) do
    :ok ->
      # proceed
      {:noreply, state}

    {:error, reason} ->
      {:stop, :normal, %{state | result: {:error, reason}}}
  end
end

defp validate_goal(goal, context) do
  target = goal[:target] || %{}
  joints = Map.keys(context.robot.joints)

  invalid = Map.keys(target) -- joints
  if invalid == [] do
    :ok
  else
    {:error, BB.Error.Invalid.UnknownJoints.exception(joints: invalid)}
  end
end
```

## Testing Commands

Test command handlers with the robot in simulation mode:

```elixir
defmodule MyRobot.MoveToCommandTest do
  use ExUnit.Case

  setup do
    {:ok, _} = BB.Supervisor.start_link(MyRobot, simulation: :kinematic)
    {:ok, cmd} = MyRobot.arm()
    {:ok, :armed, _} = BB.Command.await(cmd)
    :ok
  end

  test "moves to target positions" do
    {:ok, cmd} = MyRobot.move_to(target: %{shoulder: 0.5})
    assert {:ok, %{shoulder: position}} = BB.Command.await(cmd, 5000)
    assert_in_delta position, 0.5, 0.02
  end

  test "returns error for invalid joints" do
    {:ok, cmd} = MyRobot.move_to(target: %{nonexistent: 0.5})
    assert {:error, %BB.Error.Invalid.UnknownJoints{}} = BB.Command.await(cmd)
  end
end
```

## Common Issues

### Command not starting

Check that:
- The robot is in one of the `allowed_states` for the command
- The command handler module is compiled and available

### Command hangs forever

Ensure you:
- Call `{:stop, reason, state}` when complete
- Handle timeout cases
- Subscribe to the correct PubSub paths for feedback

### State transition not working

The `result/1` callback must return `{:ok, value, next_state: state}` - the third element must be a keyword list with `:next_state`.

## Next Steps

- Learn about [Custom States and Command Categories](../tutorials/11-custom-states.md) for advanced state machines
- Understand the [Command System](../topics/command-system.md) architecture
