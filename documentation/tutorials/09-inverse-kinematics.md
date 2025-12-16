<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Inverse Kinematics

In this tutorial, you'll learn how to compute joint angles from target positions using the FABRIK inverse kinematics solver.

## Prerequisites

Complete [Forward Kinematics](04-kinematics.md). You should understand how joint angles map to link positions.

## What is Inverse Kinematics?

> **For Elixirists:** Inverse kinematics is the opposite of forward kinematics. Instead of asking "where is my end-effector given these joint angles?", we ask "what joint angles do I need to reach this target position?"

Inverse kinematics (IK) is fundamental for robot control:

- **Input:** Target position in 3D space (x, y, z)
- **Output:** Joint angles that position the end-effector at that target

This is harder than forward kinematics because:
1. There may be multiple solutions (or none)
2. The equations are often non-linear
3. Joint limits must be respected

## Installing bb_ik_fabrik

Add the FABRIK solver to your dependencies:

```elixir
def deps do
  [
    {:bb, "~> 0.1.0"},
    {:bb_ik_fabrik, "~> 0.1.0"}
  ]
end
```

FABRIK (Forward And Backward Reaching Inverse Kinematics) is an iterative algorithm that works well for serial chains like robot arms.

## Basic Usage

```elixir
alias BB.IK.FABRIK
alias BB.Robot.State

# Get your robot
robot = MyRobot.robot()
{:ok, state} = State.new(robot)

# Define where you want the end-effector to go
target = {0.3, 0.2, 0.1}

# Solve for joint angles
case FABRIK.solve(robot, state, :end_effector_link, target) do
  {:ok, positions, meta} ->
    IO.puts("Solved in #{meta.iterations} iterations")
    IO.puts("Distance to target: #{Float.round(meta.residual * 1000, 2)}mm")

    # Apply the solution
    State.set_positions(state, positions)

  {:error, :unreachable, meta} ->
    IO.puts("Target is out of reach")
    IO.puts("Best distance achieved: #{Float.round(meta.residual * 1000, 2)}mm")
end
```

## Understanding the Result

The solver returns a `meta` map with useful information:

| Field | Description |
|-------|-------------|
| `iterations` | Number of FABRIK iterations performed |
| `residual` | Distance from end-effector to target (metres) |
| `reached` | Boolean - did we converge within tolerance? |
| `reason` | `:converged`, `:unreachable`, `:max_iterations`, etc. |

On error, `meta` also contains `:positions` with the best-effort joint values.

## Practical Example: Moving to a Target

Here's a complete example that solves IK and verifies the result with forward kinematics:

```elixir
defmodule IKDemo do
  alias BB.IK.FABRIK
  alias BB.Robot.{Kinematics, State}

  def move_to_target(robot, state, target_link, target) do
    # Solve IK
    case FABRIK.solve(robot, state, target_link, target) do
      {:ok, positions, meta} ->
        # Apply the solution
        State.set_positions(state, positions)

        # Verify with forward kinematics
        {x, y, z} = Kinematics.link_position(robot, positions, target_link)

        IO.puts("Target:   #{format_point(target)}")
        IO.puts("Achieved: #{format_point({x, y, z})}")
        IO.puts("Error:    #{Float.round(meta.residual * 1000, 2)}mm")

        {:ok, positions}

      {:error, reason, _meta} ->
        {:error, reason}
    end
  end

  defp format_point({x, y, z}) do
    "(#{Float.round(x, 3)}, #{Float.round(y, 3)}, #{Float.round(z, 3)})"
  end
end

# Usage
robot = MyRobot.robot()
{:ok, state} = BB.Robot.State.new(robot)
IKDemo.move_to_target(robot, state, :tip, {0.3, 0.2, 0.0})
```

## Solver Options

Fine-tune the solver behaviour with options:

```elixir
FABRIK.solve(robot, state, :end_effector, target,
  max_iterations: 100,    # Default: 50
  tolerance: 0.001,       # Default: 1.0e-4 (0.1mm)
  respect_limits: true    # Default: true
)
```

### When to Adjust Options

- **Increase `max_iterations`** if the solver returns `:max_iterations` but is getting close
- **Increase `tolerance`** if you don't need sub-millimetre precision
- **Set `respect_limits: false`** to see what the "ideal" solution would be (useful for debugging)

## Handling Unreachable Targets

Not all targets can be reached. The solver handles this gracefully:

```elixir
# Target way beyond the robot's reach
target = {10.0, 0.0, 0.0}

case FABRIK.solve(robot, state, :tip, target) do
  {:error, :unreachable, meta} ->
    # The solver stretched the arm as far as possible
    # meta.positions contains the best-effort joint angles
    IO.puts("Target unreachable")
    IO.puts("Best distance: #{meta.residual}m")

    # You might still want to use the best-effort result
    # to point the arm in the right direction
    State.set_positions(state, meta.positions)
end
```

## Target Formats

The solver accepts several target formats:

```elixir
# Position tuple (most common)
target = {0.3, 0.2, 0.1}

# Nx tensor
target = Nx.tensor([0.3, 0.2, 0.1])

# 4x4 homogeneous transform (position extracted)
target = BB.Robot.Transform.translation(0.3, 0.2, 0.1)
```

> **Note:** FABRIK currently solves for position only. Orientation in transforms is ignored.

## Using solve_and_update/5

For convenience, `solve_and_update/5` solves and applies the result in one call:

```elixir
case FABRIK.solve_and_update(robot, state, :tip, target) do
  {:ok, positions, meta} ->
    # State has already been updated
    IO.puts("Moved to target")

  {:error, reason, _meta} ->
    # State is unchanged on error
    IO.puts("Failed: #{reason}")
end
```

## Motion Integration

The `BB.Motion` module bridges IK solving with actuator commands, making it easy to move your robot to target positions. Use it directly or through `BB.IK.FABRIK.Motion` for FABRIK-specific convenience.

### Moving to a Target

```elixir
alias BB.Motion

# Start your robot
{:ok, _pid} = MyRobot.start_link([])

# Move the end-effector to a target position
case Motion.move_to(MyRobot, :tip, {0.3, 0.2, 0.1}, solver: BB.IK.FABRIK) do
  {:ok, meta} ->
    IO.puts("Moved in #{meta.iterations} iterations")

  {:error, reason, meta} ->
    IO.puts("Failed: #{reason}, best residual: #{meta.residual}")
end
```

This solves IK, updates the robot state, and sends position commands to all actuators.

### Using FABRIK Convenience Functions

`BB.IK.FABRIK.Motion` provides defaults for common options:

```elixir
alias BB.IK.FABRIK.Motion

# Same as above but pre-configured for FABRIK
case Motion.move_to(MyRobot, :tip, {0.3, 0.2, 0.1}) do
  {:ok, meta} -> :moved
  {:error, _, _} -> :failed
end

# Just solve without moving (for validation)
case Motion.solve(MyRobot, :tip, {0.3, 0.2, 0.1}) do
  {:ok, positions, _meta} -> IO.inspect(positions, label: "Would set")
  {:error, _, _} -> :unreachable
end
```

### Multi-Target Motion

For coordinated motion (like walking gaits), solve multiple targets simultaneously:

```elixir
targets = %{
  left_foot: {0.1, 0.0, 0.0},
  right_foot: {-0.1, 0.0, 0.0}
}

case Motion.move_to_multi(MyRobot, targets, solver: BB.IK.FABRIK) do
  {:ok, results} ->
    Enum.each(results, fn {link, {:ok, _pos, meta}} ->
      IO.puts("#{link}: #{meta.iterations} iterations")
    end)

  {:error, failed_link, reason, _results} ->
    IO.puts("#{failed_link} failed: #{reason}")
end
```

Targets are solved in parallel using `Task.async_stream` for efficiency.

### Continuous Tracking

For following moving targets (e.g., visual tracking), use `BB.IK.FABRIK.Tracker`:

```elixir
alias BB.IK.FABRIK.Tracker

# Start tracking
{:ok, tracker} = Tracker.start_link(
  robot: MyRobot,
  target_link: :gripper,
  initial_target: {0.3, 0.2, 0.1},
  update_rate: 30   # Hz
)

# Update target (from vision callback, etc.)
Tracker.update_target(tracker, {0.35, 0.25, 0.15})

# Check status
%{residual: residual, tracking: true} = Tracker.status(tracker)

# Stop when done
{:ok, final_positions} = Tracker.stop(tracker)
```

The tracker runs a continuous solve loop at the specified rate, sending actuator commands on each successful solve.

### In Custom Commands

When implementing custom commands, use Motion with the command context:

```elixir
defmodule MyRobot.Commands.Reach do
  @behaviour BB.Command

  @impl true
  def handle_command(%{target: target}, context) do
    case BB.Motion.move_to(context, :gripper, target, solver: BB.IK.FABRIK) do
      {:ok, meta} ->
        {:ok, %{residual: meta.residual, iterations: meta.iterations}}

      {:error, reason, _meta} ->
        {:error, {:ik_failed, reason}}
    end
  end
end
```

## Working with Joint Limits

By default, the solver respects joint limits defined in your robot:

```elixir
topology do
  link :base do
    joint :shoulder do
      type(:revolute)
      limit do
        lower(~u(-90 degree))
        upper(~u(90 degree))
      end
      # ...
    end
  end
end
```

The solver will clamp joint values to these limits, which may prevent reaching some targets even if they're geometrically possible.

To see the unconstrained solution:

```elixir
{:ok, unconstrained, _} = FABRIK.solve(robot, state, :tip, target, respect_limits: false)
{:ok, constrained, _} = FABRIK.solve(robot, state, :tip, target, respect_limits: true)

# Compare the solutions
IO.inspect(unconstrained, label: "Without limits")
IO.inspect(constrained, label: "With limits")
```

## Limitations

FABRIK works well for many use cases but has some limitations:

1. **Position only** - It solves for end-effector position, not orientation
2. **Serial chains** - It assumes a single chain from base to end-effector (no branching)
3. **Collinear targets** - Can struggle when the target is directly in line with a straight chain

For more complex requirements, consider implementing a different solver using the `BB.IK.Solver` behaviour.

## What's Next?

You now know how to:
- Compute joint angles for target positions
- Handle unreachable targets gracefully
- Fine-tune solver parameters
- Work with joint limits
- Use the Motion API to send actuator commands
- Track moving targets with the Tracker

Inverse kinematics combined with Motion provides a complete solution for position-based robot control. Use these primitives to build higher-level behaviours like gait generators, pick-and-place routines, or visual servoing systems.
