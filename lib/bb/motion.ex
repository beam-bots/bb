# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Motion do
  @moduledoc """
  High-level motion primitives that bridge IK solving and actuator commands.

  This module provides functions for moving robot end-effectors to target
  positions using pluggable IK solvers. It handles the full workflow:
  solving IK, updating robot state, and sending commands to actuators.

  ## Usage

  Single-target functions:
  - `move_to/4` - Solve IK for one target, update state, send actuator commands
  - `solve_only/4` - Solve IK without sending commands (for planning/validation)

  Multi-target functions (for coordinated motion like gait):
  - `move_to_multi/3` - Solve IK for multiple targets simultaneously
  - `solve_only_multi/3` - Solve multiple targets without sending commands

  Utility:
  - `send_positions/3` - Send pre-computed positions to actuators (bypasses IK)

  ## Context Sources

  Functions accept either:
  - A robot module (uses Runtime to get robot and state)
  - A `BB.Command.Context` struct (uses context fields directly)

  The second form is useful when implementing custom commands that need
  to perform IK-based motion.

  ## Examples

      # Single target
      case BB.Motion.move_to(MyRobot, :gripper, {0.3, 0.2, 0.1},
             source_link: :base_link, solver: BB.IK.FABRIK) do
        {:ok, meta} -> IO.puts("Reached target in \#{meta.iterations} iterations")
        {:error, %{class: :kinematics} = error} -> IO.puts("Failed: \#{BB.Error.message(error)}")
      end

      # Multiple targets (for gait, coordinated motion)
      targets = %{left_foot: {0.1, 0.0, 0.0}, right_foot: {-0.1, 0.0, 0.0}}
      case BB.Motion.move_to_multi(MyRobot, targets,
             source_link: :body, solver: BB.IK.FABRIK) do
        {:ok, results} -> IO.puts("All targets reached")
        {:error, error} -> IO.puts("Failed: \#{BB.Error.message(error)}")
      end

      # In a custom command handler
      def handle_command(%{target: target}, context) do
        case BB.Motion.move_to(context, :gripper, target,
               source_link: :base_link, solver: BB.IK.FABRIK) do
          {:ok, meta} -> {:ok, %{residual: meta.residual}}
          {:error, error} -> {:error, error}
        end
      end

      # Just solve without moving (for validation)
      case BB.Motion.solve_only(MyRobot, :gripper, {0.3, 0.2, 0.1},
             source_link: :base_link, solver: BB.IK.FABRIK) do
        {:ok, positions, meta} -> IO.inspect(positions, label: "Would set")
        {:error, %BB.Error.Kinematics.Unreachable{}} -> IO.puts("Cannot reach target")
      end

      # Send pre-computed positions
      positions = %{shoulder: 0.5, elbow: 1.2}
      :ok = BB.Motion.send_positions(MyRobot, positions, delivery: :direct)
  """

  alias BB.Actuator
  alias BB.Command.Context
  alias BB.Error.Kinematics.MultiFailed
  alias BB.IK.Solver
  alias BB.Robot.Runtime
  alias BB.Robot.State, as: RobotState

  @type target :: Solver.target()
  @type positions :: Solver.positions()
  @type meta :: Solver.meta()
  @type kinematics_error :: Solver.kinematics_error()
  @type robot_or_context :: module() | Context.t()
  @type delivery :: :pubsub | :direct
  @type targets :: %{atom() => target()}
  @type multi_results ::
          %{atom() => {:ok, positions(), meta()} | {:error, kinematics_error()}}

  @typedoc """
  Why a motion stopped: either the solver couldn't reach the target, or an
  actuator refused the command it was sent.
  """
  @type motion_error :: kinematics_error() | BB.Error.t()

  @type motion_result :: {:ok, meta()} | {:error, motion_error()}
  @type solve_result :: {:ok, positions(), meta()} | {:error, kinematics_error()}
  @type multi_motion_result :: {:ok, multi_results()} | {:error, motion_error()}
  @type multi_solve_result :: {:ok, multi_results()} | {:error, MultiFailed.t()}

  @doc """
  Move an end-effector to a target position.

  Solves inverse kinematics for the given target, updates the robot state,
  and sends position commands to all actuators controlling the affected joints.

  ## Options

  Required:
  - `:solver` - Module implementing `BB.IK.Solver` behaviour
  - `:source_link` - The link the chain starts at. No default: the root is right
    for a fixed-base arm and silently wrong for a robot whose base floats, so
    pass `BB.Robot.root_link(robot)` when you do mean the whole tree

  Optional:
  - `:delivery` - How to send actuator commands. `:pubsub` (default) publishes
    each command and waits for the actuator to accept it, reporting the first
    refusal; `:direct` casts to each actuator and waits for nothing, so a
    refusal is never reported
  - `:velocity` - Velocity hint (passed to actuators)
  - `:duration` - Duration hint in milliseconds (passed to actuators)
  - `:command_id` - Correlation ID for feedback tracking (passed to actuators)
  - `:timeout` - How long to wait for each actuator to accept its command, in
    milliseconds (default 5000). Unused under `:direct`, which waits for
    nothing. A timeout exits the caller, as `GenServer.call/3` does — a loop
    that would rather skip a late step than die wants `:direct`
  - `:max_iterations` - Maximum solver iterations (passed to solver)
  - `:tolerance` - Convergence tolerance in metres (passed to solver)
  - `:respect_limits` - Whether to clamp to joint limits (passed to solver)

  ## Returns

  - `{:ok, meta}` - Successfully moved; meta contains solver info (iterations, residual, etc.)
  - `{:error, error}` - Failed; either a struct from `BB.Error.Kinematics` if
    the target couldn't be solved, or the actuator's own error if one refused
    the command it was sent. The robot state is updated either way: the
    positions were computed, and some joints may have taken them

  ## Examples

      BB.Motion.move_to(MyRobot, :gripper, {0.3, 0.2, 0.1},
        source_link: :base_link,
        solver: BB.IK.FABRIK
      )

      BB.Motion.move_to(context, :gripper, target,
        source_link: :base_link,
        solver: BB.IK.FABRIK,
        delivery: :direct,
        max_iterations: 100
      )
  """
  @spec move_to(robot_or_context(), atom(), target(), keyword()) :: motion_result()
  def move_to(robot_or_context, target_link, target, opts) do
    solver = Keyword.fetch!(opts, :solver)
    source_link = Keyword.fetch!(opts, :source_link)
    delivery = Keyword.get(opts, :delivery, :pubsub)
    solver_opts = extract_solver_opts(opts)
    actuator_opts = extract_actuator_opts(opts)

    {robot_module, robot, robot_state} = extract_context(robot_or_context)

    :telemetry.span(
      [:bb, :motion, :move_to],
      %{robot: robot.name, target_link: target_link, solver: solver},
      fn ->
        with {:ok, positions, meta} <-
               solver.solve(robot, robot_state, source_link, target_link, target, solver_opts),
             RobotState.set_configurations(robot_state, positions),
             :ok <-
               send_positions_to_actuators(
                 robot_module,
                 robot,
                 positions,
                 delivery,
                 actuator_opts
               ) do
          extra_meta = %{
            iterations: meta.iterations,
            residual: meta.residual,
            reached: meta.reached
          }

          {{:ok, meta}, extra_meta}
        else
          {:error, error} -> {{:error, error}, %{error: error.__struct__}}
        end
      end
    )
  end

  @doc """
  Solve IK without moving the robot.

  Useful for:
  - Validating that a target is reachable before committing
  - Planning multi-step motions
  - Visualising solutions before execution

  ## Options

  Same as `move_to/4` except `:delivery` is not used.

  ## Returns

  - `{:ok, positions, meta}` - Successfully solved; positions is a joint name → value map
  - `{:error, error}` - Failed to solve; error is a struct from `BB.Error.Kinematics`

  ## Examples

      # Check if target is reachable
      case BB.Motion.solve_only(MyRobot, :gripper, target,
             source_link: :base_link, solver: BB.IK.FABRIK) do
        {:ok, _positions, %{reached: true}} -> :reachable
        {:error, _} -> :unreachable
      end
  """
  @spec solve_only(robot_or_context(), atom(), target(), keyword()) :: solve_result()
  def solve_only(robot_or_context, target_link, target, opts) do
    solver = Keyword.fetch!(opts, :solver)
    source_link = Keyword.fetch!(opts, :source_link)
    solver_opts = extract_solver_opts(opts)

    {_robot_module, robot, robot_state} = extract_context(robot_or_context)

    :telemetry.span(
      [:bb, :motion, :solve],
      %{robot: robot.name, target_link: target_link, solver: solver},
      fn ->
        case solver.solve(robot, robot_state, source_link, target_link, target, solver_opts) do
          {:ok, _positions, meta} = result ->
            extra_meta = %{
              iterations: meta.iterations,
              residual: meta.residual,
              reached: meta.reached
            }

            {result, extra_meta}

          {:error, error} = result ->
            extra_meta = %{error: error.__struct__}
            {result, extra_meta}
        end
      end
    )
  end

  @doc """
  Move multiple end-effectors to target positions simultaneously.

  Useful for coordinated motion like walking gaits where multiple limbs
  must move together. Each target is solved independently and all actuator
  commands are sent together.

  If any target fails to solve, the operation stops and returns an error
  with information about which target failed. Targets solved before the
  failure are included in the results.

  ## Options

  Required:
  - `:solver` - Module implementing `BB.IK.Solver` behaviour
  - `:source_link` - The link the chain starts at. No default: the root is right
    for a fixed-base arm and silently wrong for a robot whose base floats, so
    pass `BB.Robot.root_link(robot)` when you do mean the whole tree

  Optional:
  - `:delivery` - How to send actuator commands. `:pubsub` (default) publishes
    each command and waits for the actuator to accept it, reporting the first
    refusal; `:direct` casts to each actuator and waits for nothing, so a
    refusal is never reported
  - `:velocity` - Velocity hint (passed to actuators)
  - `:duration` - Duration hint in milliseconds (passed to actuators)
  - `:command_id` - Correlation ID for feedback tracking (passed to actuators)
  - `:timeout` - How long to wait for each actuator to accept its command, in
    milliseconds (default 5000). Unused under `:direct`, which waits for
    nothing. A timeout exits the caller, as `GenServer.call/3` does — a loop
    that would rather skip a late step than die wants `:direct`
  - `:max_iterations` - Maximum solver iterations (passed to solver)
  - `:tolerance` - Convergence tolerance in metres (passed to solver)
  - `:respect_limits` - Whether to clamp to joint limits (passed to solver)

  ## Returns

  - `{:ok, results}` - All targets solved; results is a map of link → `{:ok, positions, meta}`
  - `{:error, %BB.Error.Kinematics.MultiFailed{}}` - A target failed to solve.
    The error names the link that failed, carries the underlying kinematics
    error, and keeps the results of the targets solved before it
  - `{:error, error}` - Every target solved, but an actuator refused the command
    it was sent. That failure isn't kinematic, so it arrives as the actuator's
    own error rather than wrapped in `MultiFailed`

  ## Examples

      targets = %{
        left_foot: {0.1, 0.0, 0.0},
        right_foot: {-0.1, 0.0, 0.0}
      }

      case BB.Motion.move_to_multi(MyRobot, targets,
             source_link: :body, solver: BB.IK.FABRIK) do
        {:ok, results} ->
          IO.puts("All targets reached")

        {:error, %MultiFailed{failed_link: link} = error} ->
          IO.puts("Failed to reach \#{link}: \#{BB.Error.message(error)}")

        {:error, error} ->
          IO.puts("An actuator refused: \#{BB.Error.message(error)}")
      end
  """
  @spec move_to_multi(robot_or_context(), targets(), keyword()) :: multi_motion_result()
  def move_to_multi(robot_or_context, targets, opts) do
    with {:ok, results} <- solve_only_multi(robot_or_context, targets, opts) do
      delivery = Keyword.get(opts, :delivery, :pubsub)
      actuator_opts = extract_actuator_opts(opts)
      {robot_module, robot, robot_state} = extract_context(robot_or_context)

      all_positions = merge_all_positions(results)
      RobotState.set_configurations(robot_state, all_positions)

      case send_positions_to_actuators(
             robot_module,
             robot,
             all_positions,
             delivery,
             actuator_opts
           ) do
        :ok ->
          {:ok, results}

        # Not wrapped in `MultiFailed`: every target solved, and the command was
        # refused on the way out, so nothing about this failure is kinematic.
        {:error, error} ->
          {:error, error}
      end
    end
  end

  @doc """
  Solve IK for multiple targets without moving the robot.

  Useful for validating that a set of coordinated targets are all reachable
  before committing to motion.

  ## Options

  Same as `move_to_multi/3` except `:delivery` is not used.

  ## Returns

  - `{:ok, results}` - All targets solved; results is a map of link → `{:ok, positions, meta}`
  - `{:error, %BB.Error.Kinematics.MultiFailed{}}` - A target failed. The error
    names the link that failed, carries the underlying kinematics error, and
    keeps the results of the targets solved before it

  ## Examples

      targets = %{left_foot: {0.1, 0.0, 0.0}, right_foot: {-0.1, 0.0, 0.0}}

      case BB.Motion.solve_only_multi(MyRobot, targets,
             source_link: :body, solver: BB.IK.FABRIK) do
        {:ok, results} ->
          Enum.each(results, fn {link, {:ok, _positions, meta}} ->
            IO.puts("\#{link}: residual=\#{meta.residual}")
          end)

        {:error, %MultiFailed{failed_link: link} = error} ->
          IO.puts("\#{link} is unreachable: \#{BB.Error.message(error)}")
      end
  """
  @spec solve_only_multi(robot_or_context(), targets(), keyword()) :: multi_solve_result()
  def solve_only_multi(robot_or_context, targets, opts) do
    solver = Keyword.fetch!(opts, :solver)
    source_link = Keyword.fetch!(opts, :source_link)
    solver_opts = extract_solver_opts(opts)

    {_robot_module, robot, robot_state} = extract_context(robot_or_context)

    targets
    |> Task.async_stream(fn {target_link, target} ->
      {target_link,
       solver.solve(robot, robot_state, source_link, target_link, target, solver_opts)}
    end)
    |> Enum.reduce_while({:ok, %{}}, fn
      {:ok, {link, {:ok, _, _} = result}}, {:ok, results} ->
        {:cont, {:ok, Map.put(results, link, result)}}

      {:ok, {link, {:error, error} = result}}, {:ok, results} ->
        {:halt,
         {:error,
          MultiFailed.exception(
            failed_link: link,
            error: error,
            partial_results: Map.put(results, link, result)
          )}}
    end)
  end

  defp merge_all_positions(results) do
    Enum.reduce(results, %{}, fn {_link, {:ok, positions, _meta}}, acc ->
      Map.merge(acc, positions)
    end)
  end

  @doc """
  Send pre-computed joint positions to actuators.

  Bypasses IK solving entirely - useful when you've already computed
  positions through other means (e.g., trajectory planning, manual input).

  Updates the robot state and sends commands to all actuators controlling
  the specified joints.

  ## Options

  - `:delivery` - How to send actuator commands. `:pubsub` (default) publishes
    each command and waits for the actuator to accept it, reporting the first
    refusal; `:direct` casts to each actuator and waits for nothing, so a
    refusal is never reported
  - `:velocity` - Velocity hint for actuators (rad/s or m/s)
  - `:duration` - Duration hint for actuators (milliseconds)
  - `:command_id` - Correlation ID for feedback tracking
  - `:timeout` - How long to wait for each actuator to accept its command, in
    milliseconds (default 5000). Unused under `:direct`, which waits for
    nothing. A timeout exits the caller, as `GenServer.call/3` does — a loop
    that would rather skip a late step than die wants `:direct`

  ## Returns

  - `:ok` - Every actuator accepted its command
  - `{:error, error}` - One refused; the rest were still sent, and the robot
    state was updated regardless

  ## Examples

      positions = %{shoulder: 0.5, elbow: 1.2, wrist: 0.3}
      :ok = BB.Motion.send_positions(MyRobot, positions)

      # With direct delivery for lower latency
      :ok = BB.Motion.send_positions(MyRobot, positions, delivery: :direct)
  """
  @spec send_positions(robot_or_context(), positions(), keyword()) ::
          :ok | {:error, motion_error()}
  def send_positions(robot_or_context, positions, opts \\ []) do
    delivery = Keyword.get(opts, :delivery, :pubsub)
    actuator_opts = extract_actuator_opts(opts)

    {robot_module, robot, robot_state} = extract_context(robot_or_context)

    :telemetry.span(
      [:bb, :motion, :send_positions],
      %{robot: robot.name, joint_count: map_size(positions), delivery: delivery},
      fn ->
        RobotState.set_configurations(robot_state, positions)

        result =
          send_positions_to_actuators(robot_module, robot, positions, delivery, actuator_opts)

        {result, %{}}
      end
    )
  end

  defp extract_context(%Context{} = context) do
    {context.robot_module, context.robot, context.robot_state}
  end

  defp extract_context(robot_module) when is_atom(robot_module) do
    robot = Runtime.get_robot(robot_module)
    robot_state = Runtime.get_robot_state(robot_module)
    {robot_module, robot, robot_state}
  end

  @actuator_opts [:delivery, :velocity, :duration, :command_id, :timeout]

  defp extract_solver_opts(opts) do
    opts
    |> Keyword.drop([:solver, :source_link | @actuator_opts])
    |> Keyword.reject(fn {_k, v} -> is_nil(v) end)
  end

  defp extract_actuator_opts(opts) do
    opts
    |> Keyword.take(@actuator_opts)
    |> Keyword.reject(fn {_k, v} -> is_nil(v) end)
  end

  defp send_positions_to_actuators(robot_module, robot, positions, delivery, opts) do
    positions
    |> Enum.flat_map(&actuators_for_joint(robot, &1))
    |> send_to_actuators(robot_module, robot, delivery, opts)
  end

  defp actuators_for_joint(robot, {joint_name, position}) do
    case Map.get(robot.joints, joint_name) do
      nil -> []
      joint -> Enum.map(joint.actuators, &{&1, position})
    end
  end

  # The joints of one motion are meant to move together, so they are commanded
  # together: waiting on each in turn would cost a round trip per joint, and
  # nothing downstream is ordered by the order they were sent in.
  defp send_to_actuators(commands, robot_module, robot, :pubsub, opts) do
    commands
    |> Enum.map(fn {actuator_name, position} ->
      # The name came from the joint's own actuator list, so the lookup cannot miss.
      {:ok, path} = BB.Robot.actuator_path(robot, actuator_name)
      Task.async(fn -> Actuator.set_position(robot_module, path, position, opts) end)
    end)
    |> Task.await_many(:infinity)
    |> first_refusal()
  end

  defp send_to_actuators(commands, robot_module, _robot, :direct, opts) do
    Enum.each(commands, fn {actuator_name, position} ->
      Actuator.set_position(robot_module, actuator_name, position, opts)
    end)
  end

  defp first_refusal(results) do
    case Enum.find(results, &match?({:error, _}, &1)) do
      nil -> :ok
      {:error, error} -> {:error, error}
    end
  end
end
