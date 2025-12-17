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
      case BB.Motion.move_to(MyRobot, :gripper, {0.3, 0.2, 0.1}, solver: BB.IK.FABRIK) do
        {:ok, meta} -> IO.puts("Reached target in \#{meta.iterations} iterations")
        {:error, reason, _meta} -> IO.puts("Failed: \#{reason}")
      end

      # Multiple targets (for gait, coordinated motion)
      targets = %{left_foot: {0.1, 0.0, 0.0}, right_foot: {-0.1, 0.0, 0.0}}
      case BB.Motion.move_to_multi(MyRobot, targets, solver: BB.IK.FABRIK) do
        {:ok, results} -> IO.puts("All targets reached")
        {:error, failed_link, reason, results} -> IO.puts("Failed: \#{failed_link}")
      end

      # In a custom command handler
      def handle_command(%{target: target}, context) do
        case BB.Motion.move_to(context, :gripper, target, solver: BB.IK.FABRIK) do
          {:ok, meta} -> {:ok, %{residual: meta.residual}}
          {:error, reason, _meta} -> {:error, reason}
        end
      end

      # Just solve without moving (for validation)
      case BB.Motion.solve_only(MyRobot, :gripper, {0.3, 0.2, 0.1}, solver: BB.IK.FABRIK) do
        {:ok, positions, meta} -> IO.inspect(positions, label: "Would set")
        {:error, reason, _meta} -> IO.puts("Cannot reach: \#{reason}")
      end

      # Send pre-computed positions
      positions = %{shoulder: 0.5, elbow: 1.2}
      :ok = BB.Motion.send_positions(MyRobot, positions, delivery: :direct)
  """

  alias BB.Actuator
  alias BB.Command.Context
  alias BB.Robot.Runtime
  alias BB.Robot.State, as: RobotState

  @type target :: BB.IK.Solver.target()
  @type positions :: BB.IK.Solver.positions()
  @type meta :: BB.IK.Solver.meta()
  @type robot_or_context :: module() | Context.t()
  @type delivery :: :pubsub | :direct | :sync
  @type targets :: %{atom() => target()}
  @type multi_results :: %{atom() => {:ok, positions(), meta()} | {:error, atom(), meta()}}

  @type motion_result :: {:ok, meta()} | {:error, atom(), meta()}
  @type solve_result :: {:ok, positions(), meta()} | {:error, atom(), meta()}
  @type multi_motion_result :: {:ok, multi_results()} | {:error, atom(), atom(), multi_results()}
  @type multi_solve_result :: {:ok, multi_results()} | {:error, atom(), atom(), multi_results()}

  @doc """
  Move an end-effector to a target position.

  Solves inverse kinematics for the given target, updates the robot state,
  and sends position commands to all actuators controlling the affected joints.

  ## Options

  Required:
  - `:solver` - Module implementing `BB.IK.Solver` behaviour

  Optional:
  - `:delivery` - How to send actuator commands: `:pubsub` (default), `:direct`, or `:sync`
  - `:max_iterations` - Maximum solver iterations (passed to solver)
  - `:tolerance` - Convergence tolerance in metres (passed to solver)
  - `:respect_limits` - Whether to clamp to joint limits (passed to solver)

  ## Returns

  - `{:ok, meta}` - Successfully moved; meta contains solver info (iterations, residual, etc.)
  - `{:error, reason, meta}` - Failed; reason is `:ik_failed`, `:unreachable`, etc.

  ## Examples

      BB.Motion.move_to(MyRobot, :gripper, {0.3, 0.2, 0.1}, solver: BB.IK.FABRIK)

      BB.Motion.move_to(context, :gripper, target,
        solver: BB.IK.FABRIK,
        delivery: :direct,
        max_iterations: 100
      )
  """
  @spec move_to(robot_or_context(), atom(), target(), keyword()) :: motion_result()
  def move_to(robot_or_context, target_link, target, opts) do
    solver = Keyword.fetch!(opts, :solver)
    delivery = Keyword.get(opts, :delivery, :pubsub)
    solver_opts = extract_solver_opts(opts)

    {robot_module, robot, robot_state} = extract_context(robot_or_context)

    case solver.solve(robot, robot_state, target_link, target, solver_opts) do
      {:ok, positions, meta} ->
        RobotState.set_positions(robot_state, positions)
        send_positions_to_actuators(robot_module, robot, positions, delivery)
        publish_joint_state(robot_module, positions)
        {:ok, meta}

      {:error, reason, meta} ->
        {:error, reason, meta}
    end
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
  - `{:error, reason, meta}` - Failed to solve

  ## Examples

      # Check if target is reachable
      case BB.Motion.solve_only(MyRobot, :gripper, target, solver: BB.IK.FABRIK) do
        {:ok, _positions, %{reached: true}} -> :reachable
        _ -> :unreachable
      end
  """
  @spec solve_only(robot_or_context(), atom(), target(), keyword()) :: solve_result()
  def solve_only(robot_or_context, target_link, target, opts) do
    solver = Keyword.fetch!(opts, :solver)
    solver_opts = extract_solver_opts(opts)

    {_robot_module, robot, robot_state} = extract_context(robot_or_context)

    solver.solve(robot, robot_state, target_link, target, solver_opts)
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

  Optional:
  - `:delivery` - How to send actuator commands: `:pubsub` (default), `:direct`, or `:sync`
  - `:max_iterations` - Maximum solver iterations (passed to solver)
  - `:tolerance` - Convergence tolerance in metres (passed to solver)
  - `:respect_limits` - Whether to clamp to joint limits (passed to solver)

  ## Returns

  - `{:ok, results}` - All targets solved; results is a map of link → `{:ok, positions, meta}`
  - `{:error, failed_link, reason, results}` - A target failed; results contains successful solves

  ## Examples

      targets = %{
        left_foot: {0.1, 0.0, 0.0},
        right_foot: {-0.1, 0.0, 0.0}
      }

      case BB.Motion.move_to_multi(MyRobot, targets, solver: BB.IK.FABRIK) do
        {:ok, results} ->
          IO.puts("All targets reached")

        {:error, failed_link, reason, _results} ->
          IO.puts("Failed to reach \#{failed_link}: \#{reason}")
      end
  """
  @spec move_to_multi(robot_or_context(), targets(), keyword()) :: multi_motion_result()
  def move_to_multi(robot_or_context, targets, opts) do
    case solve_only_multi(robot_or_context, targets, opts) do
      {:ok, results} ->
        delivery = Keyword.get(opts, :delivery, :pubsub)
        {robot_module, robot, robot_state} = extract_context(robot_or_context)

        all_positions = merge_all_positions(results)
        RobotState.set_positions(robot_state, all_positions)
        send_positions_to_actuators(robot_module, robot, all_positions, delivery)
        publish_joint_state(robot_module, all_positions)

        {:ok, results}

      {:error, failed_link, reason, results} ->
        {:error, failed_link, reason, results}
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
  - `{:error, failed_link, reason, results}` - A target failed; results contains successful solves

  ## Examples

      targets = %{left_foot: {0.1, 0.0, 0.0}, right_foot: {-0.1, 0.0, 0.0}}

      case BB.Motion.solve_only_multi(MyRobot, targets, solver: BB.IK.FABRIK) do
        {:ok, results} ->
          Enum.each(results, fn {link, {:ok, _positions, meta}} ->
            IO.puts("\#{link}: residual=\#{meta.residual}")
          end)

        {:error, failed_link, reason, _results} ->
          IO.puts("\#{failed_link} is unreachable: \#{reason}")
      end
  """
  @spec solve_only_multi(robot_or_context(), targets(), keyword()) :: multi_solve_result()
  def solve_only_multi(robot_or_context, targets, opts) do
    solver = Keyword.fetch!(opts, :solver)
    solver_opts = extract_solver_opts(opts)

    {_robot_module, robot, robot_state} = extract_context(robot_or_context)

    targets
    |> Task.async_stream(fn {target_link, target} ->
      {target_link, solver.solve(robot, robot_state, target_link, target, solver_opts)}
    end)
    |> Enum.reduce_while({:ok, %{}}, fn
      {:ok, {link, {:ok, _, _} = result}}, {:ok, results} ->
        {:cont, {:ok, Map.put(results, link, result)}}

      {:ok, {link, {:error, reason, _meta} = result}}, {:ok, results} ->
        {:halt, {:error, link, reason, Map.put(results, link, result)}}
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

  - `:delivery` - How to send actuator commands: `:pubsub` (default), `:direct`, or `:sync`
  - `:velocity` - Velocity hint for actuators (rad/s or m/s)
  - `:duration` - Duration hint for actuators (milliseconds)

  ## Examples

      positions = %{shoulder: 0.5, elbow: 1.2, wrist: 0.3}
      :ok = BB.Motion.send_positions(MyRobot, positions)

      # With direct delivery for lower latency
      :ok = BB.Motion.send_positions(MyRobot, positions, delivery: :direct)
  """
  @spec send_positions(robot_or_context(), positions(), keyword()) :: :ok
  def send_positions(robot_or_context, positions, opts \\ []) do
    delivery = Keyword.get(opts, :delivery, :pubsub)
    actuator_opts = extract_actuator_opts(opts)

    {robot_module, robot, robot_state} = extract_context(robot_or_context)

    RobotState.set_positions(robot_state, positions)
    send_positions_to_actuators(robot_module, robot, positions, delivery, actuator_opts)
    publish_joint_state(robot_module, positions)
  end

  defp extract_context(%Context{} = context) do
    {context.robot_module, context.robot, context.robot_state}
  end

  defp extract_context(robot_module) when is_atom(robot_module) do
    robot = Runtime.get_robot(robot_module)
    robot_state = Runtime.get_robot_state(robot_module)
    {robot_module, robot, robot_state}
  end

  defp extract_solver_opts(opts) do
    opts
    |> Keyword.take([:max_iterations, :tolerance, :respect_limits, :initial_positions])
    |> Keyword.reject(fn {_k, v} -> is_nil(v) end)
  end

  defp extract_actuator_opts(opts) do
    opts
    |> Keyword.take([:velocity, :duration, :command_id])
    |> Keyword.reject(fn {_k, v} -> is_nil(v) end)
  end

  defp send_positions_to_actuators(robot_module, robot, positions, delivery, opts \\ []) do
    Enum.each(positions, fn {joint_name, position} ->
      send_joint_position(robot_module, robot, joint_name, position, delivery, opts)
    end)

    :ok
  end

  defp send_joint_position(robot_module, robot, joint_name, position, delivery, opts) do
    case Map.get(robot.joints, joint_name) do
      nil ->
        :ok

      joint ->
        Enum.each(joint.actuators, fn actuator_name ->
          send_position_to_actuator(robot_module, robot, actuator_name, position, delivery, opts)
        end)
    end
  end

  defp send_position_to_actuator(robot_module, robot, actuator_name, position, :pubsub, opts) do
    path = actuator_path(robot, actuator_name)
    Actuator.set_position(robot_module, path, position, opts)
  end

  defp send_position_to_actuator(robot_module, _robot, actuator_name, position, :direct, opts) do
    Actuator.set_position!(robot_module, actuator_name, position, opts)
  end

  defp send_position_to_actuator(robot_module, _robot, actuator_name, position, :sync, opts) do
    case Actuator.set_position_sync(robot_module, actuator_name, position, opts) do
      {:ok, _} -> :ok
      {:error, reason} -> raise "Actuator #{actuator_name} rejected position: #{inspect(reason)}"
    end
  end

  defp actuator_path(robot, actuator_name) do
    case Map.get(robot.actuators, actuator_name) do
      nil ->
        [actuator_name]

      %{joint: joint_name} ->
        case BB.Robot.path_to(robot, joint_name) do
          nil -> [actuator_name]
          joint_path -> joint_path ++ [actuator_name]
        end
    end
  end

  defp publish_joint_state(robot_module, positions) when map_size(positions) > 0 do
    {names, values} = positions |> Enum.unzip()
    count = length(names)

    {:ok, msg} =
      BB.Message.new(BB.Message.Sensor.JointState, :motion,
        names: names,
        positions: Enum.map(values, &(&1 * 1.0)),
        velocities: List.duplicate(0.0, count),
        efforts: List.duplicate(0.0, count)
      )

    BB.publish(robot_module, [:sensor, :motion], msg)
  end

  defp publish_joint_state(_robot_module, _positions), do: :ok
end
