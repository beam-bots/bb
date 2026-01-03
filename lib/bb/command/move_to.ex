# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Command.MoveTo do
  @moduledoc """
  Standard command handler for moving end-effectors to target positions.

  This command uses inverse kinematics to compute joint angles and sends
  position commands to all actuators controlling the affected joints.

  Supports both single-target and multi-target modes for coordinated motion.

  ## Goal Parameters

  ### Single Target Mode

  Required:
  - `target` - Target position as `BB.Vec3.t()` in metres
  - `target_link` - Name of the link to move (end-effector)
  - `solver` - Module implementing `BB.IK.Solver` behaviour

  ### Multi-Target Mode

  Required:
  - `targets` - Map of link names to target positions: `%{link: BB.Vec3.t()}`
  - `solver` - Module implementing `BB.IK.Solver` behaviour

  ### Optional (both modes)

  - `max_iterations` - Maximum solver iterations (default: 50)
  - `tolerance` - Convergence tolerance in metres (default: 1.0e-4)
  - `respect_limits` - Whether to clamp to joint limits (default: true)
  - `delivery` - Actuator command delivery: `:pubsub` (default), `:direct`, or `:sync`

  ## Usage

  ### Single Target

      alias BB.Vec3

      {:ok, cmd} = MyRobot.move_to(%{
        target: Vec3.new(0.3, 0.2, 0.1),
        target_link: :gripper,
        solver: BB.IK.FABRIK
      })
      {:ok, meta} = BB.Command.await(cmd)

  ### Multiple Targets (for gait, coordinated motion)

      {:ok, cmd} = MyRobot.move_to(%{
        targets: %{
          left_foot: Vec3.new(0.1, 0.0, 0.0),
          right_foot: Vec3.new(-0.1, 0.0, 0.0)
        },
        solver: BB.IK.FABRIK
      })
      {:ok, results} = BB.Command.await(cmd)

  ## Return Value

  ### Single Target

  On success, returns metadata from the IK solver:

      %{
        iterations: 12,
        residual: 0.00003,
        reached: true,
        reason: :converged
      }

  ### Multiple Targets

  On success, returns a map of link → result:

      %{
        left_foot: {:ok, %{joint1: 0.5}, %{iterations: 10, ...}},
        right_foot: {:ok, %{joint2: 0.3}, %{iterations: 8, ...}}
      }

  """
  use BB.Command

  alias BB.Error.Invalid.Command, as: InvalidCommand
  alias BB.Error.Kinematics.MultiFailed
  alias BB.Math.Vec3
  alias BB.Message.Geometry.Point3D
  alias BB.Motion

  @impl BB.Command
  def handle_command(goal, context, state) when is_map_key(goal, :targets) do
    result = handle_multi_target(goal, context)
    {:stop, :normal, %{state | result: result}}
  end

  def handle_command(goal, context, state) when is_map_key(goal, :target) do
    result = handle_single_target(goal, context)
    {:stop, :normal, %{state | result: result}}
  end

  def handle_command(_goal, _context, state) do
    error =
      InvalidCommand.exception(
        command: __MODULE__,
        argument: :target_or_targets,
        reason: "required: must specify either :target or :targets"
      )

    {:stop, :normal, %{state | result: {:error, error}}}
  end

  @impl BB.Command
  def result(%{result: result}), do: result

  defp handle_single_target(goal, context) do
    with {:ok, target} <- fetch_required(goal, :target),
         {:ok, target_link} <- fetch_required(goal, :target_link),
         {:ok, solver} <- fetch_required(goal, :solver) do
      opts = build_opts(goal, solver)
      target = normalize_target(target)

      case Motion.move_to(context, target_link, target, opts) do
        {:ok, meta} ->
          {:ok, meta}

        {:error, error} ->
          {:error, error}
      end
    end
  end

  defp handle_multi_target(goal, context) do
    with {:ok, targets} <- fetch_required(goal, :targets),
         {:ok, solver} <- fetch_required(goal, :solver) do
      opts = build_opts(goal, solver)

      case Motion.move_to_multi(context, targets, opts) do
        {:ok, results} ->
          {:ok, results}

        {:error, failed_link, error, results} ->
          {:error,
           MultiFailed.exception(
             failed_link: failed_link,
             error: error,
             partial_results: results
           )}
      end
    end
  end

  defp fetch_required(goal, key) do
    case Map.fetch(goal, key) do
      {:ok, value} ->
        {:ok, value}

      :error ->
        {:error, InvalidCommand.exception(command: __MODULE__, argument: key, reason: "required")}
    end
  end

  defp build_opts(goal, solver) do
    [
      solver: solver,
      max_iterations: Map.get(goal, :max_iterations),
      tolerance: Map.get(goal, :tolerance),
      respect_limits: Map.get(goal, :respect_limits),
      delivery: Map.get(goal, :delivery)
    ]
    |> Keyword.reject(fn {_k, v} -> is_nil(v) end)
  end

  defp normalize_target(%Vec3{} = target), do: target
  defp normalize_target(%Point3D{} = point), do: Point3D.to_vec3(point)
end
