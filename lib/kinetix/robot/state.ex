# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Robot.State do
  @moduledoc """
  ETS-backed mutable state for robot instances.

  This module manages joint positions, velocities, and computed transforms
  for robot instances. Each robot instance has its own ETS table for
  concurrent read access.

  ## State Structure

  For each joint, the following state is stored:
  - `position`: current joint position (radians for revolute, meters for prismatic)
  - `velocity`: current joint velocity (rad/s or m/s)

  ## Usage

      # Create state for a robot instance
      {:ok, state} = Kinetix.Robot.State.new(robot)

      # Set/get joint positions
      :ok = Kinetix.Robot.State.set_joint_position(state, :shoulder, 0.5)
      pos = Kinetix.Robot.State.get_joint_position(state, :shoulder)

      # Get all joint positions as a map
      positions = Kinetix.Robot.State.get_all_positions(state)

      # Clean up when done
      :ok = Kinetix.Robot.State.delete(state)
  """

  alias Kinetix.Robot

  defstruct [:table, :robot]

  @type t :: %__MODULE__{
          table: :ets.table(),
          robot: Robot.t()
        }

  @doc """
  Create a new state table for a robot.

  Returns `{:ok, state}` on success.
  """
  @spec new(Robot.t()) :: {:ok, t()}
  def new(%Robot{} = robot) do
    table =
      :ets.new(:robot_state, [
        :set,
        :public,
        read_concurrency: true,
        write_concurrency: false
      ])

    state = %__MODULE__{table: table, robot: robot}

    initialise_joints(state)

    {:ok, state}
  end

  @doc """
  Delete a state table and free resources.
  """
  @spec delete(t()) :: :ok
  def delete(%__MODULE__{table: table}) do
    :ets.delete(table)
    :ok
  end

  @doc """
  Get the current position of a joint.

  Returns `nil` if the joint doesn't exist.
  """
  @spec get_joint_position(t(), atom()) :: float() | nil
  def get_joint_position(%__MODULE__{table: table}, joint_name) do
    case :ets.lookup(table, {:position, joint_name}) do
      [{{:position, ^joint_name}, value}] -> value
      [] -> nil
    end
  end

  @doc """
  Set the position of a joint.
  """
  @spec set_joint_position(t(), atom(), float()) :: :ok
  def set_joint_position(%__MODULE__{table: table}, joint_name, position)
      when is_float(position) or is_integer(position) do
    :ets.insert(table, {{:position, joint_name}, position / 1})
    :ok
  end

  @doc """
  Get the current velocity of a joint.

  Returns `nil` if the joint doesn't exist.
  """
  @spec get_joint_velocity(t(), atom()) :: float() | nil
  def get_joint_velocity(%__MODULE__{table: table}, joint_name) do
    case :ets.lookup(table, {:velocity, joint_name}) do
      [{{:velocity, ^joint_name}, value}] -> value
      [] -> nil
    end
  end

  @doc """
  Set the velocity of a joint.
  """
  @spec set_joint_velocity(t(), atom(), float()) :: :ok
  def set_joint_velocity(%__MODULE__{table: table}, joint_name, velocity)
      when is_float(velocity) or is_integer(velocity) do
    :ets.insert(table, {{:velocity, joint_name}, velocity / 1})
    :ok
  end

  @doc """
  Get all joint positions as a map.

  ## Examples

      iex> positions = Kinetix.Robot.State.get_all_positions(state)
      %{shoulder: 0.0, elbow: 0.5, wrist: -0.3}
  """
  @spec get_all_positions(t()) :: %{atom() => float()}
  def get_all_positions(%__MODULE__{table: table, robot: robot}) do
    robot.joints
    |> Map.keys()
    |> Map.new(fn joint_name ->
      case :ets.lookup(table, {:position, joint_name}) do
        [{{:position, ^joint_name}, value}] -> {joint_name, value}
        [] -> {joint_name, 0.0}
      end
    end)
  end

  @doc """
  Get all joint velocities as a map.
  """
  @spec get_all_velocities(t()) :: %{atom() => float()}
  def get_all_velocities(%__MODULE__{table: table, robot: robot}) do
    robot.joints
    |> Map.keys()
    |> Map.new(fn joint_name ->
      case :ets.lookup(table, {:velocity, joint_name}) do
        [{{:velocity, ^joint_name}, value}] -> {joint_name, value}
        [] -> {joint_name, 0.0}
      end
    end)
  end

  @doc """
  Set multiple joint positions at once.

  ## Examples

      :ok = Kinetix.Robot.State.set_positions(state, %{
        shoulder: 0.5,
        elbow: -0.3,
        wrist: 0.0
      })
  """
  @spec set_positions(t(), %{atom() => float()}) :: :ok
  def set_positions(%__MODULE__{table: table}, positions) when is_map(positions) do
    entries =
      Enum.map(positions, fn {joint_name, position} ->
        {{:position, joint_name}, position / 1}
      end)

    :ets.insert(table, entries)
    :ok
  end

  @doc """
  Set multiple joint velocities at once.
  """
  @spec set_velocities(t(), %{atom() => float()}) :: :ok
  def set_velocities(%__MODULE__{table: table}, velocities) when is_map(velocities) do
    entries =
      Enum.map(velocities, fn {joint_name, velocity} ->
        {{:velocity, joint_name}, velocity / 1}
      end)

    :ets.insert(table, entries)
    :ok
  end

  @doc """
  Reset all joints to their default positions (0.0).
  """
  @spec reset(t()) :: :ok
  def reset(%__MODULE__{} = state) do
    initialise_joints(state)
    :ok
  end

  @doc """
  Get the positions of joints along a path from root to a target link.

  Returns a list of {joint_name, position} tuples in traversal order.
  """
  @spec get_chain_positions(t(), atom()) :: [{atom(), float()}]
  def get_chain_positions(%__MODULE__{robot: robot} = state, target_link) do
    case Robot.path_to(robot, target_link) do
      nil ->
        []

      path ->
        path
        |> Enum.filter(&Map.has_key?(robot.joints, &1))
        |> Enum.map(fn joint_name ->
          {joint_name, get_joint_position(state, joint_name) || 0.0}
        end)
    end
  end

  defp initialise_joints(%__MODULE__{table: table, robot: robot}) do
    entries =
      robot.joints
      |> Map.keys()
      |> Enum.flat_map(fn joint_name ->
        [
          {{:position, joint_name}, 0.0},
          {{:velocity, joint_name}, 0.0}
        ]
      end)

    :ets.insert(table, entries)
  end
end
