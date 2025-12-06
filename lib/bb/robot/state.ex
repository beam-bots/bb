# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Robot.State do
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
      {:ok, state} = BB.Robot.State.new(robot)

      # Set/get joint positions
      :ok = BB.Robot.State.set_joint_position(state, :shoulder, 0.5)
      pos = BB.Robot.State.get_joint_position(state, :shoulder)

      # Get all joint positions as a map
      positions = BB.Robot.State.get_all_positions(state)

      # Clean up when done
      :ok = BB.Robot.State.delete(state)
  """

  alias BB.Robot

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

      iex> positions = BB.Robot.State.get_all_positions(state)
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

      :ok = BB.Robot.State.set_positions(state, %{
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

  @doc """
  Get the current robot state machine state.

  Returns the state atom (e.g., `:disarmed`, `:idle`, `:executing`).
  """
  @spec get_robot_state(t()) :: atom()
  def get_robot_state(%__MODULE__{table: table}) do
    case :ets.lookup(table, :robot_state) do
      [{:robot_state, state}] -> state
      [] -> :disarmed
    end
  end

  @doc """
  Set the robot state machine state.
  """
  @spec set_robot_state(t(), atom()) :: :ok
  def set_robot_state(%__MODULE__{table: table}, state) when is_atom(state) do
    :ets.insert(table, {:robot_state, state})
    :ok
  end

  # Parameter functions

  @doc """
  Get a parameter value by path.

  Returns `{:ok, value}` if the parameter exists, `{:error, :not_found}` otherwise.
  """
  @spec get_parameter(t(), [atom()]) :: {:ok, term()} | {:error, :not_found}
  def get_parameter(%__MODULE__{table: table}, path) when is_list(path) do
    case :ets.lookup(table, {:param, path}) do
      [{{:param, ^path}, value}] -> {:ok, value}
      [] -> {:error, :not_found}
    end
  end

  @doc """
  Set a parameter value by path.

  This is a low-level function that does not validate or notify.
  Use `BB.Parameter.set/3` for the validated, notifying version.
  """
  @spec set_parameter(t(), [atom()], term()) :: :ok
  def set_parameter(%__MODULE__{table: table}, path, value) when is_list(path) do
    :ets.insert(table, {{:param, path}, value})
    :ok
  end

  @doc """
  Set multiple parameters atomically.

  This is a low-level function that does not validate or notify.
  """
  @spec set_parameters(t(), [{[atom()], term()}]) :: :ok
  def set_parameters(%__MODULE__{table: table}, params) when is_list(params) do
    entries = Enum.map(params, fn {path, value} -> {{:param, path}, value} end)
    :ets.insert(table, entries)
    :ok
  end

  @doc """
  List all parameters, optionally filtered by path prefix.

  Returns a list of `{path, metadata}` tuples where metadata includes
  the current value and schema information if registered.
  """
  @spec list_parameters(t(), [atom()]) :: [{[atom()], map()}]
  def list_parameters(%__MODULE__{table: table}, prefix \\ []) when is_list(prefix) do
    # Get all parameter entries
    params =
      :ets.match_object(table, {{:param, :_}, :_})
      |> Enum.filter(fn {{:param, path}, _value} ->
        List.starts_with?(path, prefix)
      end)

    # Get schemas to enrich metadata
    schemas = get_all_schemas(table)

    Enum.map(params, fn {{:param, path}, value} ->
      schema_info = find_schema_for_path(schemas, path)
      {path, build_parameter_metadata(value, schema_info, path)}
    end)
  end

  @doc """
  Register a parameter schema for a component path.

  The schema is stored and used for validation and metadata.
  """
  @spec register_parameter_schema(t(), [atom()], Spark.Options.t()) :: :ok
  def register_parameter_schema(%__MODULE__{table: table}, path, schema)
      when is_list(path) do
    :ets.insert(table, {{:param_schema, path}, schema})
    :ok
  end

  @doc """
  Get the registered schema for a path prefix.

  Returns `{:ok, schema}` if found, `{:error, :not_found}` otherwise.
  """
  @spec get_parameter_schema(t(), [atom()]) :: {:ok, Spark.Options.t()} | {:error, :not_found}
  def get_parameter_schema(%__MODULE__{table: table}, path) when is_list(path) do
    case :ets.lookup(table, {:param_schema, path}) do
      [{{:param_schema, ^path}, schema}] -> {:ok, schema}
      [] -> {:error, :not_found}
    end
  end

  @doc """
  Find the schema that applies to a given parameter path.

  Searches for the longest matching schema prefix.
  """
  @spec find_schema_for_parameter(t(), [atom()]) ::
          {:ok, [atom()], Spark.Options.t()} | {:error, :not_found}
  def find_schema_for_parameter(%__MODULE__{table: table}, path) when is_list(path) do
    schemas = get_all_schemas(table)

    case find_schema_for_path(schemas, path) do
      nil -> {:error, :not_found}
      {schema_path, schema} -> {:ok, schema_path, schema}
    end
  end

  defp get_all_schemas(table) do
    :ets.match_object(table, {{:param_schema, :_}, :_})
    |> Enum.map(fn {{:param_schema, path}, schema} -> {path, schema} end)
  end

  defp find_schema_for_path(schemas, path) do
    # Find the schema where path is exactly one level deeper than schema_path
    # e.g., path [:motion, :max_speed] matches schema_path [:motion]
    # but path [:totally_fake, :param] does NOT match schema_path []
    schemas
    |> Enum.filter(fn {schema_path, _schema} ->
      List.starts_with?(path, schema_path) and length(path) == length(schema_path) + 1
    end)
    |> Enum.max_by(fn {schema_path, _schema} -> length(schema_path) end, fn -> nil end)
  end

  defp build_parameter_metadata(value, nil, _path) do
    # No schema - this shouldn't happen for registered parameters
    %{value: value}
  end

  defp build_parameter_metadata(value, {schema_path, %Spark.Options{schema: schema_opts}}, path) do
    # The parameter name is the part of the path after the schema path
    param_name =
      path
      |> Enum.drop(length(schema_path))
      |> List.first()

    param_opts = Keyword.get(schema_opts, param_name, [])

    %{
      value: value,
      type: Keyword.get(param_opts, :type),
      doc: Keyword.get(param_opts, :doc),
      default: Keyword.get(param_opts, :default)
    }
  end

  defp initialise_joints(%__MODULE__{table: table, robot: robot}) do
    joint_entries =
      robot.joints
      |> Map.keys()
      |> Enum.flat_map(fn joint_name ->
        [
          {{:position, joint_name}, 0.0},
          {{:velocity, joint_name}, 0.0}
        ]
      end)

    entries = [{:robot_state, :disarmed} | joint_entries]
    :ets.insert(table, entries)
  end
end
