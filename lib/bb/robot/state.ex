# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Robot.State do
  @moduledoc """
  ETS-backed mutable state for robot instances.

  This module manages joint configurations, velocities, and computed transforms
  for robot instances. Each robot instance has its own ETS table for
  concurrent read access.

  ## Configurations, not positions

  A joint's configuration is a point in its own configuration space, and its
  shape depends on the joint's type:

  | Joint type | DoF | Configuration | Velocity |
  |---|---|---|---|
  | `:fixed` | 0 | `0.0`, the only one it has | `0.0` |
  | `:revolute`, `:continuous` | 1 | `float` angle in radians | `float` rad/s |
  | `:prismatic` | 1 | `float` displacement in metres | `float` m/s |
  | `:planar` | 3 | `BB.Math.Transform2D` | `BB.Message.Geometry.Twist2D` |
  | `:floating` | 6 | `BB.Math.Transform` | `BB.Message.Geometry.Twist` |

  "Position" is the wrong word for a 4x4 homogeneous transform, which is why
  these functions say "configuration" — the standard term for a point in a
  robot's configuration space, and correct for every joint type.

  Writes are whole-configuration and atomic. There is no API for setting part of
  a floating joint, because a partial update invites inconsistent intermediate
  states and the natural producer of such a configuration is an estimator
  emitting a complete pose.

  A fixed joint has zero degrees of freedom, so its configuration space is a
  single point and `0.0` is the only value it accepts. It is still present in
  `get_all_configurations/1` so that map can be handed straight back to
  `set_configurations/2`.

  ## Usage

      # Create state for a robot instance
      {:ok, state} = BB.Robot.State.new(robot)

      # Set/get a joint configuration
      :ok = BB.Robot.State.set_configuration(state, :shoulder, 0.5)
      {:ok, angle} = BB.Robot.State.get_configuration(state, :shoulder)

      # Get every joint's configuration as a map
      configurations = BB.Robot.State.get_all_configurations(state)

      # Clean up when done
      :ok = BB.Robot.State.delete(state)
  """

  alias BB.Error.Invalid.JointConfig
  alias BB.Error.Kinematics.UnknownJoint
  alias BB.Math.Transform
  alias BB.Math.Transform2D
  alias BB.Math.Vec3
  alias BB.Message.Geometry.Twist
  alias BB.Message.Geometry.Twist2D
  alias BB.Parameter.Type, as: ParameterType
  alias BB.Robot

  defstruct [:table, :robot]

  @type t :: %__MODULE__{
          table: :ets.table(),
          robot: Robot.t()
        }

  @typedoc """
  A joint's configuration, shaped to its type.

  See the module documentation for which shape belongs to which joint type.
  """
  @type configuration :: float() | Transform2D.t() | Transform.t()

  @typedoc "A joint's velocity, shaped to its type."
  @type velocity :: float() | Twist2D.t() | Twist.t()

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
  Get the current configuration of a joint.

  ## Examples

      {:ok, 0.5} = BB.Robot.State.get_configuration(state, :shoulder)
      {:ok, %BB.Math.Transform{}} = BB.Robot.State.get_configuration(state, :base)
  """
  @spec get_configuration(t(), atom()) :: {:ok, configuration()} | {:error, UnknownJoint.t()}
  def get_configuration(%__MODULE__{table: table} = state, joint_name) do
    with {:ok, joint} <- fetch_joint(state, joint_name) do
      {:ok, read(table, :configuration, joint_name, joint.type)}
    end
  end

  @doc """
  Set the configuration of a joint.

  The value's shape must match the joint's type, so a `BB.Math.Transform2D`
  aimed at a revolute joint is an error rather than a wrong pose.

  ## Examples

      :ok = BB.Robot.State.set_configuration(state, :shoulder, 0.5)
      :ok = BB.Robot.State.set_configuration(state, :base, transform)
  """
  @spec set_configuration(t(), atom(), configuration()) ::
          :ok | {:error, UnknownJoint.t() | JointConfig.t()}
  def set_configuration(%__MODULE__{table: table} = state, joint_name, value) do
    with {:ok, joint} <- fetch_joint(state, joint_name),
         {:ok, encoded} <- encode(joint, :configuration, value) do
      :ets.insert(table, {{:configuration, joint_name}, encoded})
      :ok
    end
  end

  @doc """
  Get the current velocity of a joint.
  """
  @spec get_velocity(t(), atom()) :: {:ok, velocity()} | {:error, UnknownJoint.t()}
  def get_velocity(%__MODULE__{table: table} = state, joint_name) do
    with {:ok, joint} <- fetch_joint(state, joint_name) do
      {:ok, read(table, :velocity, joint_name, joint.type)}
    end
  end

  @doc """
  Set the velocity of a joint.
  """
  @spec set_velocity(t(), atom(), velocity()) ::
          :ok | {:error, UnknownJoint.t() | JointConfig.t()}
  def set_velocity(%__MODULE__{table: table} = state, joint_name, value) do
    with {:ok, joint} <- fetch_joint(state, joint_name),
         {:ok, encoded} <- encode(joint, :velocity, value) do
      :ets.insert(table, {{:velocity, joint_name}, encoded})
      :ok
    end
  end

  @doc """
  Get every joint's configuration as a map.

  Every joint in the robot is present, shaped to its own type.

  ## Examples

      iex> BB.Robot.State.get_all_configurations(state)
      %{shoulder: 0.5, elbow: -0.2, base: %BB.Math.Transform{}}
  """
  @spec get_all_configurations(t()) :: %{atom() => configuration()}
  def get_all_configurations(%__MODULE__{} = state), do: read_all(state, :configuration)

  @doc """
  Get every joint's velocity as a map.
  """
  @spec get_all_velocities(t()) :: %{atom() => velocity()}
  def get_all_velocities(%__MODULE__{} = state), do: read_all(state, :velocity)

  @doc """
  Set multiple joint configurations at once.

  Every value is validated before anything is written, so a rejected map leaves
  the table untouched rather than applying part of itself.

  ## Examples

      :ok = BB.Robot.State.set_configurations(state, %{
        shoulder: 0.5,
        elbow: -0.3,
        base: transform
      })
  """
  @spec set_configurations(t(), %{atom() => configuration()}) ::
          :ok | {:error, UnknownJoint.t() | JointConfig.t()}
  def set_configurations(%__MODULE__{} = state, configurations) when is_map(configurations) do
    write_all(state, :configuration, configurations)
  end

  @doc """
  Set multiple joint velocities at once.
  """
  @spec set_velocities(t(), %{atom() => velocity()}) ::
          :ok | {:error, UnknownJoint.t() | JointConfig.t()}
  def set_velocities(%__MODULE__{} = state, velocities) when is_map(velocities) do
    write_all(state, :velocity, velocities)
  end

  @doc """
  Reset all joints to their identity configurations and zero velocities.
  """
  @spec reset(t()) :: :ok
  def reset(%__MODULE__{} = state) do
    initialise_joints(state)
    :ok
  end

  @doc """
  Get the configurations of joints along a path from root to a target link.

  Returns a list of `{joint_name, configuration}` tuples in traversal order.
  """
  @spec get_chain_configurations(t(), atom()) :: [{atom(), configuration()}]
  def get_chain_configurations(%__MODULE__{robot: robot} = state, target_link) do
    case Robot.path_to(robot, target_link) do
      {:error, _} ->
        []

      {:ok, path} ->
        path
        |> Enum.filter(&Map.has_key?(robot.joints, &1))
        |> Enum.map(fn joint_name ->
          {:ok, configuration} = get_configuration(state, joint_name)
          {joint_name, configuration}
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
    {type, min, max} = ParameterType.describe(Keyword.get(param_opts, :type))

    %{
      value: value,
      type: type,
      min: min,
      max: max,
      doc: Keyword.get(param_opts, :doc),
      default: Keyword.get(param_opts, :default)
    }
  end

  defp initialise_joints(%__MODULE__{table: table, robot: robot}) do
    joint_entries =
      Enum.flat_map(robot.joints, fn {joint_name, joint} ->
        [
          {{:configuration, joint_name}, encoded_default(joint.type, :configuration)},
          {{:velocity, joint_name}, encoded_default(joint.type, :velocity)}
        ]
      end)

    # Internal state is :idle/:executing (armed/disarmed is in BB.Safety.Controller)
    entries = [{:robot_state, :idle} | joint_entries]
    :ets.insert(table, entries)
  end

  defp fetch_joint(%__MODULE__{robot: robot}, joint_name), do: Robot.get_joint(robot, joint_name)

  defp read(table, kind, joint_name, joint_type) do
    case :ets.lookup(table, {kind, joint_name}) do
      [{{^kind, ^joint_name}, stored}] -> decode(joint_type, kind, stored)
      [] -> decode(joint_type, kind, encoded_default(joint_type, kind))
    end
  end

  defp read_all(%__MODULE__{table: table, robot: robot}, kind) do
    Map.new(robot.joints, fn {joint_name, joint} ->
      {joint_name, read(table, kind, joint_name, joint.type)}
    end)
  end

  # Every value is encoded before anything is inserted, so a map containing one
  # bad entry leaves the table untouched rather than applying its valid half.
  defp write_all(%__MODULE__{table: table} = state, kind, values) do
    with {:ok, entries} <- encode_all(state, kind, values) do
      :ets.insert(table, entries)
      :ok
    end
  end

  defp encode_all(state, kind, values) do
    Enum.reduce_while(values, {:ok, []}, fn {joint_name, value}, {:ok, encoded} ->
      with {:ok, joint} <- fetch_joint(state, joint_name),
           {:ok, value} <- encode(joint, kind, value) do
        {:cont, {:ok, [{{kind, joint_name}, value} | encoded]}}
      else
        {:error, reason} -> {:halt, {:error, reason}}
      end
    end)
  end

  # ----------------------------------------------------------------------------
  # Storage encoding
  #
  # Nothing backend-specific may reach ETS. A `%Nx.Tensor{}` struct carries
  # backend state: fine under `Nx.BinaryBackend`, but under EXLA it is a
  # reference to accelerator memory, which is not meaningfully shareable through
  # a table and may be invalidated out from under a reader. So tensor-backed
  # values are stored as their raw bytes.
  #
  # Those bytes are also the only lossless option. Decomposing a 4x4 to a
  # quaternion and translation is a `sqrt` with a branch on the trace, 16 numbers
  # to 7 is not a bijection, and the round-trip would run on *every read* — so
  # error would accumulate rather than being a one-off. `Nx.to_binary/1` and
  # `Nx.from_binary/2` are bit-exact and involve no arithmetic at all.
  # ----------------------------------------------------------------------------

  # A fixed joint's configuration space is a single point, so the one thing you
  # may assign is that point. Accepting it keeps read-modify-write working — the
  # value comes straight back out of `get_all_configurations/1` — while a nonzero
  # angle or a `Transform` aimed at a welded joint is still caught. Forward
  # kinematics ignores the value either way, since both motion masks are zero.
  defp encode(%{type: :fixed}, _kind, value) when is_number(value) and value == 0, do: {:ok, 0.0}

  defp encode(%{type: type}, _kind, value)
       when type in [:revolute, :continuous, :prismatic] and is_number(value) do
    {:ok, value / 1}
  end

  defp encode(%{type: :planar}, :configuration, %Transform2D{} = value) do
    {:ok, {value.x, value.y, value.theta}}
  end

  defp encode(%{type: :planar}, :velocity, %Twist2D{} = value) do
    {:ok, {value.vx, value.vy, value.omega}}
  end

  defp encode(%{type: :floating}, :configuration, %Transform{} = value) do
    {:ok, value |> Transform.tensor() |> Nx.to_binary()}
  end

  defp encode(%{type: :floating}, :velocity, %Twist{linear: linear, angular: angular}) do
    {:ok, Nx.to_binary(Nx.concatenate([Vec3.tensor(linear), Vec3.tensor(angular)]))}
  end

  defp encode(joint, kind, value), do: {:error, invalid(joint, kind, value)}

  defp decode(type, _kind, stored) when type in [:fixed, :revolute, :continuous, :prismatic] do
    stored
  end

  defp decode(:planar, :configuration, {x, y, theta}), do: Transform2D.new(x, y, theta)
  defp decode(:planar, :velocity, {vx, vy, omega}), do: %Twist2D{vx: vx, vy: vy, omega: omega}

  defp decode(:floating, :configuration, bytes) do
    bytes |> Nx.from_binary(:f64) |> Nx.reshape({4, 4}) |> Transform.from_tensor()
  end

  defp decode(:floating, :velocity, bytes) do
    components = Nx.from_binary(bytes, :f64)

    %Twist{
      linear: components |> Nx.slice([0], [3]) |> Vec3.from_tensor(),
      angular: components |> Nx.slice([3], [3]) |> Vec3.from_tensor()
    }
  end

  defp encoded_default(:planar, :configuration), do: {0.0, 0.0, 0.0}
  defp encoded_default(:planar, :velocity), do: {0.0, 0.0, 0.0}

  defp encoded_default(:floating, :configuration) do
    Transform.identity() |> Transform.tensor() |> Nx.to_binary()
  end

  defp encoded_default(:floating, :velocity) do
    Nx.to_binary(Nx.broadcast(Nx.tensor(0.0, type: :f64), {6}))
  end

  defp encoded_default(_type, _kind), do: 0.0

  defp invalid(%{name: name, type: type}, kind, value) do
    JointConfig.exception(
      joint: name,
      field: kind,
      value: value,
      expected: expected(type, kind)
    )
  end

  defp expected(:fixed, _kind), do: 0.0
  defp expected(:planar, :configuration), do: Transform2D
  defp expected(:planar, :velocity), do: Twist2D
  defp expected(:floating, :configuration), do: Transform
  defp expected(:floating, :velocity), do: Twist
  defp expected(_type, _kind), do: :float
end
