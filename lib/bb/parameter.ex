# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Parameter do
  @moduledoc """
  Runtime-adjustable parameters for robot components.

  Parameters provide a way to configure robot behaviour at runtime without
  recompilation. They support validation, change notifications via PubSub,
  and optional persistence.

  ## Behaviour

  Components that expose parameters implement the `BB.Parameter` behaviour:

      defmodule MyController do
        use GenServer
        @behaviour BB.Parameter

        @impl BB.Parameter
        def param_schema do
          Spark.Options.new!(
            kp: [type: :float, required: true, doc: "Proportional gain"],
            ki: [type: :float, default: 0.0, doc: "Integral gain"],
            kd: [type: :float, default: 0.0, doc: "Derivative gain"]
          )
        end
      end

  ## Path-Based Identification

  Parameters are identified by hierarchical paths that match the PubSub
  convention:

  - `[:robot, :max_velocity]` - Robot-level parameter
  - `[:controller, :pid, :kp]` - Component parameter
  - `[:sensor, :imu, :sample_rate]` - Sensor configuration

  ## Usage

      # Read a parameter (fast, direct ETS access)
      {:ok, value} = BB.Parameter.get(MyRobot, [:motion, :max_speed])

      # Write a parameter (validated, publishes change)
      :ok = BB.Parameter.set(MyRobot, [:motion, :max_speed], 2.0)

      # Atomic batch update
      :ok = BB.Parameter.set_many(MyRobot, [
        {[:controller, :pid, :kp], 2.0},
        {[:controller, :pid, :ki], 0.2}
      ])

      # List parameters
      params = BB.Parameter.list(MyRobot, prefix: [:controller])

  ## Change Notifications

  Parameter changes are published via `BB.PubSub` with the `:param` prefix:

      BB.PubSub.subscribe(MyRobot, [:param, :controller, :pid])
      # Receives: {:bb, [:param, :controller, :pid, :kp], %BB.Message{}}
  """

  alias BB.Robot.Runtime
  alias BB.Robot.State, as: RobotState

  @doc """
  Returns a compiled `Spark.Options` schema for this component's parameters.

  The schema defines parameter names, types, defaults, and constraints.
  """
  @callback param_schema() :: Spark.Options.t()

  @doc """
  Get a parameter value.

  Returns `{:ok, value}` if the parameter exists, `{:error, :not_found}` otherwise.

  This is a fast operation - it reads directly from ETS.

  ## Examples

      {:ok, 1.5} = BB.Parameter.get(MyRobot, [:motion, :max_speed])
      {:error, :not_found} = BB.Parameter.get(MyRobot, [:nonexistent])
  """
  @spec get(module(), [atom()]) :: {:ok, term()} | {:error, :not_found}
  def get(robot_module, path) when is_atom(robot_module) and is_list(path) do
    robot_state = get_robot_state(robot_module)
    RobotState.get_parameter(robot_state, path)
  end

  @doc """
  Get a parameter value, raising if not found.

  ## Examples

      1.5 = BB.Parameter.get!(MyRobot, [:motion, :max_speed])
  """
  @spec get!(module(), [atom()]) :: term()
  def get!(robot_module, path) do
    case get(robot_module, path) do
      {:ok, value} -> value
      {:error, :not_found} -> raise ArgumentError, "parameter not found: #{inspect(path)}"
    end
  end

  @doc """
  Set a parameter value.

  The value is validated against the registered schema (if any) before being
  stored. On success, a change notification is published via PubSub.

  Returns `:ok` on success, `{:error, reason}` on validation failure.

  ## Examples

      :ok = BB.Parameter.set(MyRobot, [:motion, :max_speed], 2.0)
      {:error, reason} = BB.Parameter.set(MyRobot, [:motion, :max_speed], -1.0)
  """
  @spec set(module(), [atom()], term()) :: :ok | {:error, term()}
  def set(robot_module, path, value) when is_atom(robot_module) and is_list(path) do
    GenServer.call(Runtime.via(robot_module), {:set_parameter, path, value})
  end

  @doc """
  Set multiple parameters atomically.

  All parameters are validated before any are written. If any validation fails,
  no parameters are changed.

  ## Examples

      :ok = BB.Parameter.set_many(MyRobot, [
        {[:controller, :pid, :kp], 2.0},
        {[:controller, :pid, :ki], 0.2}
      ])
  """
  @spec set_many(module(), [{[atom()], term()}]) :: :ok | {:error, [{[atom()], term()}]}
  def set_many(robot_module, params) when is_atom(robot_module) and is_list(params) do
    GenServer.call(Runtime.via(robot_module), {:set_parameters, params})
  end

  @doc """
  List all parameters, optionally filtered by path prefix.

  Returns a list of `{path, metadata}` tuples where metadata includes
  the current value, type, and other schema information.

  ## Options

  - `:prefix` - Only return parameters under this path prefix (default: `[]`)

  ## Examples

      # All parameters
      params = BB.Parameter.list(MyRobot)

      # Parameters under [:controller]
      params = BB.Parameter.list(MyRobot, prefix: [:controller])
  """
  @spec list(module(), keyword()) :: [{[atom()], map()}]
  def list(robot_module, opts \\ []) when is_atom(robot_module) do
    prefix = Keyword.get(opts, :prefix, [])
    robot_state = get_robot_state(robot_module)
    RobotState.list_parameters(robot_state, prefix)
  end

  @doc """
  Register a component's parameters with the robot.

  Called by components during init to register their parameter schema.
  Parameters are initialised with default values from the schema.

  ## Examples

      def init(opts) do
        bb = Keyword.fetch!(opts, :bb)
        BB.Parameter.register(bb.robot, bb.path, __MODULE__)
        {:ok, %{bb: bb}}
      end
  """
  @spec register(module(), [atom()], module()) :: :ok | {:error, term()}
  def register(robot_module, path, component_module)
      when is_atom(robot_module) and is_list(path) and is_atom(component_module) do
    GenServer.call(Runtime.via(robot_module), {:register_parameters, path, component_module})
  end

  @doc """
  Check if a module implements the BB.Parameter behaviour.
  """
  @spec implements?(module()) :: boolean()
  def implements?(module) when is_atom(module) do
    function_exported?(module, :param_schema, 0)
  end

  # ===========================================================================
  # Remote Parameter Access (via bridges)
  # ===========================================================================

  @doc """
  List parameters available on a remote system via a bridge.

  Returns a list of parameter info from the remote (e.g., flight controller).
  The bridge must implement `list_remote/1`.

  ## Examples

      {:ok, params} = BB.Parameter.list_remote(MyRobot, :mavlink)
      # => [{id: "PITCH_RATE_P", value: 0.1, type: :float, doc: "..."}, ...]
  """
  @spec list_remote(module(), atom()) :: {:ok, [map()]} | {:error, term()}
  def list_remote(robot_module, bridge_name)
      when is_atom(robot_module) and is_atom(bridge_name) do
    GenServer.call(bridge_via(robot_module, bridge_name), :list_remote)
  end

  @doc """
  Get a parameter value from a remote system via a bridge.

  The bridge must implement `get_remote/2`.

  ## Examples

      {:ok, 0.15} = BB.Parameter.get_remote(MyRobot, :mavlink, "PITCH_RATE_P")
  """
  @spec get_remote(module(), atom(), BB.Parameter.Protocol.param_id()) ::
          {:ok, term()} | {:error, term()}
  def get_remote(robot_module, bridge_name, param_id)
      when is_atom(robot_module) and is_atom(bridge_name) do
    GenServer.call(bridge_via(robot_module, bridge_name), {:get_remote, param_id})
  end

  @doc """
  Set a parameter value on a remote system via a bridge.

  The bridge must implement `set_remote/3`.

  ## Examples

      :ok = BB.Parameter.set_remote(MyRobot, :mavlink, "PITCH_RATE_P", 0.15)
  """
  @spec set_remote(module(), atom(), BB.Parameter.Protocol.param_id(), term()) ::
          :ok | {:error, term()}
  def set_remote(robot_module, bridge_name, param_id, value)
      when is_atom(robot_module) and is_atom(bridge_name) do
    GenServer.call(bridge_via(robot_module, bridge_name), {:set_remote, param_id, value})
  end

  @doc """
  Subscribe to changes for a remote parameter via a bridge.

  When the remote parameter changes, the bridge publishes via `BB.PubSub`.
  The path structure is determined by the bridge implementation.

  The bridge must implement `subscribe_remote/2`.

  ## Examples

      :ok = BB.Parameter.subscribe_remote(MyRobot, :mavlink, "PITCH_RATE_P")
  """
  @spec subscribe_remote(module(), atom(), BB.Parameter.Protocol.param_id()) ::
          :ok | {:error, term()}
  def subscribe_remote(robot_module, bridge_name, param_id)
      when is_atom(robot_module) and is_atom(bridge_name) do
    GenServer.call(bridge_via(robot_module, bridge_name), {:subscribe_remote, param_id})
  end

  defp get_robot_state(robot_module) do
    Runtime.get_robot_state(robot_module)
  end

  defp bridge_via(robot_module, bridge_name) do
    BB.Process.via(robot_module, bridge_name)
  end
end
