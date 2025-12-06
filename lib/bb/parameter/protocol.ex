# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Parameter.Protocol do
  @moduledoc """
  Behaviour for parameter protocol transports (bridges).

  Bridges provide bidirectional parameter access between BB and remote systems
  (flight controllers, GCS, web UIs, etc.).

  ## Two Directions

  **Outbound (local → remote):** Expose BB's parameters to remote clients
  - Subscribe to `[:param]` via `BB.PubSub` in `init/2`
  - Implement `handle_change/3` to push local changes to remote clients
  - Remote clients query local params via bridge (calls `BB.Parameter.list/get/set`)

  **Inbound (remote → local):** Access remote system's parameters from BB
  - Implement `list_remote/1` to enumerate remote parameters
  - Implement `get_remote/2` to read remote values
  - Implement `set_remote/3` to write remote values
  - Implement `subscribe_remote/2` to subscribe to remote changes
  - Publish remote changes via PubSub (path structure up to bridge)

  ## IEx Usage

  ```elixir
  # List remote parameters (e.g., ArduPilot's params)
  {:ok, params} = BB.Parameter.list_remote(MyRobot, :mavlink)
  # => [%{id: "PITCH_RATE_P", value: 0.1, path: [:mavlink, :pitch, :rate, :p], ...}, ...]

  # Get a remote parameter
  {:ok, value} = BB.Parameter.get_remote(MyRobot, :mavlink, "PITCH_RATE_P")
  # => 0.1

  # Set a remote parameter
  :ok = BB.Parameter.set_remote(MyRobot, :mavlink, "PITCH_RATE_P", 0.15)

  # Subscribe to remote parameter changes (tells bridge to track this param)
  :ok = BB.Parameter.subscribe_remote(MyRobot, :mavlink, "PITCH_RATE_P")

  # Then subscribe to PubSub using the path from list_remote
  BB.PubSub.subscribe(MyRobot, [:mavlink, :pitch, :rate, :p])
  ```

  ## Example Implementation

  ```elixir
  defmodule MyMavlinkBridge do
    use GenServer
    @behaviour BB.Parameter.Protocol

    # Define a payload type for remote param change messages
    defmodule ParamValue do
      @behaviour BB.Message
      defstruct [:value]

      @schema Spark.Options.new!(value: [type: :any, required: true])
      @impl BB.Message
      def schema, do: @schema

      defimpl BB.Message.Payload do
        def schema(_), do: @for.schema()
      end
    end

    # GenServer init extracts robot from :bb metadata
    @impl GenServer
    def init(opts) do
      %{robot: robot} = Keyword.fetch!(opts, :bb)
      {:ok, state} = __MODULE__.init(robot, Keyword.delete(opts, :bb))
      {:ok, state}
    end

    # Protocol init
    @impl BB.Parameter.Protocol
    def init(robot, opts) do
      BB.PubSub.subscribe(robot, [:param])
      conn = connect_to_mavlink(opts[:conn])
      {:ok, %{robot: robot, conn: conn, subscriptions: MapSet.new()}}
    end

    # Outbound: local param changed, notify remote
    @impl BB.Parameter.Protocol
    def handle_change(_robot, changed, state) do
      send_param_to_gcs(state.conn, changed)
      {:ok, state}
    end

    # Inbound: list remote params
    @impl BB.Parameter.Protocol
    def list_remote(state) do
      # Return params with path for PubSub subscriptions
      params = Enum.map(fetch_all_params_from_fc(state.conn), fn {id, value} ->
        %{id: id, value: value, type: nil, doc: nil, path: param_id_to_path(id)}
      end)
      {:ok, params, state}
    end

    # Inbound: get remote param
    @impl BB.Parameter.Protocol
    def get_remote(param_id, state) do
      value = fetch_param_from_fc(state.conn, param_id)
      {:ok, value, state}
    end

    # Inbound: set remote param
    @impl BB.Parameter.Protocol
    def set_remote(param_id, value, state) do
      :ok = send_param_set_to_fc(state.conn, param_id, value)
      {:ok, state}
    end

    # Inbound: subscribe to remote param changes
    @impl BB.Parameter.Protocol
    def subscribe_remote(param_id, state) do
      {:ok, %{state | subscriptions: MapSet.put(state.subscriptions, param_id)}}
    end

    # When FC sends param update, publish via PubSub
    @impl GenServer
    def handle_info({:mavlink_param_value, param_id, value}, state) do
      if MapSet.member?(state.subscriptions, param_id) do
        path = param_id_to_path(param_id)
        message = BB.Message.new!(ParamValue, :remote, value: value)
        BB.PubSub.publish(state.robot, path, message)
      end
      {:noreply, state}
    end

    # Convert "PITCH_RATE_P" to [:mavlink, :pitch, :rate, :p]
    defp param_id_to_path(param_id) do
      atoms = param_id |> String.downcase() |> String.split("_") |> Enum.map(&String.to_atom/1)
      [:mavlink | atoms]
    end
  end
  ```
  """

  @type state :: term()
  @type robot :: module()
  @type param_id :: String.t() | atom()

  @type remote_param :: %{
          id: param_id(),
          value: term(),
          type: atom() | nil,
          doc: String.t() | nil,
          path: [atom()] | nil
        }

  # ==========================================================================
  # Outbound: local → remote
  # ==========================================================================

  @doc """
  Initialise transport state for a robot.

  Called when the bridge process starts. Should set up connections and
  subscribe to `[:param]` via `BB.PubSub` for local change notifications.
  """
  @callback init(robot, opts :: keyword()) :: {:ok, state} | {:error, term()}

  @doc """
  Handle a local parameter change.

  Called when a BB parameter changes locally. The bridge should notify
  any subscribed remote clients.
  """
  @callback handle_change(robot, changed :: BB.Parameter.Changed.t(), state) :: {:ok, state}

  # ==========================================================================
  # Inbound: remote → local
  # ==========================================================================

  @doc """
  List parameters available on the remote system.

  Returns a list of parameter info from the remote (e.g., flight controller).
  """
  @callback list_remote(state) :: {:ok, [remote_param()], state} | {:error, term(), state}

  @doc """
  Get a parameter value from the remote system.
  """
  @callback get_remote(param_id, state) :: {:ok, term(), state} | {:error, term(), state}

  @doc """
  Set a parameter value on the remote system.
  """
  @callback set_remote(param_id, value :: term(), state) ::
              {:ok, state} | {:error, term(), state}

  @doc """
  Subscribe to changes for a remote parameter.

  When the remote parameter changes, the bridge should publish via
  `BB.PubSub`. The path structure is up to the bridge implementation.
  """
  @callback subscribe_remote(param_id, state) :: {:ok, state} | {:error, term(), state}

  @optional_callbacks [list_remote: 1, get_remote: 2, set_remote: 3, subscribe_remote: 2]
end
