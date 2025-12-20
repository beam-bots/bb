# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Bridge do
  @moduledoc """
  Behaviour for parameter bridge GenServers in the BB framework.

  Bridges provide bidirectional parameter access between BB and remote systems
  (GCS, web UIs, flight controllers).

  Bridges do NOT implement safety callbacks - they handle data transport,
  not physical hardware control.

  ## Two Directions

  **Outbound (local → remote):** Expose BB's parameters to remote clients
  - Subscribe to `[:param]` via `BB.PubSub` in GenServer `init/1`
  - Implement `handle_change/3` to push local changes to remote clients
  - Remote clients query local params via bridge (calls `BB.Parameter.list/get/set`)

  **Inbound (remote → local):** Access remote system's parameters from BB
  - Implement `list_remote/1` to enumerate remote parameters
  - Implement `get_remote/2` to read remote values
  - Implement `set_remote/3` to write remote values
  - Implement `subscribe_remote/2` to subscribe to remote changes
  - Publish remote changes via PubSub (path structure up to bridge)

  ## Usage

  The `use BB.Bridge` macro:
  - Adds `use GenServer` (you must implement GenServer callbacks)
  - Adds `@behaviour BB.Bridge`
  - Optionally defines `options_schema/0` if you pass the `:options_schema` option

  ## Options Schema

  If your bridge accepts configuration options, pass them via `:options_schema`:

      defmodule MyMavlinkBridge do
        use BB.Bridge,
          options_schema: [
            port: [type: :string, required: true, doc: "Serial port path"],
            baud_rate: [type: :pos_integer, default: 57600, doc: "Baud rate"]
          ]

        @impl BB.Bridge
        def handle_change(_robot, changed, state) do
          send_to_gcs(state.conn, changed)
          {:ok, state}
        end

        # ... other BB.Bridge callbacks
      end

  For bridges that don't need configuration, omit `:options_schema`:

      defmodule SimpleBridge do
        use BB.Bridge

        # Must be used as bare module in DSL: bridge :simple, SimpleBridge
      end

  ## DSL Usage

      parameters do
        bridge :mavlink, {MyMavlinkBridge, port: "/dev/ttyACM0", baud_rate: 115200}
        bridge :phoenix, {PhoenixBridge, url: "ws://gcs.local/socket"}
      end

  ## Auto-injected Options

  The `:bb` option is automatically provided by the supervisor and should
  NOT be included in your `options_schema`. It contains `%{robot: module, path: [atom]}`.

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
    use BB.Bridge

    # Define a payload type for remote param change messages
    defmodule ParamValue do
      defstruct [:value]

      use BB.Message,
        schema: [value: [type: :any, required: true]]
    end

    # GenServer init - extract robot from :bb metadata, subscribe to param changes
    @impl GenServer
    def init(opts) do
      %{robot: robot} = Keyword.fetch!(opts, :bb)
      BB.PubSub.subscribe(robot, [:param])
      conn = connect_to_mavlink(opts[:conn])
      {:ok, %{robot: robot, conn: conn, subscriptions: MapSet.new()}}
    end

    # Outbound: local param changed, notify remote
    @impl BB.Bridge
    def handle_change(_robot, changed, state) do
      send_param_to_gcs(state.conn, changed)
      {:ok, state}
    end

    # Inbound: list remote params
    @impl BB.Bridge
    def list_remote(state) do
      # Return params with path for PubSub subscriptions
      params = Enum.map(fetch_all_params_from_fc(state.conn), fn {id, value} ->
        %{id: id, value: value, type: nil, doc: nil, path: param_id_to_path(id)}
      end)
      {:ok, params, state}
    end

    # Inbound: get remote param
    @impl BB.Bridge
    def get_remote(param_id, state) do
      value = fetch_param_from_fc(state.conn, param_id)
      {:ok, value, state}
    end

    # Inbound: set remote param
    @impl BB.Bridge
    def set_remote(param_id, value, state) do
      :ok = send_param_set_to_fc(state.conn, param_id, value)
      {:ok, state}
    end

    # Inbound: subscribe to remote param changes
    @impl BB.Bridge
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
  # Configuration
  # ==========================================================================

  @doc """
  Returns the options schema for this bridge.

  The schema should NOT include the `:bb` option - it is auto-injected.
  If this callback is not implemented, the module cannot accept options
  in the DSL (must be used as a bare module).
  """
  @callback options_schema() :: Spark.Options.t()

  # ==========================================================================
  # Outbound: local → remote
  # ==========================================================================

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

  @optional_callbacks [
    options_schema: 0,
    list_remote: 1,
    get_remote: 2,
    set_remote: 3,
    subscribe_remote: 2
  ]

  @doc false
  defmacro __using__(opts) do
    schema_opts = opts[:options_schema]

    quote do
      use GenServer
      @behaviour BB.Bridge

      unquote(
        if schema_opts do
          quote do
            @__bb_options_schema Spark.Options.new!(unquote(schema_opts))

            @impl BB.Bridge
            def options_schema, do: @__bb_options_schema

            defoverridable options_schema: 0
          end
        end
      )
    end
  end
end
