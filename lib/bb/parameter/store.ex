# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Parameter.Store do
  @moduledoc """
  Behaviour for parameter persistence backends.

  Implementations handle loading and saving parameter values to durable storage.
  The store is initialized when a robot starts and closed when it stops.

  ## Lifecycle

  1. `init/2` - Called during robot startup with module name and options
  2. `load/1` - Called to retrieve all persisted parameters
  3. `save/3` - Called after each successful parameter change
  4. `close/1` - Called during robot shutdown

  ## Built-in Implementations

  - `BB.Parameter.Store.Dets` - Disk-backed storage using OTP's `:dets`

  ## Example Implementation

  ```elixir
  defmodule MyApp.ParameterStore do
    @behaviour BB.Parameter.Store

    @impl true
    def init(robot_module, opts) do
      # Initialize storage connection
      {:ok, %{robot: robot_module, conn: connect(opts)}}
    end

    @impl true
    def load(state) do
      # Return all stored parameters
      {:ok, fetch_all(state.conn)}
    end

    @impl true
    def save(state, path, value) do
      # Persist a parameter change
      :ok = write(state.conn, path, value)
      :ok
    end

    @impl true
    def close(state) do
      # Clean up resources
      disconnect(state.conn)
      :ok
    end
  end
  ```
  """

  @type state :: term()
  @type path :: [atom()]
  @type value :: term()

  @doc """
  Initialize the parameter store.

  Called during robot startup. The `robot_module` identifies which robot
  this store is for (useful for multi-robot setups). Options come from
  the DSL configuration.

  Returns `{:ok, state}` where state is passed to subsequent callbacks,
  or `{:error, reason}` if initialization fails.
  """
  @callback init(robot_module :: module(), opts :: keyword()) ::
              {:ok, state()} | {:error, term()}

  @doc """
  Load all persisted parameters.

  Called after initialization to retrieve previously saved parameter values.
  Returns a list of `{path, value}` tuples.

  These values are applied after DSL defaults, so persisted values take
  precedence over defaults.
  """
  @callback load(state()) ::
              {:ok, [{path(), value()}]} | {:error, term()}

  @doc """
  Save a parameter value.

  Called after each successful `BB.Parameter.set/3` operation.
  The implementation should persist the value durably.
  """
  @callback save(state(), path(), value()) :: :ok | {:error, term()}

  @doc """
  Close the parameter store.

  Called during robot shutdown. Implementations should release any
  resources (file handles, connections, etc.).
  """
  @callback close(state()) :: :ok
end
