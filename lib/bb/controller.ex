# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Controller do
  @moduledoc """
  Behaviour for controllers in the BB framework.

  Controllers manage hardware communication (I2C buses, serial ports, etc.)
  and are typically shared by multiple actuators. They run at the robot level
  and are supervised by `BB.ControllerSupervisor`.

  ## Usage

  The `use BB.Controller` macro sets up your module as a controller callback module.
  Your module is NOT a GenServer - the framework provides a wrapper GenServer
  (`BB.Controller.Server`) that delegates to your callbacks.

  ### Required Callbacks

  - `init/1` - Initialise controller state from resolved options

  ### Optional Callbacks

  - `disarm/1` - Make hardware safe (only for controllers with active hardware)
  - `handle_options/2` - React to parameter changes at runtime
  - `handle_call/3`, `handle_cast/2`, `handle_info/2` - Standard GenServer-style callbacks
  - `handle_continue/2`, `terminate/2` - Lifecycle callbacks
  - `options_schema/0` - Define accepted configuration options

  ### Options Schema

  If your controller accepts configuration options, pass them via `:options_schema`:

      defmodule MyI2CController do
        use BB.Controller,
          options_schema: [
            bus: [type: :string, required: true, doc: "I2C bus name"],
            address: [type: :integer, required: true, doc: "I2C device address"]
          ]

        @impl BB.Controller
        def init(opts) do
          bus = Keyword.fetch!(opts, :bus)
          address = Keyword.fetch!(opts, :address)
          bb = Keyword.fetch!(opts, :bb)
          {:ok, %{bus: bus, address: address, bb: bb}}
        end
      end

  For controllers that don't need configuration, omit `:options_schema`:

      defmodule SimpleController do
        use BB.Controller

        @impl BB.Controller
        def init(opts) do
          {:ok, %{bb: opts[:bb]}}
        end
      end

  ### Parameter References

  Options can reference parameters for runtime-adjustable configuration:

      controller :i2c, {MyI2CController, bus: param([:hardware, :i2c_bus])}

  When the parameter changes, `handle_options/2` is called with the new resolved
  options. Override it to update your state accordingly.

  ### Auto-injected Options

  The `:bb` option is automatically provided and should NOT be included in your
  `options_schema`. It contains `%{robot: module, path: [atom]}`.

  ### Safety Registration

  If your controller manages hardware that needs to be made safe when disarmed,
  implement the optional `disarm/1` callback:

      defmodule MyController do
        use BB.Controller, options_schema: [bus: [type: :string, required: true]]

        @impl BB.Controller
        def init(opts), do: {:ok, %{}}

        @impl BB.Controller
        def disarm(opts), do: disable_hardware(opts[:bus])
      end

  When `disarm/1` is implemented, the framework automatically registers your
  controller with `BB.Safety`.
  """

  # ----------------------------------------------------------------------------
  # Behaviour
  # ----------------------------------------------------------------------------

  @doc """
  Initialise controller state from resolved options.

  Called with options after parameter references have been resolved.
  The `:bb` key contains `%{robot: module, path: [atom]}`.

  Return `{:ok, state}` or `{:ok, state, timeout_or_continue}` on success,
  `{:stop, reason}` to abort startup, or `:ignore` to skip this controller.
  """
  @callback init(opts :: keyword()) ::
              {:ok, state :: term()}
              | {:ok, state :: term(), timeout() | :hibernate | {:continue, term()}}
              | {:stop, reason :: term()}
              | :ignore

  @doc """
  Make the hardware safe.

  Called with the opts provided at registration. Must work without GenServer state.
  Only implement this if your controller manages hardware that needs to be disabled
  when the robot is disarmed or crashes.
  """
  @callback disarm(opts :: keyword()) :: :ok | {:error, term()}

  @doc """
  Handle parameter changes at runtime.

  Called when a referenced parameter changes. The `new_opts` contain all options
  with the updated parameter value(s) resolved.

  Return `{:ok, new_state}` to update state, or `{:stop, reason}` to shut down.
  """
  @callback handle_options(new_opts :: keyword(), state :: term()) ::
              {:ok, new_state :: term()} | {:stop, reason :: term()}

  @doc """
  Handle synchronous calls.

  Same semantics as `c:GenServer.handle_call/3`.
  """
  @callback handle_call(request :: term(), from :: GenServer.from(), state :: term()) ::
              {:reply, reply :: term(), new_state :: term()}
              | {:reply, reply :: term(), new_state :: term(),
                 timeout() | :hibernate | {:continue, term()}}
              | {:noreply, new_state :: term()}
              | {:noreply, new_state :: term(), timeout() | :hibernate | {:continue, term()}}
              | {:stop, reason :: term(), new_state :: term()}
              | {:stop, reason :: term(), reply :: term(), new_state :: term()}

  @doc """
  Handle asynchronous casts.

  Same semantics as `c:GenServer.handle_cast/2`.
  """
  @callback handle_cast(request :: term(), state :: term()) ::
              {:noreply, new_state :: term()}
              | {:noreply, new_state :: term(), timeout() | :hibernate | {:continue, term()}}
              | {:stop, reason :: term(), new_state :: term()}

  @doc """
  Handle all other messages.

  Same semantics as `c:GenServer.handle_info/2`.
  """
  @callback handle_info(msg :: term(), state :: term()) ::
              {:noreply, new_state :: term()}
              | {:noreply, new_state :: term(), timeout() | :hibernate | {:continue, term()}}
              | {:stop, reason :: term(), new_state :: term()}

  @doc """
  Handle continue instructions.

  Same semantics as `c:GenServer.handle_continue/2`.
  """
  @callback handle_continue(continue_arg :: term(), state :: term()) ::
              {:noreply, new_state :: term()}
              | {:noreply, new_state :: term(), timeout() | :hibernate | {:continue, term()}}
              | {:stop, reason :: term(), new_state :: term()}

  @doc """
  Clean up before termination.

  Same semantics as `c:GenServer.terminate/2`.
  """
  @callback terminate(reason :: term(), state :: term()) :: term()

  @doc """
  Returns the options schema for this controller.

  The schema should NOT include the `:bb` option - it is auto-injected.
  If this callback is not implemented, the module cannot accept options
  in the DSL (must be used as a bare module).
  """
  @callback options_schema() :: Spark.Options.t()

  @optional_callbacks [
    options_schema: 0,
    disarm: 1,
    handle_options: 2,
    handle_call: 3,
    handle_cast: 2,
    handle_info: 2,
    handle_continue: 2,
    terminate: 2
  ]

  @doc false
  defmacro __using__(opts) do
    schema_opts = opts[:options_schema]

    quote do
      @behaviour BB.Controller

      # Default implementations - all overridable
      @impl BB.Controller
      def handle_options(_new_opts, state), do: {:ok, state}

      @impl BB.Controller
      def handle_call(_request, _from, state), do: {:reply, {:error, :not_implemented}, state}

      @impl BB.Controller
      def handle_cast(_request, state), do: {:noreply, state}

      @impl BB.Controller
      def handle_info(_msg, state), do: {:noreply, state}

      @impl BB.Controller
      def handle_continue(_continue_arg, state), do: {:noreply, state}

      @impl BB.Controller
      def terminate(_reason, _state), do: :ok

      defoverridable handle_options: 2,
                     handle_call: 3,
                     handle_cast: 2,
                     handle_info: 2,
                     handle_continue: 2,
                     terminate: 2

      unquote(
        if schema_opts do
          quote do
            @__bb_options_schema Spark.Options.new!(unquote(schema_opts))

            @impl BB.Controller
            def options_schema, do: @__bb_options_schema

            defoverridable options_schema: 0
          end
        end
      )
    end
  end
end
