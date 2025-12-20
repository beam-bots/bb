# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Controller do
  @moduledoc """
  Behaviour for controller GenServers in the BB framework.

  Controllers manage hardware communication (I2C buses, serial ports, etc.)
  and are typically shared by multiple actuators. They run at the robot level
  and are supervised by `BB.ControllerSupervisor`.

  ## Usage

  The `use BB.Controller` macro:
  - Adds `use GenServer` (you must implement GenServer callbacks)
  - Adds `@behaviour BB.Controller`
  - Optionally defines `options_schema/0` if you pass the `:options_schema` option

  ## Options Schema

  If your controller accepts configuration options, pass them via `:options_schema`:

      defmodule MyI2CController do
        use BB.Controller,
          options_schema: [
            bus: [type: :string, required: true, doc: "I2C bus name"],
            address: [type: :integer, required: true, doc: "I2C device address"]
          ]

        @impl GenServer
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

        # Must be used as bare module in DSL: controller :foo, SimpleController

        @impl GenServer
        def init(opts) do
          bb = Keyword.fetch!(opts, :bb)
          {:ok, %{bb: bb}}
        end
      end

  ## Safety

  If your controller manages hardware that needs to be made safe when disarmed,
  implement the optional `disarm/1` callback:

      defmodule MyController do
        use BB.Controller, options_schema: [bus: [type: :string, required: true]]

        @impl BB.Controller
        def disarm(opts), do: disable_hardware(opts[:bus])
      end

  ## Auto-injected Options

  The `:bb` option is automatically provided by the supervisor and should
  NOT be included in your `options_schema`. It contains `%{robot: module, path: [atom]}`.
  """

  @doc """
  Returns the options schema for this controller.

  The schema should NOT include the `:bb` option - it is auto-injected.
  If this callback is not implemented, the module cannot accept options
  in the DSL (must be used as a bare module).
  """
  @callback options_schema() :: Spark.Options.t()

  @doc """
  Make the hardware safe.

  Called with the opts provided at registration. Must work without GenServer state.
  Only implement this if your controller manages hardware that needs to be disabled
  when the robot is disarmed or crashes.
  """
  @callback disarm(opts :: keyword()) :: :ok | {:error, term()}

  @optional_callbacks [options_schema: 0, disarm: 1]

  @doc false
  defmacro __using__(opts) do
    schema_opts = opts[:options_schema]

    quote do
      use GenServer
      @behaviour BB.Controller

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
