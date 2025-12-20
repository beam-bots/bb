# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Sensor do
  @moduledoc """
  Behaviour for sensor GenServers in the BB framework.

  Sensors read from hardware or other sources and publish messages. They can
  be attached at the robot level, to links, or to joints.

  ## Usage

  The `use BB.Sensor` macro:
  - Adds `use GenServer` (you must implement GenServer callbacks)
  - Adds `@behaviour BB.Sensor`
  - Optionally defines `options_schema/0` if you pass the `:options_schema` option

  ## Options Schema

  If your sensor accepts configuration options, pass them via `:options_schema`:

      defmodule MyTemperatureSensor do
        use BB.Sensor,
          options_schema: [
            bus: [type: :string, required: true, doc: "I2C bus name"],
            address: [type: :integer, required: true, doc: "I2C device address"],
            poll_interval_ms: [type: :pos_integer, default: 1000, doc: "Poll interval"]
          ]

        @impl GenServer
        def init(opts) do
          # Options already validated at compile time
          bus = Keyword.fetch!(opts, :bus)
          address = Keyword.fetch!(opts, :address)
          bb = Keyword.fetch!(opts, :bb)
          {:ok, %{bus: bus, address: address, bb: bb}}
        end
      end

  You can override the generated `options_schema/0` if needed:

      defmodule MySensor do
        use BB.Sensor, options_schema: [frequency: [type: :pos_integer, default: 50]]

        @impl BB.Sensor
        def options_schema do
          # Custom implementation
          Spark.Options.new!([...])
        end
      end

  For sensors that don't need configuration, omit `:options_schema`:

      defmodule SimpleSensor do
        use BB.Sensor

        # Must be used as bare module in DSL: sensor :temp, SimpleSensor

        @impl GenServer
        def init(opts) do
          bb = Keyword.fetch!(opts, :bb)
          {:ok, %{bb: bb}}
        end
      end

  ## Safety

  Most sensors don't require safety callbacks since they only read data.
  If your sensor controls hardware that needs to be disabled on disarm
  (e.g., a spinning LIDAR), implement the optional `disarm/1` callback:

      defmodule MyHardwareSensor do
        use BB.Sensor

        @impl BB.Sensor
        def disarm(opts), do: stop_hardware(opts)
      end

  ## Auto-injected Options

  The `:bb` option is automatically provided by the supervisor and should
  NOT be included in your `options_schema`. It contains `%{robot: module, path: [atom]}`.
  """

  @doc """
  Returns the options schema for this sensor.

  The schema should NOT include the `:bb` option - it is auto-injected.
  If this callback is not implemented, the module cannot accept options
  in the DSL (must be used as a bare module).
  """
  @callback options_schema() :: Spark.Options.t()

  @doc """
  Make the hardware safe.

  Called with the opts provided at registration. Must work without GenServer state.
  This callback is optional for sensors - only implement it if your sensor
  controls hardware that needs to be disabled on disarm (e.g., a spinning LIDAR).
  """
  @callback disarm(opts :: keyword()) :: :ok | {:error, term()}

  @optional_callbacks [options_schema: 0, disarm: 1]

  @doc false
  defmacro __using__(opts) do
    schema_opts = opts[:options_schema]

    quote do
      use GenServer
      @behaviour BB.Sensor

      unquote(
        if schema_opts do
          quote do
            @__bb_options_schema Spark.Options.new!(unquote(schema_opts))

            @impl BB.Sensor
            def options_schema, do: @__bb_options_schema

            defoverridable options_schema: 0
          end
        end
      )
    end
  end
end
