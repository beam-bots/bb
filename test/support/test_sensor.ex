# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule MySensor do
  @moduledoc """
  A minimal test sensor that implements BB.Sensor behaviour.
  Used in DSL tests where a sensor module is required.
  """
  use BB.Sensor,
    options_schema: [
      frequency: [
        type: :pos_integer,
        doc: "Sample frequency",
        default: 50
      ]
    ]

  @impl GenServer
  def init(opts) do
    {:ok, %{bb: Keyword.fetch!(opts, :bb)}}
  end
end

# Aliases for various test module names used in sensor_test.exs

defmodule CameraSensor do
  @moduledoc false
  use BB.Sensor

  @impl GenServer
  def init(opts), do: {:ok, %{bb: Keyword.fetch!(opts, :bb)}}
end

defmodule ImuSensor do
  @moduledoc false
  use BB.Sensor

  @impl GenServer
  def init(opts), do: {:ok, %{bb: Keyword.fetch!(opts, :bb)}}
end

defmodule GpsSensor do
  @moduledoc false
  use BB.Sensor, options_schema: [port: [type: :string, required: false]]

  @impl GenServer
  def init(opts), do: {:ok, %{bb: Keyword.fetch!(opts, :bb)}}
end

defmodule BaseSensor do
  @moduledoc false
  use BB.Sensor

  @impl GenServer
  def init(opts), do: {:ok, %{bb: Keyword.fetch!(opts, :bb)}}
end

defmodule ChildSensor do
  @moduledoc false
  use BB.Sensor

  @impl GenServer
  def init(opts), do: {:ok, %{bb: Keyword.fetch!(opts, :bb)}}
end

defmodule Encoder do
  @moduledoc false
  use BB.Sensor, options_schema: [bus: [type: :atom, required: false]]

  @impl GenServer
  def init(opts), do: {:ok, %{bb: Keyword.fetch!(opts, :bb)}}
end

defmodule RobotSensor do
  @moduledoc false
  use BB.Sensor

  @impl GenServer
  def init(opts), do: {:ok, %{bb: Keyword.fetch!(opts, :bb)}}
end

defmodule LinkSensor do
  @moduledoc false
  use BB.Sensor

  @impl GenServer
  def init(opts), do: {:ok, %{bb: Keyword.fetch!(opts, :bb)}}
end

defmodule JointSensor do
  @moduledoc false
  use BB.Sensor

  @impl GenServer
  def init(opts), do: {:ok, %{bb: Keyword.fetch!(opts, :bb)}}
end

defmodule SomeSensor do
  @moduledoc false
  use BB.Sensor

  @impl GenServer
  def init(opts), do: {:ok, %{bb: Keyword.fetch!(opts, :bb)}}
end
