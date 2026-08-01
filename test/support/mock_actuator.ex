# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Test.MockActuator do
  @moduledoc """
  Minimal mock actuator for testing.
  """
  use BB.Actuator, options_schema: []

  @impl BB.Actuator
  def disarm(_opts), do: :ok

  @impl BB.Actuator
  def init(opts) do
    {:ok, %{opts: opts}}
  end

  @impl BB.Actuator
  def handle_command(_message, state), do: {:noreply, state}
end

# Aliases for various test module names
defmodule ServoMotor do
  @moduledoc false
  use BB.Actuator,
    options_schema: [
      pwm_pin: [type: :pos_integer, required: false],
      frequency: [type: :pos_integer, required: false]
    ]

  @impl BB.Actuator
  def disarm(_opts), do: :ok

  @impl BB.Actuator
  def init(opts), do: {:ok, %{bb: Keyword.fetch!(opts, :bb)}}

  @impl BB.Actuator
  def handle_command(_message, state), do: {:noreply, state}
end

defmodule MainMotor do
  @moduledoc false
  use BB.Actuator

  @impl BB.Actuator
  def disarm(_opts), do: :ok

  @impl BB.Actuator
  def init(opts), do: {:ok, %{bb: Keyword.fetch!(opts, :bb)}}

  @impl BB.Actuator
  def handle_command(_message, state), do: {:noreply, state}
end

defmodule BrakeActuator do
  @moduledoc false
  use BB.Actuator, options_schema: [pin: [type: :pos_integer, required: false]]

  @impl BB.Actuator
  def disarm(_opts), do: :ok

  @impl BB.Actuator
  def init(opts), do: {:ok, %{bb: Keyword.fetch!(opts, :bb)}}

  @impl BB.Actuator
  def handle_command(_message, state), do: {:noreply, state}
end

defmodule ShoulderMotor do
  @moduledoc false
  use BB.Actuator

  @impl BB.Actuator
  def disarm(_opts), do: :ok

  @impl BB.Actuator
  def init(opts), do: {:ok, %{bb: Keyword.fetch!(opts, :bb)}}

  @impl BB.Actuator
  def handle_command(_message, state), do: {:noreply, state}
end

defmodule ElbowMotor do
  @moduledoc false
  use BB.Actuator

  @impl BB.Actuator
  def disarm(_opts), do: :ok

  @impl BB.Actuator
  def init(opts), do: {:ok, %{bb: Keyword.fetch!(opts, :bb)}}

  @impl BB.Actuator
  def handle_command(_message, state), do: {:noreply, state}
end

defmodule MyMotor do
  @moduledoc false
  use BB.Actuator

  @impl BB.Actuator
  def disarm(_opts), do: :ok

  @impl BB.Actuator
  def init(opts), do: {:ok, %{bb: Keyword.fetch!(opts, :bb)}}

  @impl BB.Actuator
  def handle_command(_message, state), do: {:noreply, state}
end

defmodule TestActuator do
  @moduledoc false
  use BB.Actuator,
    options_schema: [
      pin: [type: :pos_integer, required: false],
      pwm_frequency: [type: :pos_integer, required: false]
    ]

  @impl BB.Actuator
  def disarm(_opts), do: :ok

  @impl BB.Actuator
  def init(opts), do: {:ok, %{bb: Keyword.fetch!(opts, :bb)}}

  @impl BB.Actuator
  def handle_command(_message, state), do: {:noreply, state}
end
