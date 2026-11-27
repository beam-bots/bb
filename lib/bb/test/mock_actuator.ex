# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Test.MockActuator do
  @moduledoc """
  Test double: an actuator that accepts every command and does nothing.

  For a robot fixture that needs an actuator in the topology but whose test
  isn't about actuation. It claims `:position_feedback`, standing in for a
  smart servo that reads its own position back, so a fixture doesn't trip
  `BB.Dsl.Verifiers.ValidatePositionFeedback` for a sensor it has no use
  for. Its `disarm/1` always succeeds.
  """
  use BB.Actuator, options_schema: []

  @impl BB.Actuator
  def disarm(_opts), do: :ok

  @impl BB.Actuator
  def capabilities(_opts), do: [:position_feedback]

  @impl BB.Actuator
  def init(opts) do
    {:ok, %{opts: opts}}
  end

  @impl BB.Actuator
  def handle_command(_message, state), do: {:noreply, state}
end
