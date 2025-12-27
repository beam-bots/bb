# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Sim.Controller do
  @moduledoc """
  Mock controller for simulation mode.

  Used when a controller is configured with `simulation: :mock`. Accepts all
  commands but does nothing with them. Useful when actuators need to call
  controllers for state queries but no hardware is present.

  ## Example

      controllers do
        controller :pca9685, {BB.Servo.PCA9685.Controller, bus: "i2c-1"},
          simulation: :mock
      end
  """

  use BB.Controller, options_schema: []

  @impl BB.Controller
  def init(opts) do
    {:ok, %{bb: opts[:bb]}}
  end

  @impl BB.Controller
  def handle_call(_request, _from, state) do
    {:reply, :ok, state}
  end

  @impl BB.Controller
  def handle_cast(_request, state) do
    {:noreply, state}
  end
end
