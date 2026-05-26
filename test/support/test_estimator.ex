# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule EchoEstimator do
  @moduledoc false
  # Minimal test estimator: echoes whatever input it receives back on :out.
  use BB.Estimator

  @impl BB.Estimator
  def init(opts) do
    {:ok,
     %{
       bb: Keyword.fetch!(opts, :bb),
       context: Keyword.fetch!(opts, :estimator_context)
     }}
  end

  @impl BB.Estimator
  def handle_input(%BB.Message{} = msg, state), do: {:reply, [out: msg], state}
  def handle_input(_multi, state), do: {:reply, [], state}
end

defmodule MultiInputEstimator do
  @moduledoc false
  # Test estimator that requires fan-in: sends the fanned-in bundle to the
  # `:estimator_test_pid` registered in `:persistent_term` and emits nothing.
  use BB.Estimator

  @impl BB.Estimator
  def init(opts) do
    {:ok, %{bb: Keyword.fetch!(opts, :bb)}}
  end

  @impl BB.Estimator
  def handle_input(input, state) when is_map(input) and not is_struct(input, BB.Message) do
    case :persistent_term.get(:estimator_test_pid, nil) do
      nil -> :noop
      pid -> send(pid, {:multi_input, input})
    end

    {:noreply, state}
  end

  @impl BB.Estimator
  def handle_input(%BB.Message{}, state), do: {:noreply, state}
end

defmodule TickingEstimator do
  @moduledoc false
  # Emits an :out message on a tick, demonstrating the {:reply, outputs, state}
  # shape from handle_info.
  use BB.Estimator

  alias BB.Math.Quaternion
  alias BB.Math.Vec3
  alias BB.Message
  alias BB.Message.Sensor.Imu

  @impl BB.Estimator
  def init(opts) do
    Process.send_after(self(), :tick, 5)
    {:ok, %{bb: Keyword.fetch!(opts, :bb)}}
  end

  @impl BB.Estimator
  def handle_input(_input, state), do: {:noreply, state}

  @impl BB.Estimator
  def handle_info(:tick, state) do
    {:ok, msg} =
      Message.new(Imu, :test_frame,
        orientation: Quaternion.identity(),
        angular_velocity: Vec3.zero(),
        linear_acceleration: Vec3.zero()
      )

    {:reply, [out: msg], state}
  end
end
