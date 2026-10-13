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
