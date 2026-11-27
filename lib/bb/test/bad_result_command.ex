# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Test.BadResultCommand do
  @moduledoc """
  Test double: a command whose `result/1` breaks its contract.

  The goal picks the failure: `%{result: term}` returns that term verbatim,
  `%{raise: true}` raises instead.
  """
  use BB.Command

  @impl BB.Command
  def handle_command(%{raise: true}, _context, state) do
    {:stop, :normal, Map.put(state, :raise, true)}
  end

  def handle_command(goal, _context, state) do
    {:stop, :normal, %{state | result: Map.get(goal, :result)}}
  end

  @impl BB.Command
  def result(%{raise: true}), do: raise("boom")
  def result(%{result: result}), do: result
end
