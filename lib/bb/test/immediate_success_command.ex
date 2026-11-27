# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Test.ImmediateSuccessCommand do
  @moduledoc """
  Test double: a command that succeeds with `{:ok, :done}` as soon as it is
  handled, whatever its goal.

  For a robot fixture that needs a command to exist - as the target of an
  estimator's `on_degraded`, say - without the test being about what the
  command does.
  """
  use BB.Command

  @impl BB.Command
  def handle_command(_goal, _context, state) do
    {:stop, :normal, %{state | result: {:ok, :done}}}
  end

  @impl BB.Command
  def result(%{result: result}), do: result
end
