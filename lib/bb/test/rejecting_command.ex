# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Test.RejectingCommand do
  @moduledoc """
  Test double: a command that fails with `{:error, :not_allowed}` as soon as
  it is handled, whatever its goal.
  """
  use BB.Command

  @impl BB.Command
  def handle_command(_goal, _context, state) do
    {:stop, :normal, %{state | result: {:error, :not_allowed}}}
  end

  @impl BB.Command
  def result(%{result: result}), do: result
end
