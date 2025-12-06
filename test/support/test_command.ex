# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Test.ImmediateSuccessCommand do
  @moduledoc false
  @behaviour BB.Command

  @impl true
  def handle_command(_goal, _context), do: {:ok, :done}
end

defmodule BB.Test.AsyncCommand do
  @moduledoc false
  @behaviour BB.Command

  @impl true
  def handle_command(%{notify: pid}, _context) do
    send(pid, :executing)
    Process.sleep(50)
    {:ok, :completed}
  end

  def handle_command(_goal, _context), do: {:ok, :completed}
end

defmodule BB.Test.RejectingCommand do
  @moduledoc false
  @behaviour BB.Command

  @impl true
  def handle_command(_goal, _context), do: {:error, :not_allowed}
end
