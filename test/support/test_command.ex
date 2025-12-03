# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Test.ImmediateSuccessCommand do
  @moduledoc false
  @behaviour Kinetix.Command

  @impl true
  def handle_command(_goal, _context), do: {:ok, :done}
end

defmodule Kinetix.Test.AsyncCommand do
  @moduledoc false
  @behaviour Kinetix.Command

  @impl true
  def handle_command(%{notify: pid}, _context) do
    send(pid, :executing)
    Process.sleep(50)
    {:ok, :completed}
  end

  def handle_command(_goal, _context), do: {:ok, :completed}
end

defmodule Kinetix.Test.RejectingCommand do
  @moduledoc false
  @behaviour Kinetix.Command

  @impl true
  def handle_command(_goal, _context), do: {:error, :not_allowed}
end
