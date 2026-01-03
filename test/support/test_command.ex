# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Test.ImmediateSuccessCommand do
  @moduledoc false
  use BB.Command

  @impl BB.Command
  def handle_command(_goal, _context, state) do
    {:stop, :normal, %{state | result: {:ok, :done}}}
  end

  @impl BB.Command
  def result(%{result: result}), do: result
end

defmodule BB.Test.AsyncCommand do
  @moduledoc false
  use BB.Command

  @impl BB.Command
  def handle_command(%{notify: pid}, _context, state) do
    send(pid, :executing)
    Process.send_after(self(), :complete, 50)
    {:noreply, state}
  end

  def handle_command(_goal, _context, state) do
    {:stop, :normal, %{state | result: {:ok, :completed}}}
  end

  @impl BB.Command
  def handle_info(:complete, state) do
    {:stop, :normal, %{state | result: {:ok, :completed}}}
  end

  def handle_info(_msg, state) do
    {:noreply, state}
  end

  @impl BB.Command
  def result(%{result: nil}), do: {:error, :cancelled}
  def result(%{result: result}), do: result
end

defmodule BB.Test.RejectingCommand do
  @moduledoc false
  use BB.Command

  @impl BB.Command
  def handle_command(_goal, _context, state) do
    {:stop, :normal, %{state | result: {:error, :not_allowed}}}
  end

  @impl BB.Command
  def result(%{result: result}), do: result
end
