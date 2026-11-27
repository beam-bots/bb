# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Test.AsyncCommand do
  @moduledoc """
  Test double: a command that keeps running until it is told to finish.

  Given a goal of `%{notify: pid}`, it sends `{:executing, command_pid}` to
  `pid` once it is running, then waits for `:complete` and succeeds with
  `{:ok, :completed}`. Any other goal succeeds at once. A command stopped
  before it completes reports `{:error, :cancelled}`.

      {:ok, cmd} = BB.Robot.Runtime.execute(Robot, :async_cmd, %{notify: self()})
      assert_receive {:executing, ^cmd}
      # ... do stuff while the command is running ...
      send(cmd, :complete)
      assert {:ok, :completed} = BB.Command.await(cmd)
  """
  use BB.Command

  @impl BB.Command
  def handle_command(%{notify: pid}, _context, state) do
    send(pid, {:executing, self()})
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
