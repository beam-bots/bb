# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Test.MockActuator do
  @moduledoc """
  Minimal mock actuator for testing.
  """
  use GenServer

  def start_link(opts) do
    GenServer.start_link(__MODULE__, opts, name: via(opts))
  end

  defp via(opts) do
    BB.Process.via(opts[:bb][:robot], opts[:bb][:name])
  end

  @impl GenServer
  def init(opts) do
    {:ok, %{opts: opts}}
  end

  @impl GenServer
  def handle_cast({:command, _message}, state) do
    {:noreply, state}
  end

  @impl GenServer
  def handle_call({:command, _message}, _from, state) do
    {:reply, {:ok, :accepted}, state}
  end

  @impl GenServer
  def handle_info({:bb, _path, _message}, state) do
    {:noreply, state}
  end
end
