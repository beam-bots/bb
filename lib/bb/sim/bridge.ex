# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Sim.Bridge do
  @moduledoc """
  Mock bridge for simulation mode.

  Accepts all operations but does nothing. Useful when actuators or other
  components query the bridge during initialisation in simulation mode.
  """
  use BB.Bridge, options_schema: []

  @impl GenServer
  def init(opts) do
    {:ok, %{bb: opts[:bb]}}
  end

  @impl BB.Bridge
  def handle_change(_robot, _changed, state) do
    {:ok, state}
  end

  @impl BB.Bridge
  def list_remote(state) do
    {:ok, [], state}
  end

  @impl BB.Bridge
  def get_remote(_param_id, state) do
    {:error, :not_found, state}
  end

  @impl BB.Bridge
  def set_remote(_param_id, _value, state) do
    {:ok, state}
  end

  @impl BB.Bridge
  def subscribe_remote(_param_id, state) do
    {:ok, state}
  end

  @impl GenServer
  def handle_call(:list_remote, _from, state) do
    {:ok, params, state} = list_remote(state)
    {:reply, {:ok, params}, state}
  end

  def handle_call({:get_remote, param_id}, _from, state) do
    {:error, reason, state} = get_remote(param_id, state)
    {:reply, {:error, reason}, state}
  end

  def handle_call({:set_remote, param_id, value}, _from, state) do
    {:ok, state} = set_remote(param_id, value, state)
    {:reply, :ok, state}
  end

  def handle_call({:subscribe_remote, param_id}, _from, state) do
    {:ok, state} = subscribe_remote(param_id, state)
    {:reply, :ok, state}
  end

  def handle_call(_request, _from, state) do
    {:reply, :ok, state}
  end

  @impl GenServer
  def handle_cast(_request, state) do
    {:noreply, state}
  end

  @impl GenServer
  def handle_info(_msg, state) do
    {:noreply, state}
  end
end
