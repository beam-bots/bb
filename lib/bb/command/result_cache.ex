# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Command.ResultCache do
  @moduledoc """
  Caches command results for retrieval after the command process terminates.

  This handles the race condition where a fast command completes before
  `await/2` is called. The Command.Server stores its result here before
  terminating, and `await/2` can retrieve it if the process is already dead.

  Results are automatically cleaned up after a configurable TTL.
  """

  use GenServer

  @table_name :bb_command_result_cache
  @default_ttl_ms 60_000
  @cleanup_interval_ms 30_000

  @doc """
  Starts the result cache.
  """
  def start_link(opts \\ []) do
    GenServer.start_link(__MODULE__, opts, name: __MODULE__)
  end

  @doc """
  Stores a command result, keyed by the command process pid.

  Called by Command.Server in terminate/2 before the process exits.
  """
  @spec store(pid(), term()) :: :ok
  def store(pid, result) do
    expiry = System.monotonic_time(:millisecond) + @default_ttl_ms
    :ets.insert(@table_name, {pid, result, expiry})
    :ok
  end

  @doc """
  Fetches and removes a cached result for the given pid.

  Returns `{:ok, result}` if found, `:error` if not cached.
  """
  @spec fetch_and_delete(pid()) :: {:ok, term()} | :error
  def fetch_and_delete(pid) do
    case :ets.take(@table_name, pid) do
      [{^pid, result, _expiry}] -> {:ok, result}
      [] -> :error
    end
  end

  @impl GenServer
  def init(_opts) do
    :ets.new(@table_name, [:named_table, :public, :set])
    schedule_cleanup()
    {:ok, %{}}
  end

  @impl GenServer
  def handle_info(:cleanup, state) do
    now = System.monotonic_time(:millisecond)

    # Delete expired entries
    :ets.select_delete(@table_name, [
      {{:_, :_, :"$1"}, [{:<, :"$1", now}], [true]}
    ])

    schedule_cleanup()
    {:noreply, state}
  end

  defp schedule_cleanup do
    Process.send_after(self(), :cleanup, @cleanup_interval_ms)
  end
end
