# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Parameter.Store.Dets do
  @moduledoc """
  DETS-backed parameter persistence.

  Uses OTP's `:dets` module for disk-backed term storage. Parameters are
  persisted automatically on each change and restored on robot startup.

  ## Options

  - `:path` - (required) Path to the DETS file
  - `:auto_save` - Auto-save interval in milliseconds. Defaults to `:infinity` (sync on each write)

  ## Example

  ```elixir
  settings do
    parameter_store {BB.Parameter.Store.Dets, path: "/var/lib/robot/params.dets"}
  end
  ```

  ## File Location

  For production deployments, use an absolute path in a persistent location.
  """

  @behaviour BB.Parameter.Store

  defstruct [:table, :path]

  @type t :: %__MODULE__{
          table: :dets.tab_name(),
          path: String.t()
        }

  @impl true
  def init(robot_module, opts) do
    case Keyword.fetch(opts, :path) do
      {:ok, path} ->
        auto_save = Keyword.get(opts, :auto_save, :infinity)
        table_name = table_name(robot_module)

        case :dets.open_file(table_name, file: to_charlist(path), auto_save: auto_save) do
          {:ok, table} ->
            {:ok, %__MODULE__{table: table, path: path}}

          {:error, reason} ->
            {:error, {:dets_open_failed, reason}}
        end

      :error ->
        {:error, {:missing_option, :path}}
    end
  end

  @impl true
  def load(%__MODULE__{table: table}) do
    parameters =
      :dets.foldl(
        fn {{:param, path}, value}, acc -> [{path, value} | acc] end,
        [],
        table
      )

    {:ok, parameters}
  end

  @impl true
  def save(%__MODULE__{table: table}, path, value) do
    case :dets.insert(table, {{:param, path}, value}) do
      :ok -> :ok
      {:error, reason} -> {:error, {:dets_write_failed, reason}}
    end
  end

  @impl true
  def close(%__MODULE__{table: table}) do
    :dets.close(table)
    :ok
  end

  defp table_name(robot_module) do
    :"bb_params_#{robot_module}"
  end
end
