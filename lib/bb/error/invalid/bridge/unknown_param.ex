# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Invalid.Bridge.UnknownParam do
  @moduledoc """
  Unknown parameter name for bridge parameter.

  The parameter name is not defined in the servo's control table.
  """
  use BB.Error,
    class: :invalid,
    fields: [:param_name, :control_table]

  @type t :: %__MODULE__{
          param_name: atom(),
          control_table: module() | nil
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{param_name: param_name, control_table: nil}) do
    "Unknown parameter: #{inspect(param_name)}"
  end

  def message(%{param_name: param_name, control_table: control_table}) do
    "Unknown parameter: #{inspect(param_name)} not found in #{inspect(control_table)}"
  end
end
