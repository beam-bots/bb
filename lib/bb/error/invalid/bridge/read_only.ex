# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Invalid.Bridge.ReadOnly do
  @moduledoc """
  Attempted to write to a read-only parameter.

  Some servo parameters (like model number, firmware version) are read-only
  and cannot be modified.
  """
  use BB.Error,
    class: :invalid,
    fields: [:param_name]

  @type t :: %__MODULE__{
          param_name: atom()
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{param_name: param_name}) do
    "Cannot write to read-only parameter: #{inspect(param_name)}"
  end
end
