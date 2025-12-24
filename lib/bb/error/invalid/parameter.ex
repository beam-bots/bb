# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Invalid.Parameter do
  @moduledoc """
  Invalid runtime parameter.

  Raised when a parameter value is invalid (e.g., out of range,
  wrong type, unregistered parameter path).
  """
  use BB.Error,
    class: :invalid,
    fields: [:param_path, :value, :reason]

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{param_path: param_path, value: value, reason: reason}) do
    path_str = format_path(param_path)
    "Invalid parameter #{path_str}: #{reason} (got #{inspect(value)})"
  end

  defp format_path(path) when is_list(path), do: Enum.join(path, ".")
  defp format_path(path), do: inspect(path)
end
