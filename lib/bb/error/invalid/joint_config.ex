# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Invalid.JointConfig do
  @moduledoc """
  Invalid joint configuration.

  Raised when joint configuration is invalid (e.g., missing limits,
  invalid joint type for actuator, incompatible settings).
  """
  use BB.Error,
    class: :invalid,
    fields: [:joint, :field, :value, :expected, :message]

  @type t :: %__MODULE__{
          joint: atom(),
          field: atom() | nil,
          value: term(),
          expected: term(),
          message: String.t() | nil
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{joint: joint, field: field, value: _value, expected: _expected, message: msg})
      when not is_nil(msg) do
    "Invalid joint configuration for #{inspect(joint)}.#{field}: #{msg}"
  end

  def message(%{joint: joint, field: field, value: value, expected: expected, message: nil}) do
    "Invalid joint configuration for #{inspect(joint)}.#{field}: " <>
      "got #{inspect(value)}, expected #{inspect(expected)}"
  end
end
