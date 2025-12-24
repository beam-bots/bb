# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Invalid.Command do
  @moduledoc """
  Invalid command or command arguments.

  Raised when a command is unknown or its arguments are invalid.
  """
  use BB.Error,
    class: :invalid,
    fields: [:command, :argument, :value, :reason]

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{command: cmd, argument: nil, value: nil, reason: reason}) do
    "Invalid command #{inspect(cmd)}: #{reason}"
  end

  def message(%{command: cmd, argument: arg, value: value, reason: reason}) do
    "Invalid command #{inspect(cmd)} argument #{inspect(arg)}: " <>
      "#{reason} (got #{inspect(value)})"
  end
end
