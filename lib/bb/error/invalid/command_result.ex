# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Invalid.CommandResult do
  @moduledoc """
  A command's `result/1` returned a value outside its contract.

  `c:BB.Command.result/1` must return `{:ok, result}`, `{:ok, result, options}`
  or `{:error, reason}`. Anything else is replaced with this error before it
  reaches awaiting callers or the runtime, so a handler's mistake fails the
  command rather than the robot.
  """
  use BB.Error,
    class: :invalid,
    fields: [:command, :value]

  @typedoc """
  `:command` is the handler module, or the command's DSL name where the
  handler isn't to hand — the runtime only knows a running command by name.
  """
  @type t :: %__MODULE__{
          command: atom(),
          value: term()
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{command: command, value: value}) do
    "Command #{inspect(command)} result/1 returned #{inspect(value)}, " <>
      "expected {:ok, result}, {:ok, result, options} or {:error, reason}"
  end
end
