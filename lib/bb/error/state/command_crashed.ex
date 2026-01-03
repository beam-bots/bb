# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.State.CommandCrashed do
  @moduledoc """
  A command crashed during execution.

  This error is returned to callers awaiting a command result when the
  command's callback raises an exception.
  """
  use BB.Error,
    class: :state,
    fields: [:command, :exception]

  @type t :: %__MODULE__{
          command: module(),
          exception: Exception.t()
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{command: cmd, exception: e}) do
    "Command #{inspect(cmd)} crashed: #{Exception.message(e)}"
  end
end
