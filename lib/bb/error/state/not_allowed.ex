# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.State.NotAllowed do
  @moduledoc """
  Operation not allowed in current state.

  Raised when attempting an operation that is not permitted in the
  robot's current state machine state.
  """
  use BB.Error,
    class: :state,
    fields: [:operation, :current_state, :allowed_states]

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{operation: op, current_state: current, allowed_states: allowed}) do
    allowed_str = Enum.map_join(allowed, ", ", &inspect/1)

    "Operation #{inspect(op)} not allowed: robot is in state #{inspect(current)}, " <>
      "requires one of: #{allowed_str}"
  end
end
