# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.State.Invalid do
  @moduledoc """
  Invalid state reference.

  Raised when attempting to transition to or reference a state that
  is not defined in the robot's DSL.
  """
  use BB.Error,
    class: :invalid,
    fields: [:state, :valid_states]

  @type t :: %__MODULE__{
          state: atom(),
          valid_states: [atom()]
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{state: state, valid_states: valid}) do
    valid_str = Enum.map_join(valid, ", ", &inspect/1)
    "Invalid state #{inspect(state)}. Valid states: #{valid_str}"
  end
end
