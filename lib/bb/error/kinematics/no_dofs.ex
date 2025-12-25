# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Kinematics.NoDofs do
  @moduledoc """
  Kinematic chain has no degrees of freedom.

  Raised when attempting to solve inverse kinematics for a chain
  that contains only fixed joints and therefore cannot be moved.
  """
  use BB.Error,
    class: :kinematics,
    fields: [:target_link, :chain_length]

  @type t :: %__MODULE__{
          target_link: atom(),
          chain_length: non_neg_integer() | nil
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{target_link: link, chain_length: len}) do
    len_str = if len, do: " (chain length: #{len})", else: ""
    "No degrees of freedom: chain to #{inspect(link)} has no movable joints#{len_str}"
  end
end
